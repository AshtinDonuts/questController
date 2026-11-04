"""Configured camera acquisition; each device is opened exactly once."""
import queue
import threading
import time

import numpy as np


class Camera:
    def __init__(self, name, config, capacity):
        self.name, self.config = name, config
        self.frames = queue.Queue(capacity)
        self.stop = threading.Event()
        self.error = None
        self.metadata = {}
        self.latest = {}
        self.thread = threading.Thread(target=self._guarded, daemon=True, name=f'camera-{name}')
        self.thread.start()

    def emit(self, stream, image, frame_number, device_ms=0., domain='host_monotonic'):
        image = np.ascontiguousarray(image).copy()
        expected = (self.config['height'], self.config['width']) + ((3,) if stream == 'rgb' else ())
        if image.shape != expected or image.dtype != (np.uint8 if stream == 'rgb' else np.uint16):
            raise ValueError(f'{self.name}/{stream}: camera changed shape or dtype')
        frame = dict(t_ns=time.monotonic_ns(), image=image, frame_number=frame_number,
                     device_ms=float(device_ms), timestamp_domain=domain)
        self.metadata.setdefault('timestamp_domains', {})[stream] = domain
        self.latest[stream] = frame['t_ns']
        self.frames.put_nowait((stream, frame))

    def _guarded(self):
        try:
            getattr(self, '_'+self.config['backend'])()
        except queue.Full:
            self.error=f'{self.name}: capture queue overflow; source frames were not silently dropped'
        except Exception as exc:
            self.error = f'{self.name}: {type(exc).__name__}: {exc}'

    def _synthetic(self):
        cfg = self.config
        self.metadata = dict(mock=True, width=cfg['width'], height=cfg['height'], depth_scale_m=.001)
        index = 0
        deadline = time.monotonic()
        while not self.stop.is_set():
            for stream in cfg['streams']:
                shape = (cfg['height'], cfg['width']) + ((3,) if stream == 'rgb' else ())
                image = np.full(shape, index % (256 if stream == 'rgb' else 65536),
                                dtype=np.uint8 if stream == 'rgb' else np.uint16)
                self.emit(stream, image, index)
            index += 1
            deadline += 1/cfg['fps']
            self.stop.wait(max(0, deadline-time.monotonic()))

    def _opencv(self):
        import cv2
        cfg = self.config
        cap = cv2.VideoCapture(cfg['device'])
        try:
            if not cap.isOpened():
                raise RuntimeError('Cannot open configured OpenCV device')
            for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, cfg['width']),
                                (cv2.CAP_PROP_FRAME_HEIGHT, cfg['height']), (cv2.CAP_PROP_FPS, cfg['fps'])):
                cap.set(prop, value)
            self.metadata = dict(device=cfg['device'], reported_fps=cap.get(cv2.CAP_PROP_FPS),
                                 timing='host reception; no device exposure timestamp')
            index = 0
            while not self.stop.is_set():
                ok, image = cap.read()
                if not ok:
                    raise RuntimeError('Camera read failed')
                self.emit('rgb', cv2.cvtColor(image, cv2.COLOR_BGR2RGB), index)
                index += 1
        finally:
            cap.release()

    def _realsense(self):
        import pyrealsense2 as rs
        cfg = self.config
        settings = rs.config()
        settings.enable_device(cfg['device'])
        for stream in cfg['streams']:
            settings.enable_stream(rs.stream.color if stream == 'rgb' else rs.stream.depth,
                                   cfg['width'], cfg['height'],
                                   rs.format.rgb8 if stream == 'rgb' else rs.format.z16, cfg['fps'])
        pipeline = rs.pipeline()
        profile = pipeline.start(settings)
        try:
            self.metadata = dict(serial=profile.get_device().get_info(rs.camera_info.serial_number), intrinsics={})
            for stream in cfg['streams']:
                intr = profile.get_stream(rs.stream.color if stream == 'rgb' else rs.stream.depth).as_video_stream_profile().get_intrinsics()
                self.metadata['intrinsics'][stream] = dict(width=intr.width, height=intr.height,
                    fx=intr.fx, fy=intr.fy, ppx=intr.ppx, ppy=intr.ppy, coeffs=list(intr.coeffs), model=str(intr.model))
            if 'depth' in cfg['streams']:
                self.metadata['depth_scale_m'] = profile.get_device().first_depth_sensor().get_depth_scale()
            if set(cfg['streams']) == {'rgb', 'depth'}:
                extr = profile.get_stream(rs.stream.depth).get_extrinsics_to(profile.get_stream(rs.stream.color))
                self.metadata['depth_to_rgb'] = dict(rotation=list(extr.rotation), translation=list(extr.translation))
            last_frames={}
            while not self.stop.is_set():
                frames = pipeline.wait_for_frames(1000)
                for stream in cfg['streams']:
                    frame = frames.get_color_frame() if stream == 'rgb' else frames.get_depth_frame()
                    if not frame:
                        raise RuntimeError(f'Missing {stream} frame')
                    number=frame.get_frame_number()
                    if number==last_frames.get(stream):
                        continue  # A frameset may reuse the other stream's frame.
                    last_frames[stream]=number
                    self.emit(stream, np.asanyarray(frame.get_data()), frame.get_frame_number(),
                              frame.get_timestamp(), str(frame.get_frame_timestamp_domain()))
        finally:
            pipeline.stop()

    def healthy(self, now_ns):
        return not self.error and all(0 <= now_ns-self.latest.get(s, 0) < self.config['max_gap_s']*1e9
                                      for s in self.config['streams'])

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.5)

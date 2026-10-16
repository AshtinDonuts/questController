"""Local session MP4, timestamped telemetry and A/B episode markers."""
import json
from pathlib import Path
import subprocess
import mujoco
from .frames import draw_eef_frame, draw_planar_bounds


class Recording:
    def __init__(self, directory, model, fps, metadata, planar_limits=(), video_filter=None, camera='teleoperator_pov'):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = metadata
        self.camera = camera
        self.planar_limits = planar_limits
        self.fps = fps
        self.frames = 0
        self.episodes = []
        self.active = None
        self.stream = (self.directory / 'telemetry.jsonl').open('w')
        self.sites = [i for i in range(model.nsite) if model.site(i).name in ('right/gripper', 'left/gripper')]
        self.renderer = mujoco.Renderer(model, height=480, width=640)
        self.encoder = subprocess.Popen(
            ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
             '-s', '640x480', '-r', str(fps), '-i', '-', '-an',
             *(['-vf', video_filter] if video_filter else []), '-c:v', 'libx264',
             '-preset', 'ultrafast', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
             str(self.directory / 'session.mp4')], stdin=subprocess.PIPE)
        self._manifest('recording')

    def _manifest(self, state):
        payload = {**self.metadata, 'status': state, 'fps': self.fps, 'frames': self.frames,
                   'episodes': self.episodes, 'active_episode': self.active,
                   'timing': 'MP4 nominal frame rate; telemetry host timestamps preserve actual timing.'}
        path = self.directory / 'manifest.json'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(payload, indent=2) + '\n')
        temporary.replace(path)

    def start_episode(self):
        self.active = self.frames
        self._manifest('recording')

    def finish_episode(self, saved):
        if self.active is not None:
            self.episodes.append(dict(start_frame=self.active, end_frame_exclusive=self.frames, saved=saved))
            self.active = None
            self._manifest('recording')

    def write(self, data, row):
        self.renderer.update_scene(data, camera=self.camera)
        for site in self.sites:
            draw_eef_frame(self.renderer.scene, data, site)
        for planes in self.planar_limits:
            draw_planar_bounds(self.renderer.scene, planes)
        self.encoder.stdin.write(self.renderer.render().tobytes())
        self.stream.write(json.dumps(dict(frame=self.frames, **row), allow_nan=False) + '\n')
        self.stream.flush()
        self.frames += 1

    def close(self, status='completed'):
        self.finish_episode(False)  # Incomplete episode retained in video, marked discarded.
        self.stream.close()
        self.renderer.close()
        self.encoder.stdin.close()
        code = self.encoder.wait(timeout=30)
        self._manifest('encoder_failed' if code else status)
        if code:
            raise RuntimeError(f'ffmpeg exited with code {code}')

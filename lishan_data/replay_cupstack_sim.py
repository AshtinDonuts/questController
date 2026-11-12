"""Offline cupstack kinematic replay; never imports motor or supervisor code.

Run with the lerobot Python. Rendering uses the installed quest_ros MuJoCo.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
JOINTS = ('waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate')


def render(bundle, output):
    os.environ.setdefault('MUJOCO_GL', 'egl')
    import mujoco
    import numpy as np
    values = np.load(bundle)
    model = mujoco.MjModel.from_xml_path(str(PROJECT / 'scenes/dual_viperx_scene.xml'))
    signature = json.loads((ROOT / 'cupstack/lerobot_recording.json').read_text())['signature']
    rig = json.loads((PROJECT / 'webxr_teleop/hardware/rig.json').read_text())
    for side in ('left', 'right'):
        body = model.body(side + '/base_link').id
        model.body_pos[body] = signature['arms'][side]['base_position']
        model.body_quat[body] = signature['arms'][side]['base_quaternion_wxyz']
    data = mujoco.MjData(model)
    fps = float(values['fps'])
    command = ['ffmpeg', '-v', 'error', '-n', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', '1280x480', '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264',
               '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, .27, .23]
    camera.distance = 1.65
    camera.azimuth = -90
    camera.elevation = -32
    max_error = 0.
    limit_violations = {}
    try:
        with mujoco.Renderer(model, height=480, width=640) as renderer:
            for frame in range(len(values['action'])):
                images = []
                for key in ('action', 'state'):
                    q = values[key][frame]
                    for arm, side in enumerate(('left', 'right')):
                        for j, name in enumerate(JOINTS):
                            joint = model.joint(side + '/' + name)
                            value = float(q[arm * 7 + j])
                            data.qpos[joint.qposadr[0]] = value
                            if not joint.range[0] <= value <= joint.range[1]:
                                label = key + ':' + side + '.' + name
                                limit_violations[label] = limit_violations.get(label, 0) + 1
                        a = rig['arms'][side]
                        opening = a['finger_closed_m'] + float(q[arm * 7 + 6]) * (a['finger_open_m'] - a['finger_closed_m'])
                        for finger in ('left', 'right'):
                            data.qpos[model.joint(side + '/' + finger + '_finger').qposadr[0]] = opening
                    # Kinematic replay preserves recorded targets exactly, without physics drift.
                    mujoco.mj_forward(model, data)
                    for arm, side in enumerate(('left', 'right')):
                        for j, name in enumerate(JOINTS):
                            actual = data.qpos[model.joint(side + '/' + name).qposadr[0]]
                            max_error = max(max_error, abs(actual - q[arm * 7 + j]))
                    renderer.update_scene(data, camera=camera)
                    images.append(renderer.render().copy())
                process.stdin.write(np.concatenate(images, axis=1).tobytes())
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError('Video encoding failed')
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait()
    report = dict(frames=len(values['action']), fps=fps, max_joint_assignment_error_rad=max_error,
                  joint_limit_violations=limit_violations, replay='kinematic; no object/contact simulation')
    output.with_suffix('.validation.json').write_text(json.dumps(report, indent=2) + '\n')


def prepare(episode):
    import numpy as np
    import pyarrow.parquet as pq
    dataset = ROOT / 'cupstack/lerobot'
    info = json.loads((dataset / 'meta/info.json').read_text())
    expected = [s + '.' + j for s in ('left', 'right') for j in (*JOINTS, 'gripper')]
    for key in ('action', 'observation.state'):
        if info['features'][key]['names'] != expected:
            raise ValueError('Unexpected joint ordering: ' + key)
    episodes = []
    for path in sorted((dataset / 'meta/episodes').rglob('*.parquet')):
        for row in pq.read_table(path).to_pylist():
            path_data = dataset / info['data_path'].format(chunk_index=row['data/chunk_index'], file_index=row['data/file_index'])
            if path_data.exists():
                episodes.append((row, path_data))
    candidates = [item for item in episodes if episode is None or item[0]['episode_index'] == episode]
    if not candidates:
        raise ValueError('No saved episode with available data')
    meta, path_data = max(candidates, key=lambda item: item[0]['episode_index'])
    episode = meta['episode_index']
    table = pq.read_table(path_data)
    rows = [r for r in table.to_pylist() if r['episode_index'] == episode]
    rows.sort(key=lambda r: r['frame_index'])
    if len(rows) != meta['length']:
        raise ValueError('Episode length does not match metadata')
    action = np.array([r['action'] for r in rows])
    state = np.array([r['observation.state'] for r in rows])
    times = np.array([r['timestamp'] for r in rows])
    if not np.isfinite(action).all() or not np.isfinite(state).all() or not np.all(np.diff(times) > 0):
        raise ValueError('Invalid joint values or timestamps')
    if not np.allclose(times - times[0], np.arange(len(rows)) / info['fps'], atol=1e-4):
        raise ValueError('Nonuniform episode timing')
    for q in (action, state):
        if (q[:, [6, 13]] < 0).any() or (q[:, [6, 13]] > 1).any():
            raise ValueError('Invalid calibrated gripper aperture')
    out = ROOT / 'sim_replay' / f'episode_{episode:03d}'
    out.mkdir(parents=True, exist_ok=True)
    bundle = out / 'trajectory.npz'
    np.savez(bundle, action=action, state=state, timestamp=times, fps=info['fps'])
    sim = out / 'simulation.mp4'
    if sim.exists():
        raise FileExistsError(f'Output already exists: {sim}')
    subprocess.run(['/home/King/miniconda3/envs/quest_ros/bin/python', str(Path(__file__).resolve()),
                    '--render', str(bundle), str(sim)], check=True)
    video_key = 'observation.images.top'
    source = dataset / info['video_path'].format(video_key=video_key,
        chunk_index=meta[f'videos/{video_key}/chunk_index'], file_index=meta[f'videos/{video_key}/file_index'])
    comparison = out / 'comparison.mp4'
    filters = "[0:v]drawtext=text='COMMAND action':x=15:y=15:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6,drawtext=text='MEASURED state':x=655:y=15:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6[s];[1:v]scale=640:480,drawtext=text='RECORDED top camera':x=15:y=15:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.6[v];[s][v]hstack=inputs=2[out]"
    subprocess.run(['ffmpeg', '-v', 'error', '-n', '-i', str(sim), '-ss', str(meta[f'videos/{video_key}/from_timestamp']),
                    '-i', str(source), '-filter_complex', filters, '-map', '[out]', '-frames:v', str(len(rows)),
                    '-r', str(info['fps']), '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(comparison)], check=True)
    (out / 'index.html').write_text('''<!doctype html><meta charset="utf-8"><title>Cupstack replay</title>
<style>body{margin:24px;background:#101822;color:#e8edf5;font:17px system-ui}video{width:100%;background:#000}button,select{font:inherit;padding:8px;margin-right:12px}p{color:#b9c8dc}</style>
<h1>Cupstack · Episode ''' + str(episode) + '''</h1>
<p>左：指令 action ｜ 中：实测 observation.state ｜ 右：原始俯视录像。双臂间距 0.64 m。</p>
<video id="v" controls autoplay muted loop src="comparison.mp4"></video>
<p><button onclick="v.paused?v.play():v.pause()">播放 / 暂停</button><button onclick="v.pause();v.currentTime=Math.max(0,v.currentTime-1/15)">上一帧</button><button onclick="v.pause();v.currentTime+=1/15">下一帧</button>
<select aria-label="播放速度" onchange="v.playbackRate=Number(this.value)"><option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1" selected>1×</option></select><span id="t"></span></p>
<p>离线 MuJoCo 关节运动回放（15 Hz）；未模拟杯子、抓取接触或动力学。数据已是模型弧度，不重复应用电机符号/偏移。</p>
<script>v.ontimeupdate=()=>t.textContent=v.currentTime.toFixed(2)+' / '+v.duration.toFixed(2)+' s';</script>''')
    print(json.dumps(dict(episode=episode, frames=len(rows), duration_s=len(rows)/info['fps'], output=str(out)), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=int, help='Saved episode index; default latest available')
    parser.add_argument('--render', nargs=2, metavar=('BUNDLE', 'VIDEO'), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.render:
        render(Path(args.render[0]), Path(args.render[1]))
    else:
        prepare(args.episode)

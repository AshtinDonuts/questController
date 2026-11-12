# HDF5 and LeRobot v3 demonstration recording

Dataset recording is opt-in on the hardware supervisor. Motor workers keep sole
ownership of the arms; a separate process owns cameras and dataset writing. The existing
measured-feedback twin and diagnostic session video continue independently.

## Configuration

Add `--recording-config /absolute/path/recording.json` to an existing launch that
uses `--sequence`. `--recording-python` selects the acquisition interpreter and
defaults to `/home/King/miniconda3/envs/lerobot/bin/python`. That environment
already contains the pinned acquisition packages in
`requirements-dataset-recording.txt`. The Quest supervisor does not import them.

Before every hardware launch or launch-command handoff, follow `AGENTS.md`:
inspect the configured control socket, supervisor/motor workers and HTTPS port;
resolve active ownership through powered hold/shutdown; remove only a confirmed
owned stale socket whose identity has not changed. Keep `--twin` explicit on
live commands. Recording configuration never authorizes a physical launch.

Example configuration (replace device identifiers before use):

```json
{
  "dataset_dir": "./demonstrations/pick_place",
  "task": "Place the red block in the tray",
  "display_data": true,
  "episode_time_s": 0,
  "cameras": {
    "front": {
      "backend": "realsense",
      "device": "YOUR_CAMERA_SERIAL",
      "width": 640,
      "height": 480,
      "fps": 30,
      "streams": ["rgb", "depth"]
    },
    "wrist": {
      "backend": "opencv",
      "device": "/dev/v4l/by-id/YOUR_CAMERA",
      "width": 640,
      "height": 480,
      "fps": 30,
      "streams": ["rgb"]
    }
  }
}
```

There is **no default camera selection**. `cameras` is required; an explicit `{}`
records proprioception only. Configure each physical device once, listing all
its enabled streams. RealSense captures RGB and/or raw depth with intrinsics,
depth scale and depth-to-RGB extrinsics. OpenCV captures RGB with host reception
timestamps. Serial numbers and shapes are never inherited from the motor rig.
Paths in recording JSON are relative to that JSON file. Camera names are simple
identifiers; resolution and FPS are required. Unsupported shapes fail capture.

Startup checks configured RealSense serials against connected devices before
opening streams. A missing device raises an error naming the camera and serial
(for example, `Required recording camera not connected: top (RealSense serial
231622302829)`). Check USB connection/power and the configured serial. Acquisition
errors identify the camera/device; heartbeat and warmup timeouts include the last
reported stage of cameras that are not ready. Motor workers start only after
recording camera warmup succeeds. The supervisor allows up to 10 seconds for
warmup, including discovery after the initial “checking connection” status.
The configured `heartbeat_timeout_s` applies after warmup completes; an early
warming heartbeat does not shorten startup to the runtime deadline. Repeated
warming updates cannot extend the startup deadline indefinitely.

`webxr_teleop/datasets/example_mock.json` uses synthetic RGB/depth and is accepted
only with mock motors. It is a test configuration, not a physical camera default.

Optional settings: `queue_frames` (8 per stream), `flush_interval_s` (1),
`heartbeat_timeout_s` (1), and per-camera `max_gap_s` (the larger of 0.2 seconds
and three camera periods). `episode_time_s: 0` means manual boundaries; a positive value automatically saves
at that duration in seconds. `num_episodes` is the target number of saved HDF5
episodes in the dataset directory, including existing accepted episodes (default
`0`: unlimited). It must be a nonnegative integer. Discarded and aborted episodes
do not count. At the target, new episode/return requests are rejected and arms
stay held; sleep remains an explicit operator request. No automatic next episode
or return is triggered by either limit.

## Operator workflow

1. Wait for camera warmup and the initial opening handoff (arrival or timeout).
   Release selected grips and press Quest left X or terminal X to align.
   Recording uses the current stationary pose; it does not require opening arrival.
   Only selected-arm controllers must remain tracked. Left-only uses left-controller
   heading; right-only/dual uses right-controller heading.
2. Press right A or terminal →. Wait for REC before moving. Selected grips must
   be released and the arms stationary when starting. After alignment, capture
   can start from the current pose; opening arrival is not a recording prerequisite.
3. Press A/→ to save. Capture ends at that boundary and the supervisor
   immediately holds arm joints, then opens the grippers fully under the configured
   aperture speed/acceleration limits. The file finalizes asynchronously.
4. After save/discard and full gripper opening/settling, A or terminal ↑ explicitly
   requests return to opening. Grippers move to each arm's saved `opening_gripper`
   ceremony aperture. Reset the objects, wait for arrival, press X
   to align, then press A to record again. No countdown starts another episode.

Terminal → starts/saves; ↑ requests return to opening; B / ← discards and returns to opening.
Escape, q, Ctrl+C and SIGTERM request sleep, wait for measured arrival and
confirmed torque-off, then exit. A stop/fault, stale feedback or interrupted sleep
retains powered hold; a second exit request exits immediately in hold. Return uses a separate arrow so
a repeated save key cannot initiate return motion.
**Terminal Space remains an immediate powered
stop** and abort an active episode. Hardware faults,
writer failure, recording queue overflow and stale required cameras also abort.
Sleep and shutdown abort unfinished episodes. Stop has priority over save/start.
Fault recovery continues to use the existing controls; recorder failure requires
resolving the cause and starting a new session, with no automatic motion resume.

The existing local control interface accepts `record-start`, `record-save`,
`record-discard`, and `record-next`. For a save, `--outcome success` or
`--outcome failure` supplies an explicit label; the default is `unknown`.
`--request-id` provides an idempotency key. The same recording command ID is
executed at most once during a supervisor session. Commands are asynchronous;
poll `status` and inspect its `recording` object for phase, result and errors.

Rerun shows configured cameras, named measured/confirmed-command joint plots,
gripper aperture and feedback delivery age. The Quest HUD shows REC, elapsed
time, task, accepted count and stream warnings. Preview queues can drop display
updates independently of acquisition. Closing the viewer does not end recording.

## Source files and timing

```
dataset_dir/
  accepted/episode_000000.hdf5
  discarded/episode_000001.hdf5
  aborted/episode_000002.hdf5
  episode_000003.partial
```

Each episode is self-contained and uses `quest_hdf5.v1`:

| Group | Native data |
| --- | --- |
| `/metadata/json` | Task, camera configuration/calibration, effective rig, per-arm calibration snapshots/fingerprints, source revision and Python-source hashes. |
| `/streams/robot/<side>/observations` | Measured `qpos`, host `t_ns`, worker `seq`, epoch and full feedback JSON including available raw encoders/health. |
| `/streams/robot/<side>/actions` | Confirmed post-quantization `qpos`, cycle timestamp, worker sequence and JSON with raw goals, original confirmation time, write flag and requested target. |
| `/streams/cameras/<name>/<rgb-or-depth>` | Lossless chunked image arrays, host timestamp, device timestamp and frame number. |
| `/streams/controllers/<side>` | Controller packets, host receipt and freshness-token issuance times. |
| `/events` | Boundaries, control transitions, gaps and reasons. |

Streams have independent lengths. Robot observations precede their cycle's
action write. An unchanged goal retains the earlier motor readback confirmation;
it is not labelled as a new write. No extra serial reads are added for capture.
Motor transport sequence gaps abort; camera frame-number gaps are recorded as
events. Missing camera frames are never filled in the source.

Per arm, model order is `waist, shoulder, elbow, forearm_roll, wrist_angle,
wrist_rotate, gripper`. Arm values are radians; gripper is calibrated aperture
(`0=closed`, `1=open`). Raw motor observations retain shadow motors separately.
RGB uses `uint8`; depth uses raw `uint16` plus metres-per-count calibration.
Images use lossless LZF compression. Gripper aperture is not a measured jaw gap.

Cross-stream timing uses host monotonic nanoseconds, with a UTC clock anchor.
Camera device timestamps/domains are retained separately. This is host reception
alignment, not hardware-synchronized exposure. Quest freshness issuance is not
misrepresented as controller capture time. Original measured mounting transforms
and the operator-copy provenance for right-arm settings remain in the snapshot.

One process holds the dataset lock. After capture ends, a single background
finalizer takes sole ownership of the HDF5 file while acquisition drains preview
frames and publishes heartbeats. Stale-camera start requests are rejected without
terminating the session and can be retried when streams recover.

Completed files are validated and renamed
from `.partial` after flush/close/fsync. Existing files are never overwritten;
new sessions allocate IDs above all prior attempts. Interrupted `.partial` files
remain quarantined, including unreadable files. A periodic flush is not a crash
recovery guarantee and does not make a partial file an accepted episode.

Validate source episodes with the acquisition interpreter:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.hdf5 /absolute/path/to/dataset
```

## Replay saved episodes

From the repository root, open the most recently saved episode in Rerun:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.replay
```

The default dataset comes from `recording.json`. Pass an HDF5 filename to select
an episode, or a dataset/`accepted` directory to open its newest saved file.
Add `--list` to list filenames, durations, status and outcome without opening a
window. Explicit aborted/discarded `.hdf5` files can also be inspected; unfinished
`.partial` files are not supported.

Select the **episode** timeline at the bottom of Rerun, press Play, or drag the
playhead to scrub. Rerun provides playback speed and loop controls. Camera RGB,
calibrated depth and measured/confirmed-command joint plots retain their native
timestamps relative to episode start. This is camera and telemetry replay, not
a 3D robot animation. Loading reads images one frame at a time; the viewer keeps
up to 2 GB in memory, so very large episodes may evict older data.

Use `--save /path/to/new-recording.rrd` for a headless Rerun export that can be
opened later. Existing output files are refused. Replay only reads HDF5 files;
it does not connect to motors, cameras or the running hardware supervisor.

## Native LeRobot v3 recording

HDF5 remains the default. Select native LeRobot v3 output with the project-root
`recording_lerobot_v3.json`, or add these fields to a copy of your recording JSON:

```json
{
  "dataset_dir": "./demonstrations/pick_place_v3",
  "format": "lerobot_v3",
  "lerobot": {
    "repo_id": "local/pick-place",
    "fps": 30,
    "robot_tolerance_s": 0.04,
    "camera_tolerance_s": null
  },
  "task": "Place the red block in the tray",
  "display_data": false,
  "cameras": {}
}
```

Copy your camera map into `cameras` to include RGB/depth. The supplied root
configuration copies the current left-arm and top camera settings from
`recording.json`, uses a new output directory, and leaves preview disabled.
`repo_id` identifies the local dataset; recording never uploads it.

Install `requirements-dataset-lerobot.txt` in a separate Python 3.12+ environment.
This includes the pinned LeRobot 0.6.1 writer, camera SDK and a NumPy 2 compatible
Rerun 0.22.1 preview. Do not upgrade the existing motor-driver or HDF5 recording
environment. Select this interpreter with the existing `recording-python` option
and the new JSON with `recording-config` on your normal recording launch. Follow
the endpoint checks above and keep the measured-feedback `twin` option explicit
on live launches. This configuration does not start hardware by itself.

The training dataset is **`dataset_dir/lerobot/`**, directly readable by the
official `LeRobotDataset` loader. It contains v3.0 `meta/`, Parquet data shards
and RGB/depth videos. State/action ordering, units, causal alignment and timing
tolerances match the offline exporter below. Each pause is aligned separately;
wall-clock pauses are removed from the training timeline. Uniform-rate alignment
and video encoding are not lossless native-rate capture; choose HDF5 when the
original asynchronous streams and lossless pixels are required.

Start, save, discard, pause, abort and episode limits use the existing controls.
During capture, training streams are staged in a disk-backed SQLite attempt
journal, without HDF5 or an episode-sized image buffer. Saving aligns those
streams, feeds the official writer through bounded video queues, and finalizes
the dataset before acknowledging success. The background finalizer keeps camera
queues and recorder heartbeats serviced. A healthy save can exceed five seconds;
shutdown allows up to 120 seconds for an in-flight save after motor shutdown.

Accepted attempts leave provenance/alignment reports in
`dataset_dir/attempts/episode_NNNNNN.saved.json`; their staging files are removed.
Discarded/aborted attempt journals stay in `attempts/` and never enter training
data. Unfinished `.partial.sqlite` journals remain quarantined after a crash;
they are not automatically recovered or accepted. A `.committing` marker blocks
further appends after an interrupted commit: inspect both the attempt and the
LeRobot dataset before repairing or restoring it. A commit is not an atomic
transaction across all LeRobot files. Disk usage grows with capture duration;
periodic SQLite commits do not guarantee recovery of the most recent samples.

Restarting with a compatible configuration appends episodes and restores the
saved count. Changes to active arms, mock/live mode, rig mapping, base transforms,
camera schema, dataset identity or output FPS require a new directory. Use separate
`dataset_dir` values for HDF5 and LeRobot recording. The existing HDF5 replay and
older ACT training bridge remain HDF5-only; load native v3 output with modern
LeRobot tools instead.

## Offline LeRobot v3 export

For ACT training with the currently installed older LeRobot fork, use the
[native recording to ACT bridge](act_training_bridge.md). It preserves this
recording format and prepares a separate v1.6 training copy, including RGB
selection, action chunks, pause boundaries, normalization, and offline training.

Use a separate Python 3.12+ virtual environment and install
`requirements-dataset-export.txt`. It pins LeRobot 0.6.1 and CPU PyTorch; do not
upgrade the working motor-driver environment. For this implementation's checks,
the isolated environment is `/tmp/quest-lerobot-v3` (temporary, not a permanent
installation).

The supplied hardware `launch.json` selects this environment with
`recording_python: /tmp/quest-lerobot-v3/bin/python`, so native cupstack recording
uses LeRobot 0.6.1 while motor workers keep their existing interpreter. This
temporary environment must exist; use `--recording-python /path/to/python` to
override it when moving to a permanent installation.

```bash
/path/to/export-venv/bin/python -m webxr_teleop.datasets.export_lerobot \
  /absolute/path/to/dataset /absolute/new/export-directory \
  --repo-id local/pick-place --fps 30
```

The source can be a dataset root, its `accepted/` directory, or a single saved
HDF5 episode. The destination must not already exist. Only accepted, validated
episodes are exported. Mock and live episodes must be
exported separately; mixed recording modes are rejected before output creation.
Incompatible arm mappings,
mounting geometry or camera schemas require separate exports. The source files
are never modified. Exporting does not upload data or connect to robot devices.

The exporter trims to common stream coverage. At each uniform grid time it
selects preceding robot observations/camera frames and the next per-arm motor
cycle's confirmed target. Robot tolerance defaults to 40 ms; camera tolerance
defaults to two periods of that camera's configured rate. Override with
`--robot-tolerance-s` and `--camera-tolerance-s`. Gaps beyond tolerance fail the
export instead of interpolating across them.

Output features are `observation.state`, `action`, `observation.images.<name>`
and optional `observation.depth.<name>`, with left then right ordering for the
active arms. Depth inputs are converted to metres for LeRobot's depth encoder;
LeRobot's default reader returns millimetres, so request `depth_output_unit="m"`
when metre-valued readback is needed. RGB video compression and LeRobot's depth
quantization affect the exported copy; native HDF5 remains lossless.

The exporter finalizes and reopens the dataset using the official loader,
including RGB/depth decoding at episode boundaries. `source_report.json` records
source hashes/identities, task/outcome, configuration provenance, source indices,
alignment offsets and sample reuse for every exported frame. A failed export
leaves its new destination for inspection and requires another new destination.

## Validation

`tests/test_dataset_hdf5.py` checks native rates, exact RGB/depth values,
timestamp precision, alignment, gap rejection and actual official LeRobot
write/readback. `tests/test_dataset_recorder.py` covers ownership, interrupted
files, bounded queues and commit failures. `tests/test_dataset_supervisor.py`
uses mock buses and synthetic cameras with real WebSocket controller packets to
exercise save/hold/return/discard, stop, tracking loss and recorder stalls.

Run source/export tests using the export environment with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`; use a writable `HF_HOME` for loader caches.
The supervisor tests additionally require the existing Quest/recording
interpreters and local socket access. All hardware acceptance in these tests is
mock-only. Real cameras and loaded physical motion still need commissioning.

Recording returns use the same 5° per-joint opening tolerance as keyboard restarts, with 300 ms of stationary feedback and the ceremony gripper aperture required. Sleep retains its tighter arrival check. After return, release grips, press X to align, then right-controller A to begin recording. If you align at the current pose without returning, A starts recording there too.

While an episode is recording, Quest B or terminal ← discards it, opens the grippers fully, then returns to opening once finalization completes, tracking is fresh and side grips are released. It does not start a new episode. Space, a fault, or shutdown cancels the pending return. Outside recording mode, B retains its powered-stop action.

Tracking/reference loss pauses capture and freezes the episode timer. This includes a controller-source timeout detected by the motor worker: motion still enters powered hold at the existing 150 ms limit. Restore tracking, release selected grips and press X: successful alignment resumes the same episode without A or →. Paused intervals are omitted from captured streams and excluded from the duration limit. Native timestamps retain the gap, explicit pause/resume events mark it, and export aligns active segments separately. Other motor faults and explicit stops still abort; they never auto-resume.

The native LeRobot regression suite (`tests/test_dataset_lerobot_v3.py`) checks
official v3 readback, parity with offline conversion, RGB/depth decoding, discard,
pause trimming, gap rejection, compatible resume, interrupted commit refusal, and
a synthetic-camera recorder subprocess including episode limits after restart.

### Native LeRobot save rejection

A failed pre-commit alignment check now reports **EPISODE NOT SAVED** rather
than killing the recorder. The saved count stays unchanged and the original
samples, events and rejection reason remain in
`attempts/episode_NNNNNN.rejected.sqlite`. No accepted dataset is modified.
Release grips and align with X before recording a new attempt; the rejection
does not resume motion or accept the invalid episode. Camera capture faults,
transport failures, disk errors and interrupted commits remain fatal.

On 2026-11-09 a cupstack save was rejected for a left-wrist RGB alignment gap
of 142.3 ms (15 FPS default tolerance: 133.3 ms). Other sessions stopped for
stale camera capture, controller tracking loss, an expired motor target and
sleep settling error of 2.72 degrees. These are distinct causes behind the
shared “Motion held” display. The standalone LeRobot sleep connection fix
is not called by Quest teleoperation. Avoid overlapping heavy dataset export
work with hardware capture while investigating timing; host load alone does
not prove the cause of any particular interruption. No timing or geometric
safety thresholds were relaxed by the save-rejection change.

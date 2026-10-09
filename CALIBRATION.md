# Quest → ALOHA IK calibration capture

`record_ik_calibration.py` records evidence without changing the mapping or publishing robot commands. Run the existing Quest bridge and `quest_aloha_sim.py` separately. Use a helper to read the terminal cues while the operator wears the headset. Run the right and left hands in separate sessions, moving the camera between sessions as needed.

## Setup and capture

Use the ROS environment already used by this project. At implementation time, this environment lacks pip and the three capture packages below. Install these additional capture dependencies there:

```bash
conda install -n quest_ros pip
conda run -n quest_ros python -m pip install pyrealsense2 opencv-python-headless mss
```

The recorder requires an X11 desktop (this workstation currently reports X11). MSS screen capture is not a general Wayland recorder. Run from the same desktop login/display as the MuJoCo viewer. Close other applications that use the D405. If the SDK cannot access USB, check the RealSense SDK installation and device permissions; do not run the recorder as root.

1. Fix the D405 on a stand facing the controller workspace. Start around 25–40 cm from the controller, keeping every motion in view and inside the camera's working range. The D405's specified ideal range is **7–50 cm**, so use it for one local hand workspace, rather than a room or full-body view. See [RealSense D405 specifications](https://www.realsenseai.com/product-family/d405-series/).
2. Place a ruler and a fixed, labeled right/forward/up reference near the working volume. Define directions from the operator's neutral, forward-facing stance; do not change stance, recenter the Quest, or move the camera during a session. If any of these change, start a new session.
3. For later quantitative optical tracking, attach a rigid, known-size fiducial board to the controller, visible throughout the motions, without obscuring Quest tracking features. Record its size, attachment transform if known, camera position, and direction convention in a `notes.txt` file in the session folder. Recording RGB-D alone does **not** extract the controller's six-degree-of-freedom pose. A single tracked point cannot measure wrist orientation.
4. Start the Quest bridge and simulator using the README. Position the MuJoCo viewer so it stays visible and unobscured. Fix the viewer angle and do not move its window during capture. A screenshot records whatever is visible in that region, including overlapping windows.
5. Start with a short recording and inspect the outputs before a full session:

```bash
conda run -n quest_ros --no-capture-output python record_ik_calibration.py --side right --seconds 15
```

The default captures the entire first display. To capture only MuJoCo, pass its desktop rectangle in pixels; for example, replace these example coordinates with the actual window rectangle:

```bash
conda run -n quest_ros --no-capture-output python record_ik_calibration.py --side right --region 100 100 1280 720
```

Use `--monitor 2` for the second display, or `--serial SERIAL` to select among multiple D405s. Screen recording defaults to 15 FPS; camera capture requests color and depth at 640×480, 30 FPS. Unsupported camera modes fail visibly. `--output PATH` selects the parent folder; each recording gets a new timestamped subfolder. Ctrl+C finalizes partial recordings. Startup failures and lost selected-arm ROS streams mark the manifest as failed; an interrupted run is retained for inspection.

Preview the guided cues without ROS, a camera, or capture dependencies:

```bash
python record_ik_calibration.py --side right --plan-only
```

## Operator motions (275 seconds per hand)

Keep the other hand's grip released. Start with a comfortable neutral controller pose and a simulated end effector away from workspace boundaries. Move slowly; do not try to chase a wrongly moving simulated arm with compensating hand movements.

| Phase | Motion | Purpose |
|---|---|---|
| Baseline, 5 s | Grip released; hold still. | Measure controller/depth jitter. |
| Translation, 162 s | Up, forward, right; three trials each. Each trial: release and reset 4 s, engage and hold 3 s, move +5 cm over 3 s, hold 3 s, return over 3 s, hold 2 s. Keep orientation fixed. | Recover axis signs, permutation, scale and cross-axis coupling; returns also test negative movement. |
| Rotation, 66 s | Roll, pitch, yaw separately: reset 4 s, engage and hold 3 s, rotate to +15° and hold within 6 s, to −15° and hold within 6 s, return/release 3 s. Minimize translation. | Check rotation axes, sign, gain and position/orientation coupling. |
| Clutch, 12 s | Move 5 cm while engaged; release, reposition hand, re-engage, repeat. | Confirm release freezes the target and re-engagement creates a new reference without a jump. |
| Gripper, 10 s | Hold hand still; squeeze/release index trigger twice. | Separate gripper control from arm pose. |
| Held-out motion, 15 s | Slow 5 cm square, then diagonal, orientation fixed. | Validate combined axes on motions excluded from fitting. |
| Final, 5 s | Release grip and hold still. | End baseline and clean stop. |

Roll/pitch/yaw here are about the operator's fixed forward/right/up axes, respectively; positive means right-hand-rule rotation about the named positive axis. Mark these conventions in the notes. A helper can demonstrate the signs before recording. Cues are timed instructions, not confirmation that a motion was performed correctly; repeat any missed or occluded trial. After corrections, repeat the whole sequence and add 10 cm translations only where camera coverage and simulator workspace allow. Repeat rotation trials for quantitative fitting rather than relying on the single exploratory sweep per axis.

## Saved evidence and synchronization

- `d405.bag`: RealSense SDK RGB-D recording, including stream calibration. Open with RealSense Viewer or SDK playback. This is an SDK recording, not a ROS 2 rosbag.
- `camera_frames.jsonl`: frames observed by the Python reader, with device frame number, device timestamp/domain and host monotonic receive time. The SDK bag may contain more frames than the reader observed; match using stream and frame number/timestamp, not line index.
- `screen.avi`: MJPEG screen capture. `screen_frames.jsonl` maps each encoded frame to the host capture interval and midpoint. Playback uses nominal FPS; use the timestamps for analysis because capture stalls can change the real frame interval.
- `telemetry.jsonl`: both hands' Quest pose/grip/trigger and simulator target, achieved pose, position error, engagement and gripper position. Pose messages preserve source ROS timestamps and frame IDs; every callback receives a host monotonic timestamp. The selected hand's eight topics are required before guided cues start and must remain live.
- `events.jsonl`: stage labels and actual host monotonic cue times.
- `manifest.json`: device/depth scale, screen region, options, protocol, counts, errors and completion status; `quest_aloha_sim.py` is copied and hashed to preserve the mapping used in this session. It is a snapshot of the file, so restart the simulator after edits to ensure the running process matches it.

This provides a shared **host timeline**, not hardware-synchronized exposure times. Camera USB buffering, ROS transport, viewer refresh and capture delay all contribute offsets. Do not subtract raw device timestamps or ROS stamps from host monotonic values. Use receive/capture times for coarse alignment and estimate residual lag from repeated visible direction changes and controller/target trajectories. The simulator currently publishes at approximately 20 Hz and refreshes the viewer at approximately 30 Hz; this setup cannot substantiate sub-frame latency claims. Preserve raw timestamps and report uncertainty. Screen images are visual evidence; use telemetry for numerical IK error.

## Calibration and diagnosis plan

1. **Check data quality.** Inspect RGB/depth playback and the screen crop; confirm the active controller stays visible and depth is valid. Check timestamps for gaps, tracking discontinuities, missing topics and engaged state. Exclude transitions and clipped/rebased trials. Average stationary endpoints over roughly the final second of each hold instead of selecting one frame.
2. **Determine translation mapping before tuning IK.** Compute Quest displacement from each engaged baseline to its held endpoint. Use the ruler/fiducial reference to establish actual physical direction and distance. Express desired robot displacements in `aloha_sim_world`, explicitly associating operator right/forward/up with the model's intended axes; the screen's apparent directions depend on viewer angle. Fit `Δp_world ≈ A Δp_Quest` using the three axis families and repeated trials. Inspect singular values, axis signs and per-axis gains. Separate a proper rotation from scale; a negative determinant suggests a handedness/convention issue requiring explanation rather than silently treating a reflection as a rotation.
3. **Establish camera extrinsics if using optical poses.** A D405 point is in camera coordinates, not the robot frame. Estimate camera-to-reference transform from a measured rigid board, track the controller board, and account for its offset from the Quest controller pose origin. Otherwise a rotating offset marker produces apparent translation. Do not fit absolute hand-to-robot translations across different clutch engagements; this controller uses relative motion.
4. **Unify the coordinate convention.** Current code uses translation `[[1,0,0],[0,0,-1],[0,-1,0]]` (determinant −1), but rotation uses `[[1,0,0],[0,0,-1],[0,1,0]]` (determinant +1). These disagree on vertical sign. Use measured basis conventions to resolve this, then validate rotation conjugation with the signed rotation trials. For a proper frame rotation `R`, compare `R ΔR_Quest Rᵀ` against robot target rotation; explicitly handle any upstream handedness conversion. Do not just flip another axis based on a screenshot.
5. **Distinguish target errors from solver errors.** Compare physical/controller displacement → target, then target → achieved end effector. Wrong target direction indicates mapping; correct target but poor achieved pose indicates IK, joint limits or workspace constraints. Current settings: position gain 1, rotation gain 0.6, rotation cap 0.65 rad, clutch displacement cap 0.30 m, frame-jump rebase above 0.12 m, stale input after 0.35 s, Z floor 0.12 m, Y range [−0.30, 0.28] m, left X ≤ −0.04 m and right X ≥ 0.04 m. Targets are already clipped when published. Reject boundary trials; this recorder does not log joint configurations or internal unclipped targets.
6. **Validate independently.** Use held-out square/diagonal motions, both hands and repeated sessions. Report directional agreement, fitted gain, cross-axis leakage, endpoint repeatability, target-to-achieved RMS/max position error, orientation geodesic error and estimated lag. Provisional simulator goals: no sign errors, translation gain within 10% of intended, cross-axis leakage below 10%, settled position error below 5 mm and orientation error below 5° on reachable unclipped targets. These are proposed criteria, not measured performance. Set latency criteria after measuring timing uncertainty. Check clutch continuity and gripper behavior separately.

The simulator is a position-level kinematic IK check; these measurements do not establish physical robot dynamics or contact performance. The capture script leaves the current mapping unchanged so baseline and corrected runs can be compared.

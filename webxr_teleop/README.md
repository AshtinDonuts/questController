# Quest browser → single-arm ViperX simulator

This is the updated teleoperation path, adapted from [StairVLA's PiperX teleoperation](https://github.com/Dicomsky/StairVLA/tree/c128495cf8c9882734d2d8e21ed7db3435d96920/examples/PiperX/teleoperation). It uses the Quest browser's tracked WebXR grip poses and a local HTTPS/WebSocket server. The existing ViperX model and joint limits remain the robot model. No CAN driver, physical robot command, old headset app, or ROS bridge is used by this path.

## Install and launch

From the project root, use the existing `quest_ros` Conda environment (Python 3.12, NumPy and MuJoCo). Additional dependencies are SciPy and websockets, declared in `requirements-webxr.txt`. ffmpeg and openssl must be on PATH. They are already available on this workstation, and the Python dependencies were installed during implementation.

For a fresh environment with pip:

home/King/miniconda3/bin/pip --python /home/King/miniconda3/envs/quest_ros/bin/python install -r requirements-webxr.txthome/King/miniconda3/bin/pip --python /home/King/miniconda3/envs/quest_ros/bin/python install -r requirements-webxr.txt```bash
conda run -n quest_ros python -m pip install -r requirements-webxr.txt
```

If that environment lacks pip but base Conda has it, as on this workstation:

```bash
/home/King/miniconda3/bin/pip --python /home/King/miniconda3/envs/quest_ros/bin/python install -r requirements-webxr.txt
```

Close the old simulator. You can stop the legacy bridge since it is not used:

```bash
systemctl --user stop quest-pose-bridge.service
cd /home/King/Desktop/QuestController
conda run -n quest_ros --no-capture-output python -m webxr_teleop.run
```

The terminal prints a LAN URL such as `https://192.168.x.x:8443`. Keep the Quest and computer on a network that allows connections between them. Open **the printed URL in the Quest browser**, accept this local server's self-signed certificate, then press **Start Controller Tracking**. The bundled frontend works without internet access. TLS files are generated under `~/.cache/questcontroller/vr_tls/` and are not committed. Only one browser/WebSocket connection is permitted at a time; close other copies of the page.

Use `--port 8444` if another process owns the default port. If the page cannot load, check network reachability and TCP port access. The program does not change firewall settings. `--headless` skips the desktop viewer but still renders MP4.

## Align once, then move

1. Stand facing the arm's forward reach in the simulator. Release the right grip.
2. Hold the right controller with **red +X right, green +Y up, blue +Z backward**. Its −Z direction should point horizontally forward along the arm’s reach.
3. Press **left X**. This sets horizontal heading; WebXR gravity supplies up. No precise hand translation measurements are needed.
4. Hold **right grip** to move; release to reposition your hand. **Right trigger** closes the gripper proportionally.
5. Click the **right stick** to show the mapped robot base frame above the left controller and the simulated end-effector frame beside the right controller. Robot **world** axes: red X = right, green Y = forward, blue Z = up. The EEF axes are the actual right/gripper site frame and rotate with the wrist; the same triad is drawn at the gripper in MuJoCo and the MP4.

Position gain is **1 robot metre per hand metre**. `--scale 0.6` gives StairVLA's default 0.6 ratio; `--scale 1.5` amplifies motion. `--rotation-scale` defaults to 1.0. This replaces the earlier noise-derived calibration and per-trial replay amplification; old calibration JSON files are not loaded by this path.

Both translation and rotation use the same proper frame rotation. Wrist rotations are relative to the pose at grip engagement; you do not have to match the robot's wrist orientation before gripping. The target is filtered with a 60 ms time constant and 2 mm / 1° deadbands. Default target speed limits are 0.35 m/s and 160°/s; CLI flags `--max-linear-speed` (m/s) and `--max-angular-speed` (degrees/s) adjust them.

Release freezes the current achieved pose. Missing pose input for 0.5 seconds holds the arm and gripper. Reconnecting or resetting the WebXR reference space requires alignment again and discards any active episode marker. Every pose packet carries a reference-space epoch, so a dropped reset notification cannot silently preserve an outdated calibration. Tracking jumps greater than 12 cm between received poses stop movement until you release and regrip; they do not silently accumulate automatic rebases. Startup with grip already held cannot move the arm.

IK uses the actual ViperX model, joint bounds and position/orientation residuals. A target outside the solver's 5 mm / 5° acceptance thresholds holds the previous command and shows an out-of-reach warning with periodic haptics. Cartesian limits are X ±0.45 m, Y −0.10..0.65 m, Z 0.12..0.75 m. This remains a kinematic simulator, not a dynamics/contact validation.

## Home, record and save

- **Left Y:** move slowly to the model's neutral joint pose (8°/s).
- **A:** idle → home → ready; press again to start an episode; press again to save the episode marker.
- **B:** discard the episode marker or cancel homing.
- **Ctrl+C / close viewer:** finalize the session MP4; any unfinished episode is marked discarded.

Every run saves a timestamped folder under `teleop_sessions/`, printed at startup and shutdown:

- `session.mp4`: 640×480 H.264, nominal 30 FPS, the full simulator session.
- `telemetry.jsonl`: one row per video frame, with actual host times, controller pose, joint state/command, target/achieved end-effector pose, tracking and IK status, events and phase.
- `manifest.json`: settings, upstream revision, frame count and episode frame ranges (end exclusive). A discarded episode remains in the full session video; the marker identifies it as discarded.
- `alignment.json`: most recent WebXR-to-world frame alignment for inspection. Calibration is session-specific and deliberately not auto-restored after a reference reset.

`--output /absolute/new/folder` selects a new output directory. Existing directories are refused to avoid overwriting recordings. MP4 playback uses nominal FPS; use telemetry timestamps for actual timing if rendering or IK makes the loop miss 30 Hz. This is a local simulation session format, not StairVLA's LeRobot real-camera dataset format.

## Reproducible verification

```bash
conda run -n quest_ros --no-capture-output python -m webxr_teleop.run --headless --demo
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 conda run -n quest_ros python -m pytest tests/test_webxr_teleop.py -q
node tests/test_webxr_client.cjs
```

The demo generates known WebXR poses for alignment, 15 cm forward/right/up moves, return motions and a 15° yaw; it writes the same MP4 and telemetry as a real session. Tests cover axes/gain, orientation and return-to-neutral, rate limits, clutch, stale input, jump recovery, homing, unreachable IK, malformed packets, TLS/WSS lifecycle, one-client ownership and a real entrypoint run with controller packets and episode recording. Pytest plugin autoload is disabled because the installed ROS launch-testing plugin is incompatible with the environment's pytest version.

Desktop rendering, protocol integration and synthetic controller behavior can be verified without a headset. Actual Quest immersive-AR rendering, controller fit/comfort, LAN access, and physical haptics still require trying the headset.

## Upstream provenance and adaptation

Pinned source: `Dicomsky/StairVLA`, commit `c128495cf8c9882734d2d8e21ed7db3435d96920`.

- `server.py` is adapted from `vr_server.py`: retained local HTTPS/WSS/TLS, bundled asset serving and haptic/status protocol; added explicit tracking validation, quaternion validation, single-client ownership and reset generations.
- `web/` comes from the upstream teleoperation frontend, including offline A-Frame/controller/font assets and their notices. Modified for the ViperX frame, tracked `XRFrame.getPose(gripSpace, referenceSpace)` acquisition, tracking-loss packets, reference resets, and local session controls.
- `control.py` adapts the upstream mapping/clutch/limits/home/IK-rejection workflow to ViperX MuJoCo. It uses metre units, SO(3) orientation filtering, gravity-preserving one-button heading calibration, and regrip after tracking loss/jumps. It does not use the incompatible PiperX URDF or CAN actuator driver.
- `recording.py` and `run.py` implement the simulator integration and MP4 session/episode workflow. LeRobot camera-dataset export and real PiperX hardware support were not imported because this project operates the ViperX simulation.

See `LICENSE-StairVLA`, `NOTICE-StairVLA`, `THIRD_PARTY_NOTICES.md`, and `web/vendor/` for retained attribution and asset licenses. The source model has its existing Menagerie license.

Frame conversion: with B mapping WebXR to world and R₀ the EEF-to-world orientation at grip engagement, C = R₀ᵀ B maps WebXR deltas into the anchored EEF frame. Targets are p = p₀ + R₀ C Δp and R = R₀ (C ΔR Cᵀ), before gain/filtering. IK expects world poses. This preserves operator-relative directions while expressing the commands in the EEF frame; it does not switch translation to wrist-steered tool jogging.

### Dual-arm option

Start two forward-facing ViperX arms, with no frame:
```bash
conda run -n quest_ros --no-capture-output python -m webxr_teleop.run --scene dual
```

The default remains `--scene single` (right arm). Reload the Quest page after switching.
In dual mode, release both grips, hold the right controller naturally pointing −Z forward,
and press **left X** once to align both arms to the shared tracking frame.
**Left grip/trigger controls the left arm/gripper; right grip/trigger controls the right.**
Each hand has its own clutch reference, tracking timeout, jump detection and IK.
A lost hand holds its matching arm; the other hand can continue.
Y homes both arms; A runs the home/start/save sequence for a shared episode; B discards
the shared episode and cancels homing. Losing either hand discards an active episode marker.
Right-stick click shows both EEF orientations. MP4 contains both arms and their EEF axes;
telemetry has `arms.left` and `arms.right`, while the legacy top-level pose fields remain right-arm data.
The arms are 64 cm apart, with each workspace translated to its own base.
The kinematic IK does not enforce inter-arm collision avoidance.

Render the independent right-only, left-only, and simultaneous-motion check:
```bash
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output python -m webxr_teleop.run --scene dual --headless --demo
```

# Quest → physical ALOHA / dual ViperX

This hardware path uses the local `alohaX_lerobot_fork` Dynamixel interface and the existing Mink model. It is separate from `python -m webxr_teleop.run`, which remains simulator-only. **Mock is the default. No physical motor bus is opened in mock mode.**

Implementation is software-tested with mock motors. Physical directions, dimensions, stopping behavior and motor firmware still require commissioning. The example profile is deliberately unverified and cannot run in live mode.

## Commands

For guided manual direction checks, run `python -m webxr_teleop.hardware.check_directions --side left --profile webxr_teleop/hardware/rig_mirror_candidate.json`. Use the terminal Enter key to start each reference demonstration, finish your manual movement, and accept the inferred sign; R repeats and Q quits. The viewer demonstrates the requested model direction, rather than mirroring physical movement during that stage. Motors must remain torque-off. Six primary joints are tested with shadow feedback checks, followed by gripper opening. Results and a separate unverified `direction_candidate.json` are saved beside `video/session.mp4`; no motor Drive_Mode registers or source profiles are changed. Changed signs preserve the provisional joint angle at each test's starting pose, so this procedure does not independently calibrate absolute offsets.

For torque-off mapping diagnostics, run `python -m webxr_teleop.hardware.mirror --side left` under `quest_ros`. This opens a live MuJoCo viewer and records MP4/encoder telemetry. The separate LeRobot reader checks all motors are torque-off and performs no motor-register writes; it refuses already enabled torque instead of disabling it. It displays the profile's current mapping without joint-limit clamping or IK, so an unverified mapping may look physically impossible. Close the viewer or press Ctrl+C to stop. `--headless --seconds 5` captures a bounded diagnostic without a window. Motor power is still needed for encoder reads.

Run from `/home/King/Desktop/QuestController`:

```bash
# Reproducible mock demonstration: teleop, smooth home, B stop.
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --demo --seconds 15

# Quest controlling mock arms. Open the printed HTTPS URL in Quest.
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run

# Single right-arm startup demonstration, with MP4 (no hardware access).
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --side right --startup --demo --seconds 15

# Read physical feedback/configuration, without motor register writes.
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --mode observe --profile /absolute/path/rig.json

# Only after the profile and watchdog behavior have been verified:
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --mode live --profile /absolute/path/rig.json

# Explicit torque-on startup for ONE commissioned, supported, stationary arm.
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --mode live --side right --startup \
  --profile /absolute/path/rig.json
```

Use `--side left` for the left follower. Only the selected motor bus is opened, controlled and rendered. The default `--side both` attaches to two already powered arms; `--startup` requires one selected side and is rejected in observe mode. Live commissioning checks apply to the selected arm, while the profile still contains both arm definitions. Both Quest controllers remain required for tracking and the X/Y/B controls, including right-controller heading alignment.

## Opening → ready → Quest → sleep sequence

Opt in with `--sequence`. Without it, the existing manual-start/home workflow remains available.
The full mock sequence (no motor bus access) can be demonstrated with:

```bash
MUJOCO_GL=egl conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run --mode mock --side left --sequence --demo --seconds 20
```

For interactive mock control, omit `--demo --seconds 20` and open the printed Quest URL.

1. Establish current-position powered hold, then automatically follow a slow joint trajectory to `opening_rad`. Quest tracking is not required during opening. `--sequence` now explicitly selects automatic single-arm torque-on startup, without a prompt, manual positioning, or countdown. It verifies a stationary measured-position hold before moving. Do not combine it with `--manual-start`. Like the previous manual workflow, it uses the software command watchdog rather than the unverified motor watchdog; PC, serial or motor-worker failure cannot guarantee a fresh hold command.
Automatic opening waits for at least 0.3 seconds of fresh, fixed-target feedback with no more than two encoder counts of total joint excursion and at most 0.5° target error (2% for the gripper). This tolerates encoder quantization without accepting cumulative drift. The startup message reports this wait rather than requesting reset.

2. Opening runs for at most 10 seconds from the `opening_start` event. It may finish earlier if it reaches and settles within 2° of the target. At the 10-second limit, the supervisor cancels the trajectory, the motor worker captures its current measured pose into powered hold, and the session enters `READY` regardless of remaining target error. `READY` therefore means ready for explicit control handoff, not proof that the ceremony target was reached. Release both grips and press left X to hand over and align before engaging a grip. R remains available for explicit recovery after a nonfault stop; X alone cannot clear a latched stop or fault. Stops and faults override this timed handoff; they never become READY automatically. Sleep/home retain their tighter 0.5° arrival tolerance and three-second settling timeout. Release both grips and press left X to hand over and align directly; no R is required from READY.
3. Hold the matching **side grip** to move as in MuJoCo; the **index trigger** controls the gripper. Grip release requests a fixed measured-position hold directly from the motor worker, without waiting for IK. Torque remains enabled. Re-gripping resumes relative control without a pose jump.
4. With grips released and the arm stationary, hold **left Y for one second** to enter `PARKING`. This now requests `sleep_rad`, not home. The trajectory ignores Quest poses/grips/triggers and preserves gripper opening.
5. After sustained settling, enter `PARKED` powered hold. A new launch is required for another opening sequence; reset cannot restart a parked session.

Right B, terminal Space, and local `stop` cancel movement into latched hold. Tracking loss during teleoperation latches hold; neither reconnection nor a held Y restarts a trajectory. Opening uses local feedback and does not require headset tracking. Motor feedback loss and worker/planner faults interrupt all motion. Stopping opening requires a new launch. After verified sleep arrival, the selected arm and gripper torque are disabled and read back.

For opening and sleep, the motor worker adopts the planner’s first target as the zero-velocity trajectory origin once. A difference of up to two encoder counts from its independently sampled hold target is accepted; larger differences, a moving command, or a repeated start marker latch hold. The planner owns speed/acceleration limiting; the worker no longer differentiates received targets at a fixed 20 ms period, which falsely rejected skipped planner frames. Scripted opening omits the 2° target-ahead gate and 1°/0.5 s following-error gate, matching the absence of those gates in the standalone diagnostic. Teleoperation now bounds command lead in the planner without latching on lag; sleep follows its slow trajectory without those lag latches. Opening still requires endpoint settling; this change does not establish that the motors follow the trajectory. Opening temporarily uses an 8-count motor velocity profile (~11°/s) so a previous slow diagnostic cap cannot constrain its 10°/s planner target. Prior motor velocity profiles are restored on hold/completion; sleep and teleoperation limits are unchanged. The standalone `direct_pose` opening uses 6 counts (~8.2°/s), twice its former 3-count speed, and also restores the prior profiles on exit. Worker rejection reasons are included in terminal hold messages.

Named movements use the existing synchronized quintic trajectory (opening: 10°/s; sleep/home: 5°/s; acceleration: 10°/s²), workspace-plane path validation before the first target, following-error checks, and motor command watchdog. These are sampled geometric checks, not general collision avoidance. Software response and physical stopping take finite time.

Each selected arm must explicitly define finite six-element `opening_rad` and `sleep_rad` in model radians. Opening must be within the configured joint bounds. Sleep is exempt from arm joint-bound rejection at profile validation, planning and the sequence motor worker; it is neither clamped nor substituted. Automatic startup may hold a measured pose outside those bounds and opening may traverse back into them. Teleoperation retains its original joint bounds. Gripper range, workspace planes, speed/acceleration, feedback freshness, following-error and stop checks remain active. Live sequence use additionally requires either recorded physical verification (`sequence_verified` and its note) or explicit operator authorization (`sequence_authorized` and its note), as well as the existing mapping/calibration requirements. No missing sleep target falls back to home. Full path checks still run from measured feedback.

`rig.json` now uses the ALOHA LeRobot `.cache/poses/ceremony_pose.json` for opening and `.cache/poses/rest_pose.json` for sleep, from `/home/King/alohaX_lerobot_fork`. Each side uses its own follower record. Primary joints are selected by motor name, excluding shadow motors and the saved gripper target. Conversion uses the current rig mapping: `model_rad = radians((calibrated_degrees - offset_deg) / sign)`. Targets are imported exactly, without clamping. Source paths, SHA-256 hashes, calibrated angles and the mapping used are saved in `sequence_sources`; a later mapping change requires reimport. Importing does not grant physical verification and clears any prior sequence authorization. The left rig records the user’s explicit authorization of this workflow separately; physical verification remains false. Right-arm verification remains unchanged.

Reimport after updating the source poses or mapping (offline; no motor drivers loaded):

```bash
python -m webxr_teleop.hardware.import_poses \
  --profile webxr_teleop/hardware/rig.json \
  --opening /home/King/alohaX_lerobot_fork/.cache/poses/ceremony_pose.json \
  --sleep /home/King/alohaX_lerobot_fork/.cache/poses/rest_pose.json \
  --output webxr_teleop/hardware/rig.json
```

The imported sleep pose is now exempt from arm joint-limit checks. This exception does not establish physical reachability or collision clearance. Physical holding/drift remains unverified. `example_profile.json` still contains **simulation-only** fixtures for mock regression tests: opening equals its home, and sleep is a small waist rotation. These fixtures are separate from the imported physical targets.

## Single-arm manual positioning and software hold

Use `--manual-start` for the five-second positioning workflow. This explicitly selects the independent motor-process **software command watchdog**, without the unverified Dynamixel bus watchdog. It does not mark `watchdog_verified` true. The selected arm's mapping, calibration fingerprint, model limits and workspace checks still apply. The left mapping is verified; the right mapping still requires physical verification.

```bash
cd /home/King/Desktop/QuestController
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --mode live --side left --manual-start \
  --profile webxr_teleop/hardware/rig.json
```

1. Start with torque already off and physically support the arm. The application refuses to disable an already powered arm. Close other programs accessing the follower bus.
2. Press Enter at the prompt. During the visible 5–1 countdown, position the supported arm within the model limits and workspace. Stop moving at zero and keep supporting it until startup hold verification succeeds.
3. Current raw encoder positions are preloaded as goals, slow profiles applied, and torque enabled once. No home trajectory is sent. The application remains disarmed.
4. Open the printed HTTPS Quest URL in the headset and enter VR. Both controllers must be tracked. Release both grips; press R in the PC terminal, then left X to align while pointing the right controller forward. Hold the left grip to control the left arm.
5. Tracking/command timeout, B, or Space latches a fixed measured-position hold with torque left on. There is **no five-second delay on loss**. Reconnection alone never resumes motion; reset and alignment are required.

A stopped software-hold session keeps the motor worker reading feedback and maintaining its fixed goal. Hold drift over 1° (or 2% gripper opening) for 0.5 s faults the session, retains that target, and refuses rearming. It never repeatedly recaptures a drifting pose. Exiting closes the connection without disabling torque; motors retain their last goals.

This mode cannot guarantee a new hold command if the motor process, serial link, PC or power fails. It has no independent motor timeout fallback. The previous physical countdown trial reported hold drift; mock tests do not verify physical holding. A preexisting nonfaulted 300 ms bus-watchdog setting is disabled only while all motors are torque-off; a latched watchdog is always refused. The usual `--startup` path retains its motor-watchdog verification requirement.

The built-in HTTPS server replaces the separate pose bridge for this workflow. Every session records `<session>/video/session.mp4`; its absolute path is printed at launch. The original folded sleep pose remains outside the calibrated model limits and is not accepted as a live IK starting pose.

## Explicit single-arm startup

Support the torque-off arm and keep it stationary until startup verifies holding. `--startup` connects the low-level follower bus and loads existing calibration without calling `ManipulatorRobot.connect()`. It checks motor settings and joint feedback, preloads every motor's current raw position and verifies goal readback, applies slow RAM profiles, and arms the watchdog **before** the sole `Torque_Enable = 1` call. Already enabled motors are not cycled or re-enabled. It then checks torque, errors, encoder continuity and a 0.3-second hold before exposing feedback to the supervisor. Quest/test motion remains disarmed until explicit reset and alignment.

Only configured Extended Position mode (4) arm joints and Current-based Position mode (5) gripper are accepted. Position Control mode (3), which can reset position on torque enable, and automatic torque-on-by-goal-update are refused before writes. An unexpected encoder change after enable triggers a best-effort hold at the new raw counts and a fault; it never silently changes calibration or accepts motion. Startup is attempted once per connection. Cancellation, a partial enable, or failed feedback does not cause a torque-off or automatic retry; holding is not considered confirmed after an error.

The default motor-worker Python is `/home/King/miniconda3/envs/lerobot/bin/python`; override with `--lerobot-python`. The Quest/Mink and LeRobot dependency environments stay separate. `--output` chooses a **new** session directory. Video is `<session>/video/session.mp4`, authoritative supervisor telemetry is `<session>/hardware.jsonl`. Video displays state/reason and measured poses; stale feedback produces a red warning frame. Rendering has its own process and cannot block the motor watchdog.

Independent local commands, from another terminal:

```bash
conda run -n quest_ros python -m webxr_teleop.hardware.run status
conda run -n quest_ros python -m webxr_teleop.hardware.run stop
conda run -n quest_ros python -m webxr_teleop.hardware.run reset
```

The socket defaults to `/tmp/quest-aloha-<uid>/control.sock` in a private directory. Startup now reports and removes a confirmed stale, owned socket after checking for a live listener and remaining hardware processes. A lifetime endpoint lock prevents competing launches; active endpoints are never replaced. HTTPS port availability is checked before motor startup. A new process does not resume the previous session. Robot-bus watchdog faults require explicit supported-arm recovery outside this application; the application never clears them automatically.

## Controls and motion ownership

1. Start with the selected arms configured, stationary and **already holding with torque on**, or use the explicit single-arm startup above. Close other motor drivers first.
2. Start the application; release both controller grips. Press **R in the supervisor terminal**, or run `reset`.
3. Point the right controller naturally forward and press **left X** to align. Reset and alignment do not move the robot.
4. Each grip enables its matching arm and gripper. Trigger movement changes opening relative to grip engagement, avoiding a jump when re-engaging. Releasing grip captures a fixed hold target for that arm and gripper.
5. **Hold left Y for one second with both grips released** to home both arms. Arms must have settled for at least 0.3 s. Homing exclusively owns motion; controller poses, grips and triggers cannot alter it. Grippers retain their opening.
6. Homing uses a synchronized quintic joint trajectory, at most **5°/s and 10°/s²**, with zero endpoint velocity and acceleration. Duration is at least two seconds and increases with travel distance. The entire sampled joint path is checked against limits/planes before the first move. This is not general continuous collision detection.
7. Both arms must settle within 0.5° of home for 0.3 s. Then release both grips and engage again to teleoperate. Holding Y continuously does not repeatedly home.

**Right B**, **Space in the supervisor terminal**, or the local `stop` command latches the selected arms/grippers into powered hold. Space requires focus in that terminal. A is unused. In the legacy workflow there is no sleep command; `--sequence` provides powered sleep parking. The sequence disables torque only after verified sleep arrival; ordinary stops retain torque. In single-arm mode, homing and all joint targets apply only to the selected arm.

After a stop, release both grips, explicitly reset locally, align with X, then squeeze again. After hardware or planner faults, restart after resolving the cause; watchdog faults additionally require supported-arm recovery. Homing never resumes automatically after interruption.

## Stop behavior and limits

- Invalid/untracked controller, reference reset, or controller freshness older than 150 ms: both sides hold, including during homing.
- Each independent motor worker latches hold if commands stop for 150 ms. Planner/encoder crashes cannot keep issuing motion. Feedback older than 100 ms blocks motion. Individual serial operations have one attempt and a 20 ms response timeout.
- A stop captures the latest valid measured joint pose once. Repeated stop packets do not recapture a drifting position. Goal-register readback and physical settling are reported separately (`hold_command_confirmed`, `hold_confirmed`).
- If a bus is inaccessible, the other arm is stopped, the inaccessible arm is reported unconfirmed, and its motor watchdog is the fallback. The application cannot enforce an exact stop pose without communication or motor power.
- Live attachment writes the measured pose, applies conservative **RAM** velocity/acceleration profiles without increasing an existing finite cap, and enables a 300 ms motor bus watchdog. Only explicit single-arm `--startup` can enable torque. Stop, disconnect and completion leave torque enabled. No operating mode, EEPROM, gain/current-limit or reboot writes are made.
- Watchdog recovery is never automatic. The motors' built-in overtemperature/voltage shutdown remains enabled; software cannot prevent loss of holding torque caused by those faults or power failure.
- Teleop command limits: 10°/s and 30°/s² at joints; Cartesian target 5 cm/s and 30°/s; gripper 20% opening/s. Motor profile quantization can make actual limits lower. Targets over 2° ahead of measured arm position are rejected. Following error over 1° for 0.5 s latches hold.
- Planes are fixed in each robot's base frame; tabletop clearance defaults to 20 mm. No object, self-collision or arm-to-arm avoidance is claimed.
- A powered software hold is not a safety-rated emergency-stop circuit or a mechanical brake. Do not infer hardware stopping distance or load-holding capability from mock test results.

## Commissioning the hardware profile

`rig.json` is populated from the local LeRobot `aloha.yaml` and cached follower calibrations, plus the single-arm simulation's neutral pose, joint/actuator ranges, mounting transform and CLI plane defaults (5 mm clearance). Use it with `--side left` or `--side right`; both entries intentionally share the single-arm display placement, not measured dual-arm geometry. It includes calibration fingerprints and source notes. The user-confirmed left mapping is now `sign=[-1,-1,1,-1,1,-1]`, `offset_deg=[-1.7578125,90,88.4375,6.6796875,0,-2.109375]`. It is copied to the right at the user's request; right-side verification remains false. Left mapping verification is recorded, but `watchdog_verified` remains false pending a physical powered-hold test. The observed folded rest pose is outside the model shoulder/elbow limits; manually position within model limits and workspace before startup. Mock validation is available with `--mode mock --side right --startup --profile webxr_teleop/hardware/rig.json --demo --seconds 15`.

Copy `example_profile.json` to a local rig profile. All coordinates/home poses in the example are **simulation placeholders**. No automatic motor calibration is run.

- Set actual follower serial aliases, base positions/quaternions and tabletop height. Both arms are assumed forward-facing. Set the intended workspace planes.
- Use existing, independently checked LeRobot follower calibrations. Its arm values are degrees and gripper values are opening percentages. Internal arm values are radians. The explicit conversion is `lerobot_degrees = degrees(model_radians) * sign + offset_deg`.
- Use observe-mode telemetry (`health.raw_position`, `health.calibrated_position`) and known reference poses to check each joint sign/offset and compare real geometry with the displayed model. Do not enable live output to guess a mapping. Verify at least two distinct known poses per joint and both sides separately.
- Shadow motors must have the expected secondary IDs and compatible raw targets. All nine motors are inspected. Startup preloads and verifies all nine raw goals; normal motion sends goals to the six primary joints plus gripper, with shadow motors following their primary IDs. Unsupported motor IDs/models/firmware/modes fail closed.
- Set measured gripper `finger_closed_m`/`finger_open_m` and check open/close directions. Set `home_rad` to a verified, reachable, collision-free pose on the actual rig. Startup uses measured positions, never this home pose or `initial_rad` (mock-only).
- Keep joint bounds within the MuJoCo mechanical limits. An initial arm outside a plane is rejected rather than moved back into bounds.
- Set each arm's `verification_note`, `verified: true`, and `calibration_sha256` to the SHA-256 of its checked calibration file. Calibration changes invalidate live startup. The flag records operator commissioning; it is not an automated safety certification.
- With mechanically supported arms and the relevant vendor procedure, verify the installed motor watchdog stops in the configured position/current-based-position modes **without removing torque**. Record that result before setting `watchdog_verified: true`. Firmware support alone is insufficient evidence. Normal supervisor exit leaves the watchdog enabled; it may latch after bus traffic ceases.
- Supervised acceptance: first single-joint small motions, then each arm, then both; test B/Space, tracking loss during teleop and homing, re-engagement, planner/process failure and physical settling. Record measured stop latency/drift. No unrestricted live run is performed by development tests.

Read-only observe mode still requires correct stationary, powered motor configuration, but does not require the commissioning flags. It deliberately refuses an already latched watchdog or unsupported configuration. Use the existing supported-arm setup procedure to prepare motors; never start generic LeRobot `ManipulatorRobot.connect()` while relying on powered hold, since that routine disables torque during configuration.

### Sleep recovery from HOLD

In `--sequence`, release both side grips and hold left Y for one second to park from READY, ARMED, or nonfault HOLD. Alternatively press **S** in the supervisor terminal, or run:

```bash
conda run -n quest_ros python -m webxr_teleop.hardware.run sleep
```

Sleep requires fresh motor feedback, not a settling window or released grips. Quest Y must be freshly tracked while held for one second; after acceptance parking needs no headset tracking. Both paths start a fresh command generation from measured position; B/Space/stop still interrupt. Hardware/planner faults require resolution and restart. Command timeout, feedback validity, and joint bounds for teleop remain active. Tracking lag no longer latches a motor HOLD. The `Motor hold:` message now reports the actual cause instead of labeling every rejection a watchdog expiry.

### Live digital twin

Live runs now start a passive MuJoCo window automatically (`--no-twin` disables it; `--twin` enables it in mock mode). The solid arm uses measured encoder joints through the active rig mapping, not commanded joints. Orange spheres show the unclipped relative hand position target; cyan spheres show the speed-filtered position target. Red lighting means feedback is stale; the displayed arm then remains at its last measurement. These markers appear during an active clutch. Closing the window only closes the display. The viewer has no serial connection or command channel, and runs separately from the motor worker, planner, and video recorder.

The hardware JSONL command records now include position targets, IK position/orientation errors, plane limiting, and the alignment basis. A twin is only as accurate as the rig's sign/offset calibration: compare its physical pose with the real arm before treating its Cartesian motion as ground truth.

### Left-arm tracking response

`rig.json` now explicitly sets `arms.left.motor_tracking_profile` to `velocity_counts: 14` (~19.24°/s) and `acceleration_counts: 4` (~85.83°/s²). These RAM motor profile settings override previous finite arm caps on startup, giving motors margin over the unchanged 9°/s planned teleop trajectory. They apply to primary and shadow arm motors; gripper caps retain the previous policy. Opening temporarily uses its existing velocity profile and restores the tuned tracking velocity afterward. Profiles without this setting keep the conservative legacy behavior. No gains, current limits, calibration, or watchdog thresholds are changed. Faster physical tracking remains unverified until a supervised live run.

### Catch-up and priority sleep

Teleop limits new command lead to 2 degrees per joint. At that lead it keeps the existing powered target and pauses further advancement; it resumes automatically as measured joints catch up. Grip release still captures a hold. The motor worker no longer latches on 2-degree target error or persistent following error. This is not unlimited queued motion.

Terminal S, local `sleep`, or a fresh one-second left-Y hold can interrupt OPENING, READY, ARMED, or nonfault HOLD. Sleep cancels the prior command generation and starts from measured position, even if grips are held or feedback has not settled. Accepted sleep ignores tracking loss. Stop commands, stale motor feedback, and latched hardware/planner faults retain precedence. Sleep still follows a slow trajectory and retains its arrival timeout; this change does not guarantee physical arrival.

### Torque off after sleep

Successful sleep arrival (within 0.5° and settled for 300 ms) requests torque-off from the motor worker. The worker checks the current sleep generation, fresh request, and measured arrival before disabling all nine selected-arm motors, including shadows and gripper. It verifies all Torque_Enable registers are zero before the supervisor reports PARKED / torque OFF. Feedback and the digital twin remain active, but no further position commands or rearming are accepted in that session.

Interrupted sleep, normal stops, failed arrival, and shutdown do not trigger torque-off. A failed or missing torque-off acknowledgement is reported as a fault, never as successful parking. With torque off the arm and gripper are unpowered; the sleep position must support the arm and any held object must already be released or supported.

### Opening ceremony speed update

Opening now runs the original quintic trajectory at 3x time: its planned travel takes one third as long, with peak limits of 30°/s and 90°/s². Its temporary motor profiles are 24 velocity counts (~33°/s) and 5 acceleration counts (~107°/s²); previous profiles are restored afterward and gripper profiles are preserved. The ten-second maximum opening period remains. Teleop and sleep trajectory timing are unchanged. The standalone ceremony default is also tripled from 6 to 18 velocity counts (~24.7°/s); explicitly supplied `--velocity-counts` overrides that default, including the slower sleep command.

Planner diagnostic flags are encoded as native Python booleans for JSON IPC. A target-serialization TypeError sends no target and enters recoverable HOLD; the planner remains alive for explicit reset or sleep. It never automatically resumes movement. Hardware faults, invalid geometry, and other planner failures retain their fault behavior. Processes already faulted under the old code require a restart to load this fix.

### Grip-relative direct target tracking (current default)

`teleop_catch_up: false` is the active default. Each side-grip press captures current controller position/orientation and measured EEF position/orientation; release clears that motion's reference and holds the measured pose. Controller movement while released is ignored. A new press also resets joint integration and velocity from current measured joints. Index trigger retains its gripper function.

Direct mode passes the latest relative Cartesian target directly to IK, without the extra 60 ms Cartesian filter, Cartesian slew limits, or bounded-lead catch-up pause. Joint velocity/acceleration limits, tracking validity, and geometry validation still apply; this does not promise zero physical following error. Restore the prior filtered, bounded-lead behavior by setting top-level `teleop_catch_up: true`. Pre-change planner and rig snapshots are in `feature_archive/bounded_catch_up/`.

Sleep now advances its quintic trajectory at 3x time, giving 15°/s peak velocity and 90°/s² peak acceleration limits. During sleep, arm motor profiles use 14 velocity counts and 5 acceleration counts, preserving gripper profiles and restoring prior arm settings on interruption. The post-trajectory settling timeout remains three wall-clock seconds. Opening remains at its previously configured 3x speed.

### Translation and rotation sensitivity

Tune independently with `--translation-scale` (alias `--scale`) and `--rotation-scale`. Both default to 1.0 unless configured as top-level `translation_scale` and `rotation_scale` in the profile. CLI flags override the profile for that session; effective values print at startup and are recorded in session metadata. For example, append `--translation-scale 0.5 --rotation-scale 0.25` to the hardware command for half-distance translation and quarter-angle rotation.

Translation scales controller displacement in metres from the current side-grip press. Rotation scales the relative rotation vector (axis-angle), preserving its axis rather than independently scaling Euler angles. Values must be finite and nonnegative; 0 freezes the relevant target component, 1 is one-to-one, and 2 doubles it. Each grip press captures fresh references. These settings do not change the gripper trigger, opening/sleep trajectories, or joint speed limits. Restart the session to change gains; no mid-motion gain changes are applied.

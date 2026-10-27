# Index-trigger gripper control

Implemented offline; no hardware connection, live launch, torque command or robot motion was performed during this work.

## User control

In aligned teleop, the side grip controls the arm clutch and the index trigger independently controls gripper aperture. The requested normalized aperture is `1 - clamp(trigger, 0, 1)`:

| Index trigger | Requested aperture | Left calibrated motor goal |
|---|---:|---:|
| Released (0) | 100% open | 3099 counts |
| 0.25 | 75% open | 2942 counts |
| 0.5 | 50% open | 2786 counts |
| 0.75 | 25% open | 2629 counts |
| Fully pressed (1) | Closed | 2472 counts |

Goals ramp to these apertures at no more than 0.18 calibrated aperture/s and 0.6 aperture/s² (18 percentage points/s and 60 percentage points/s²). These are software trajectory bounds, not measured motor speed. The existing motor profile policy remains. Small raw-count rounding errors are expected: the half-open round-trip is approximately 50.08%.

Releasing the side grip captures the measured arm pose once while index-trigger gripper control remains active. Arm re-grip still captures fresh controller and measured EEF anchors; it does not reset the gripper command or velocity. The archived arm catch-up option, when explicitly enabled, also does not pause the gripper.

Stop, stale input and faults override trigger commands and hold both arm and gripper. Missing tracking/trigger values are rejected, never converted to zero trigger. Released-side-grip commands still require fresh controller timestamps at the motor worker. Opening/sleep keep their captured gripper aperture and ignore trigger input. Sleep completion preserves the existing verified torque-off handling and forbids further goal writes. A requested fully open aperture is only acted on during active aligned teleop; READY/stop are not implicit opening commands.

## Electrical current cap and calibration evidence

Read-only inspection of session records, including `teleop_sessions/hardware_20261021_170144_416883/hardware.jsonl`, established these previous **left** gripper register readbacks:

- XM430-W350, motor ID 9; Operating Mode 5 (current-based position).
- Goal Current = 200 counts; Current Limit = 200 counts.
- Both correspond to approximately **538 mA**, using 2.69 mA/count. This is the configured electrical cap, not a measurement of motor current or jaw force. [ROBOTIS XM430-W350 control table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)

The left rig now records `arms.left.gripper_current_cap_counts: 200` as the maximum verified electrical ceiling. No current, gain, force, operating-mode or EEPROM register is written by this change. Runtime preflight must establish a valid actual readback (`mode == 5`, positive Goal Current no greater than Current Limit, both within model range and the configured ceiling). Lower valid existing settings are preserved. If no ceiling is configured on another rig, valid actual preflight readbacks establish that session's immutable baseline; absent/invalid readbacks reject live operation. Right-arm commissioning requirements still apply.

Every motor feedback cycle re-reads gripper Operating Mode, Goal Current and Current Limit and checks them against that baseline. Changed or missing/invalid readbacks fault the session; the code does not repair or increase them. Feedback/preflight records report counts and mA. No current readback was taken from real hardware during implementation, so these historical values do not establish the arm's present physical state.

The fingerprinted left LeRobot calibration is `/home/King/alohaX_lerobot_fork/.cache/calibration/aloha_solo/left_follower.json`, SHA-256 `b325481613ae0ff601faa3a7ef0ee2415b029742ceed1a5cdadf15fec9726d31`. Gripper calibration is LINEAR, closed at start_pos 2472, open at end_pos 3099. Endpoints must be finite and distinct; descending calibrations are supported. Existing file fingerprint checks remain. The configured twin finger coordinates are 0.002 m closed and 0.037 m open per finger; these are model coordinates, not newly measured physical jaw gaps.

## Offline validation

`tests/test_hardware_gripper.py` covers released/partial/full triggers and clamping; invalid input; calibration endpoints in both directions; speed and acceleration bounds through reversals and variable timesteps; absolute aperture with side grip released; arm clutch changes with gripper following error; stop priority; missing/expired timestamps; current-cap validity, ceiling and readback changes; and real adapter methods against fake register banks. Mock subprocesses exercise actual worker authority/stale/stop paths without serial access.

The installed LeRobot calibration functions were also exercised under the LeRobot Python environment, using `DynamixelMotorsBus.__new__` without construction/connect/serial I/O. Their closed, partial and open raw-count round-trips produced the table above. All gripper-specific tests are offline; physical speed, grip force and holding behavior remain unmeasured by this change.

Run tests:

```bash
cd /home/King/Desktop/QuestController
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl \
  /home/King/miniconda3/envs/quest_ros/bin/python -m pytest \
  tests/test_hardware_gripper.py -q
```

Validation result: **49 gripper-specific tests passed**; the complete hardware, Mink IK and planar-limit regression run passed **274 tests**. `git diff --check` and compilation in the Quest and LeRobot Python environments passed. Existing arm-only assertions were updated to distinguish arm clutch hold from independent aperture motion. Named-trajectory origin rebasing, sleep arrival and torque-off regressions pass.

Tune opening and closing speed together with `--gripper-speed 0.10` (calibrated aperture/second: 0.10 = 10 percentage points/s). The default is 0.18. The optional top-level profile key is `gripper_speed`; the CLI overrides it for the session, and the effective value is printed and logged. Values must be finite and positive. Acceleration remains bounded at 0.6 aperture/s². The fixed 20%-travel motor velocity ceiling is removed. The motor worker receives the same session speed and derives Profile_Velocity from the calibrated travel with 1.5× headroom, rounded upward to a finite nonzero register count. This replaces old finite gripper velocity caps; readback verification and opening/sleep profile preservation remain. Electrical current and acceleration limits remain in place, so physical motion still depends on those limits and load.

## Configurable acceleration update

`gripper_speed` now selects automatic acceleration `gripper_speed / 0.3` unless a numeric `gripper_acceleration` is supplied in the launch JSON/profile. The original 0.18 aperture/s default therefore retains 0.6 aperture/s²; speed 1.0 selects 3.333 aperture/s². The planner and motor worker receive the same resolved settings. Motor velocity/acceleration profiles are derived with 1.5× / 2× margin, converted through the calibrated motor travel and rounded upward; the previous fixed one-count gripper acceleration cap is removed. No electrical current or gain is increased.

For the left 627-count calibration span, speed 1.0 and auto acceleration select velocity/acceleration profile counts 61/18, with readbacks checked. Temporary opening/sleep arm profiles preserve these selected gripper values. The effective settings and motor profile counts are logged. JSON edits take effect after a full application relaunch, not the K recovery shortcut.

## Feedback excursions and recovery after a slipped object

The 2026-10-26 session `teleop_sessions/hardware_20261026_163032_965319/hardware.jsonl` ended with `Gripper feedback outside calibrated range`. Its last valid gripper sample was raw 2473 (closed calibration endpoint 2472), with a commanded fully closed aperture and verified 200-count electrical caps. The offending sample was not recorded, so its magnitude and the physical reason for the slip cannot be established from that log. The old strict `[0,1]` check raised out of the worker, closed its bus and made K recovery impossible because no motor worker remained alive.

Encoder feedback now has separate observation and command policies:

- Excursions within **two raw encoder counts** of either endpoint are logged as `endpoint_tolerance`. The normalized aperture used for the model/control is projected to `[0,1]`; actual raw counts and the unclamped aperture remain in `health.gripper_feedback`. These small excursions do not stop teleop or change calibration.
- Larger excursions up to **10% of calibrated travel** produce a recoverable `gripper_range` powered-hold latch. This envelope permits observation and bounded holding, not teleoperation with invalid data. The worker continues reading and reporting fresh encoders, torque, errors and electrical-cap registers. Ordinary reset cannot clear the latch and no motion resumes automatically.
- **K** is accepted only once feedback returns to the endpoint tolerance/valid range, health is good and the existing encoder-stability window passes. It clears the specific gripper-range latch and performs the existing return to the opening pose, preserving measured gripper aperture. While still outside, K reports that exact condition rather than labelling it an unspecified hardware fault.
- Excursions beyond that observation envelope remain hard faults, with raw count and calibration endpoints included in the diagnostic. Hardware/serial/current-cap faults remain nonrecoverable in-process.

Commands never extend beyond the calibrated motor endpoints, including exact-encoder hold and startup raw preloading. Startup refuses a gripper excursion beyond two counts before issuing goals/enabling torque; a large encoder jump cannot be converted into a large projected startup goal. No calibration file, current, gain or force setting is changed. This change addresses the software reaction, not the unverified physical cause of lost grip.

The update requires one full application relaunch; K cannot revive a motor worker that already exited under the previous code. The old offending-frame visibility gap is closed: diagnostic events/faults now include raw count, unclamped aperture, endpoint excursion and classification.

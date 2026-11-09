# Left-arm wrist calibration, sleep/opening targets, and startup faults

**Incident date:** 2026-11-07, Australia/Adelaide (UTC+10:30).  
**Report prepared:** 2026-11-07, following the operator's request for an extensive incident record.  
**Scope:** QuestController left follower, its measured-feedback MuJoCo twin, and the standalone ALOHA sleep/opening workflows.  
**Status:** Configuration fixes applied and checked offline. Earlier physical diagnostic moves and operator visual confirmation are recorded below. **The final restored opening wrist target has not been physically tested.**

This is a retrospective account of the conversation, saved session telemetry, configuration snapshots, and diagnostic artifacts. It is not a new hardware test or a declaration that the complete robot is commissioned. Filenames provide local session times; telemetry `t` fields are monotonic timestamps, not wall-clock dates.

## 1. Outcome and outstanding limits

The operator initially reported an inaccurate sleep pose, then observed incorrect wrist motion and repeated startup/runtime faults. Investigation exposed several interacting issues:

1. Standalone `sleep.py` and Quest loaded different left-arm calibration files. The same numerical wrist target therefore selected substantially different raw encoder goals.
2. Re-anchoring a calibration at one sleep pose had changed the raw meaning of other saved targets. The initial Quest opening wrist target became encoder **5448**, whereas the original-calibration ceremony target was **2654**.
3. The diagnostic twin's wrist bend direction and angle were wrong. The operator corrected its direction and selected a **+23° model-alignment adjustment** at a measured opening snapshot.
4. The original startup exception conflated torque-state and hardware-error failures. Readbacks eventually showed uniform torque-off together with input-voltage error flags on motor 9, and later motors 8 and 9.
5. Cleanup profile-restoration failures obscured the earlier fault with a second traceback.
6. Some direct-pose tests reached the commanded wrist-bend encoder goal but timed out because other loaded joints remained outside a stricter arrival tolerance.
7. A diagnostic reconnection to an already-powered arm disturbed the loaded elbow hold by replacing the existing motor goals with measured positions. A temporary diagnostic attachment path preserved the existing goals and retained the stationary threshold.
8. A successful geometric calibration did not establish the desired task pose. Preserving the tested encoder target produced an opening wrist model angle of **−30.26171875°**, while the operator wanted the historical opening model angle of **+53.26171875°**.

The final Quest configuration now uses the operator-selected **+53.26171875°** wrist opening target. Its final raw target is **1704**. The captured wrist sleep target remains **3555**. The verified wrist model mapping is preserved by an equivalent software-coordinate rebase, described in section 8.

The physical cause of the earlier voltage-error flags and serial read failures was **not conclusively established**. Successful later tests do not prove that those faults cannot recur. Full-range mapping, all task poses, collision clearance, long-term electrical behavior, and complete sequence arrival remain outside the evidence established here. Right-arm physical verification was not performed.

## 2. Devices and coordinate systems

### Motor identifiers

| Dynamixel ID | Name | Role |
|---|---|---|
| 1 | `waist` | Primary joint |
| 2 | `shoulder` | Primary joint |
| 3 | `shoulder_shadow` | Coupled shoulder motor |
| 4 | `elbow` | Primary joint |
| 5 | `elbow_shadow` | Coupled elbow motor |
| 6 | `forearm_roll` | Primary joint |
| 7 | `wrist_angle` | Wrist bend |
| 8 | `wrist_rotate` | Gripper-axis rotation |
| 9 | `gripper` | Gripper actuator |

The six-element model vectors use `waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate`. Thus motor **7** is model-vector element **4**, not element 7. Nine-motor raw arrays include both shadow motors and the gripper.

### Three distinct quantities

For a DEGREE-calibrated motor:

```text
calibrated_degrees = (direction * raw_encoder + software_homing_offset) * 360 / 4096
calibrated_degrees = model_degrees * rig_sign + rig_offset_degrees
model_radians = radians((calibrated_degrees - rig_offset_degrees) / rig_sign)
```

`direction` here comes from LeRobot's calibration JSON `drive_mode`; it must not be confused with the physical Dynamixel `Drive_Mode` register. The incident's wrist direction correction changed a **software model mapping**, not the motor's hardware drive-direction register.

The following have different meanings and must be recorded separately:

- **Calibration:** how measured encoder counts correspond to the model's angle and direction.
- **Pose target:** which model angle or physical encoder goal is intended for opening or sleep.
- **Tracking error:** the difference between the commanded motor goal and measured feedback.

A small tracking error can mean that a motor accurately reached the wrong task pose. It cannot, by itself, verify physical model alignment.

## 3. Entry state and initial sleep-pose investigation

The operator requested a MuJoCo calibration twin to show the intended wrist angle. An offline viewer and sliders were opened without connecting to motors. The operator then identified the elbow as the first joint needing adjustment and saved **93.8°**. The saved file was verified; this was a **selection artifact**, not an automatic update to live joint calibration.

Relevant artifacts:

- [Initial wrist selection](../../outputs/diagnostics/sleep_wrist_selection_20261107_170915.json).
- [Elbow selection, including 93.8°](../../outputs/diagnostics/sleep_wrist_selection_20261107_171018.json).
- [Captured sleep reference and calibration provenance](../../webxr_teleop/hardware/calibration/left_follower_sleep_20261107.reference.json).
- [Rig snapshot before the initial sleep re-anchor](../../webxr_teleop/hardware/calibration/rig_before_left_sleep_20261107.json).

The later physical sleep return used the captured **raw encoder reference**. The 93.8° slider selection must not be mistaken for a physically tested, universally applied elbow offset.

The captured nine-motor sleep reference was:

```text
[2045, 811, 811, 3122, 3122, 2020, 3555, 2063, 2601]
```

During direct pose tests, the current gripper opening was retained instead of commanding the reference's gripper value.

## 4. Startup fault: torque state versus hardware errors

The reported normal-launch invocation selected `webxr_teleop/hardware/launch.json`, the left arm, and `--twin`. This identifies the affected workflow; it is not a fresh launch-command handoff.

The initial exception was:

```text
Expected uniform torque state and no motor errors;
torque-off requires explicit startup
```

Inspection of `launch.json` and the session log showed `sequence=true` and `enable_torque=true`. Startup was already explicitly enabled through sequence mode. Adding `--startup` would not address the observed failure.

The inspection condition accepted either all nine torque registers off or all nine on when startup was allowed, but rejected any hardware-error bit. Therefore the original text could not distinguish mixed torque states from a hardware error.

After improved reporting, the operator saw:

```text
torque=[0, 0, 0, 0, 0, 0, 0, 0, 0]
hardware_errors=[0, 0, 0, 0, 0, 0, 0, 0, 1]
startup_allowed=True
```

That was **uniform torque-off**, not a torque-uniformity failure. Motor 9's hardware error was the blocker. A later blocked direct-pose attempt showed error `1` on both motors 8 and 9, again with all torque off and no requested pose command sent:

- [Blocked direct-pose attempt, 17:48:04](../../teleop_sessions/direct_pose_20261107_174804_288192/feedback.jsonl).

The earlier interpretation that torque-off alone explained the fault was incomplete. The diagnostics were changed to expose actual torque, error, and watchdog arrays.

## 5. Standalone sleep behavior and target correction

### Torque-off implementation

The external script was located at:

`/home/King/alohaX_lerobot_fork/lerobot/scripts/sleep.py`

Its prior sleep completion used a group torque-off write followed by aggregate verification. The operator requested uniform torque state after sleep. The script was updated with `ensure_uniform_torque_off`:

- Run after the existing motion-stop check.
- Write torque-off individually to every selected motor, including shadow motors.
- Continue attempting the other motors and arms if one write fails.
- Read each selected motor's torque register individually.
- Retry at most three times, with a short delay between attempts.
- Report remaining enabled or unreadable motors instead of printing successful completion.

This provides bounded attempts and verified reporting; it cannot guarantee torque-off when a device is electrically unreachable. It does not force torque-off from every exception or bypass the requirement for the sleep movement to stop first.

Three temporary offline fake-bus tests covered transient write failure, persistent failure without skipping other motors/arms, and unreadable feedback. All passed. These tests were run from `/tmp/test_sleep_uniform.py`; that temporary file is not a permanent repository regression test.

The external script backup is `sleep.py.~1~`. Report-time script SHA-256:

```text
28863796d8f291bbf4106c994360ef95a9615a954a08d22ccc1d1148602138f0
```

### Wrist sleep target in the standalone workflow

The operator's standalone command used `lerobot/configs/robot/aloha.yaml`. That file selected `.cache/calibration/aloha_solo`, while Quest selected a separate sleep-reanchored calibration JSON.

For motor 7, the same saved value **−113.115234375°** mapped to:

| Calibration representation | Software wrist homing offset | Raw goal |
|---|---:|---:|
| Original LeRobot calibration | −2048 | 761 |
| Sleep-reanchored Quest calibration | −4842 | 3555 |

The difference was **2794 counts = 245.56640625°**. A numerical target could not be copied unchanged between these calibration representations.

The standalone default `.cache/poses/rest_pose.json` left-wrist value was changed to **+132.451171875 calibrated degrees** under the original LeRobot calibration, recovering raw goal **3555**. Both its positional list and named dictionary were updated. Other saved joint targets were preserved, and the operator subsequently confirmed that the physical sleep pose was correct.

External pose path:

`/home/King/alohaX_lerobot_fork/.cache/poses/rest_pose.json`

Backup: `rest_pose.json.~1~`. Report-time SHA-256:

```text
126740e152adb654b912097de4b6f7375079e342b69442069ae21990bcffb4bd
```

These changes do not make the standalone LeRobot scripts automatically consume Quest's later operator-selected model opening override.

## 6. Voltage-error investigation and controlled reset

Read-only diagnostics checked serial ownership before access and used the same resolved-port advisory-lock convention as the controller. Access was refused while Dynamixel Wizard owned the port; Wizard was identified rather than killed. No motor target or torque write was sent by the health readers.

Observed progression:

1. Motor 9 reported hardware error `1`, packet alert `128`, and approximately **11.9 V**. Other motors initially reported no error.
2. Motor 8 later also reported hardware error `1` and packet alert `128`.
3. The operator first clarified that only software had been restarted. Restarting Python did not reset the motors.
4. After the operator reported restoring motor power, both error flags were still present, while torque remained off.
5. One hundred idle samples showed motor 7 clear at **12.0 V**, and motors 8/9 at **11.9 V** with persistent error `1`.
6. Startup configuration register 60 was read as **0** for motors 8 and 9; all motors' torque registers were **0**.
7. A guarded software reboot was sent only to motors 8 and 9. It refused occupied serial access, any torque-enabled motor, or nonzero startup configuration. It sent no goal, torque-enable, or configuration writes.
8. Fifty follow-up samples showed motors 7–9 error-free and torque-off. Motor 7/8 read **12.0 V** and motor 9 **11.9 V**.
9. Post-reboot reads confirmed the gripper's **Goal Current = 200** and **Current Limit = 200** counts were intact.

Hardware error bit `0x01` is the input-voltage error according to the [ROBOTIS XM430 manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#hardware-error-status70). Packet error `128` is an alert indication; it does not mean that a motor failed to communicate. The reboot responses themselves carried `comm=0, packet_error=128` while the devices were reporting the pre-reset alert. Subsequent reads confirmed clearance.

The sampled voltage being normal after the event does not establish the voltage at fault onset. The software reset clearing the flags is consistent with retained error state, but does not prove whether the reported power cycle fully removed device power, whether a transient occurred during restoration, or whether wiring/supply behavior contributed. No oscilloscope capture or independent electrical measurement was taken.

Also, observed shutdown configuration was **52 (`0x34`)**. The report therefore does not equate every input-voltage alert with a proven automatic torque-off event. One runtime fault explicitly showed all motors still torque-on while the gripper error bit was set.

Evidence:

- [Initial persisted motor-health snapshot](../../outputs/diagnostics/motor_health_20261107_latest.json).
- [Post-power health snapshot](../../outputs/diagnostics/motor_health_post_power_20261107.json).
- [100-sample idle voltage record](../../outputs/diagnostics/idle_voltage_post_power_20261107.json).
- [Guarded reboot and follow-up samples](../../outputs/diagnostics/guarded_reboot_20261107.json).

## 7. Physical pose tests and the misleading small-error result

### Opening source and diagnostic selection

The external `lerobot/scripts/opening_ceremony.py` was inspected. It uses `.cache/poses/ceremony_pose.json`, but also moves leader arms, manipulates grippers, and enters teleoperation. It was not run unchanged for the isolated wrist test.

Instead, the existing [direct-pose diagnostic](../../webxr_teleop/hardware/direct_pose.py) selected the original pose by motor name, retained the current gripper aperture, used slow motor profiles, and logged measured raw positions. A temporary wrapper supplied a passive twin and additional voltage/fault snapshots. The original-calibration opening wrist target was **2654**.

The bad Quest opening target **5448** was a conversion issue, not an established mechanical bound. In two earlier failed normal-launch sessions, motor 7 repeatedly stopped near **3594** while the command advanced. A physical limit/load was considered as an explanation, but contact or a hard stop was not independently measured. That hypothesis must not be promoted to a confirmed root cause.

### Direct-pose test chronology

| Session, local time | Requested operation | Speed profile | Recorded result |
|---|---|---:|---|
| `17:48:04.288192` | Opening | Not executed | Blocked in inspection: motors 8/9 error `1`, all torque off |
| `17:56:37.629919` | Original-calibration opening | 10 velocity counts, about 13.7°/s | Motor 7 reached the encoder goal; overall ten-second timeout |
| `17:59:58.082501` | Return toward captured sleep | 18 counts requested | Startup stationary-hold failure; no requested sleep pose target sent |
| `18:01:39.410396` | Captured sleep reference | 18 counts, about 24.7°/s | Arrived and settled; torque remained on |
| `18:02:07.218696` | Opening retry | 18 counts | Wrist reached command by about 3.53 s; other joints prevented overall arrival |
| `18:10:19.786928` | Opening with saved wrist twin correction | 18 counts | No motor errors; wrist reached command by about 3.53 s; overall timeout |

Links to full telemetry:

- [17:56 opening](../../teleop_sessions/direct_pose_20261107_175637_629919/feedback.jsonl).
- [17:59 blocked sleep return](../../teleop_sessions/direct_pose_20261107_175958_082501/feedback.jsonl).
- [18:01 completed sleep return](../../teleop_sessions/direct_pose_20261107_180139_410396/feedback.jsonl).
- [18:02 faster opening](../../teleop_sessions/direct_pose_20261107_180207_218696/feedback.jsonl).
- [18:10 opening with corrected twin](../../teleop_sessions/direct_pose_20261107_181019_786928/feedback.jsonl).

The completed sleep return ended at:

```text
[2046, 811, 811, 3121, 3121, 2019, 3554, 2065, 2480]
```

Relative to the captured targets, the largest reported arm error was approximately **0.18°**. The gripper value was deliberately preserved from the current session.

The 18:10 opening test's raw goal and final sample were:

```text
goal:     [2116, 1407, 1407, 2580, 2580, 2049, 2654, 2083, 2603]
measured: [2115, 1402, 1402, 2599, 2599, 2046, 2652, 2075, 2603]
```

Its maximum arm encoder error was about **1.67°**, on the elbow pair. Wrist rotation was about **0.70°** short in the final sample, while wrist bend was approximately **0.18°** from its commanded target. Voltage across the recorded motors stayed within **11.5–12.1 V**, and recorded hardware errors remained zero.

The direct-pose arrival check requires all arm motors within **six encoder counts (about 0.527°)** for a sustained interval. Consequently, the overall move timed out even though the bend wrist had reached its target. The faster test reached essentially the same final raw wrist position as the slower one, so speed/timeout did not explain the visually wrong twin alignment.

The normal Quest opening path has different arrival and timed-handoff behavior, documented in the [hardware README](../../webxr_teleop/hardware/README.md). A diagnostic timeout must not be conflated with a hardware-error fault or with normal Quest's `READY` state.

### Why the operator questioned the reported error

The operator correctly challenged statements that the wrist was within less than one degree while its pose looked wrong. Those numbers measured encoder tracking against the selected raw target. They did **not** establish:

- that the raw target was the intended historical opening pose;
- that the model axis direction was correct;
- that the model zero angle was correct;
- that other joints or the complete arm geometry were aligned.

The earlier reporting did not emphasize this distinction sufficiently. Later messages and the saved test summary explicitly separated encoder-goal tracking from operator-confirmed visual calibration.

### Already-powered startup disturbance

During the 17:59 sleep-return attempt, startup's current-position hold capture replaced existing loaded-joint goals. The elbow moved from roughly **2607 to 2615** counts in about **0.17 seconds**, crossing the six-count stationary bound. The requested sleep pose was not issued.

Subsequent read-only sampling showed stationary feedback with a persistent difference between measured positions and stored goals. Elbow P/I/D readings were **800/0/0**. This supports loaded steady-state tracking bias as an explanation; it is not a measurement of mechanical torque or a proof of a particular physical load.

The temporary diagnostic wrapper was changed to attach to an **already-powered stationary hold** without replacing its goals. It checked the same six-count excursion bound, checked unchanged `Goal_Position` values, and retained motor-health checks. Offline tests accepted stable feedback and rejected excessive drift or changed goals.

**This attachment change exists only in the temporary diagnostic wrapper. It was not propagated to production `LeRobotBus.startup_hold` during this incident.** Production behavior must not be inferred from the successful diagnostic attachment.

## 8. Twin sign/offset correction and task-pose restoration

### Operator-adjusted wrist mapping

The operator clarified that the twin's wrist position and drive direction were both flipped. The original diagnostic twin showed a measured motor-7 count of **2651** as approximately **+52.998046875°**. Flipping the left wrist model sign produced **−52.998046875°** and reversed its model motion direction.

The operator said this was closer, then used offline sliders to save:

| Joint | Snapshot raw count | Selected model angle | Additional slider adjustment | Model sign |
|---|---:|---:|---:|---:|
| `wrist_angle` | 2651 | −29.998046875° | +23° | −1 |
| `wrist_rotate` | 2074 | +0.17578125° | 0° | −1 |

The calibration selection stored equivalent offsets for both calibration representations. No motor was commanded by these sliders. During the 18:10 opening test the operator explicitly confirmed that **both corrected twin wrist position and movement matched the physical arm**.

- [Saved wrist calibration selection and operator validation](../../outputs/diagnostics/wrist_calibration_selection_20261107_180738_393721.json).
- [Opening test report with the visual confirmation](../../outputs/diagnostics/calibrated_opening_test_20261107.json).

This is evidence for the observed left-wrist motion and pose, not a full-range or right-arm verification.

### Propagation gap

Initially, the correction was saved only in a diagnostic profile. The operator continued launching Quest through `launch.json`, which still loaded the old `rig.json`. That normal launch could not benefit from the correction. This was a workflow gap in the incident response.

The main rig was subsequently reconciled with the tested diagnostic mapping, translating all primary-joint software offsets between the original and re-anchored calibration representations. The captured physical sleep reference and original-calibration opening raw goals were preserved at that stage. Right-arm settings, ports, mounting transforms, and current caps were preserved.

### Restoring the intended opening pose

Preserving motor 7's tested opening raw goal **2654** under the corrected model mapping produced **−30.26171875°**. The operator then explained that this was not the original intended opening wrist pose.

The historical model opening definition was **+53.26171875°**. An offline pose editor displayed that angle while keeping the verified calibration fixed. The operator saved it and requested propagation to Quest:

- [Final opening-pose selection](../../outputs/diagnostics/opening_pose_selection_20261107_182143_521328.json).

The resulting target is **1704 raw counts** after quantization. This is a **new physical opening target relative to the diagnostic tests**, even though it restores the historical model angle. It must not be reported as physically validated by the earlier moves to 2654.

### Equivalent software-coordinate rebase

With the previous active calibration, that desired opening angle would correspond to calibrated angle **−275.828125°**, outside LeRobot's accepted **−270° to +270°** decoding range. Simply changing `opening_rad` would have risked a new decoding fault.

The fix copied the software calibration to a new file and shifted the wrist software homing offset and rig model offset together:

| Quantity | Before final rebase | After final rebase |
|---|---:|---:|
| Wrist software homing offset, counts | −4842 | −2048 |
| Wrist model sign | −1 | −1 |
| Wrist rig offset, calibrated degrees | −222.56640625 | +23 |
| Coordinate shift | — | +2794 counts = +245.56640625° |

For any raw count `r`:

```text
old_model_deg = (((r - 4842) * 360/4096) - (-222.56640625)) / -1
new_model_deg = (((r - 2048) * 360/4096) - 23) / -1
old_model_deg = new_model_deg
```

Thus the physical-to-model mapping, including the operator's visual correction, is unchanged. The rebase changes software coordinate representation only. **No Dynamixel `Homing_Offset`, `Drive_Mode`, EEPROM, or current-cap register was modified by this configuration operation.** The previous software calibration file was retained.

## 9. Cleanup and communication diagnostics

The observed secondary failure was initially an `Opening Profile_Velocity write failed` traceback during worker cleanup. A later diagnostic identified the motor and SDK response:

```text
Opening Profile_Velocity write failed for wrist_rotate
(id=8, comm=0, packet_error=128)
```

The primary runtime fault, in a separate observation, showed:

```text
torque=[1, 1, 1, 1, 1, 1, 1, 1, 1]
hardware_errors=[0, 0, 0, 0, 0, 0, 0, 0, 1]
watchdog=[0, 0, 0, 0, 0, 0, 0, 0, 0]
```

These are distinct observations. The gripper hardware error initiated the reported fault, while a wrist-rotation alert complicated profile restoration. A cleanup traceback was not proof that profile restoration caused the original runtime failure.

Later normal-launch reporting included `Read failed: Present_Position` and cleanup `Read failed: Profile_Velocity`. The latest available normal-launch log was:

- [Normal-launch session `hardware_20261107_180849_706004`](../../teleop_sessions/hardware_20261107_180849_706004/hardware.jsonl).

It recorded normal startup, opening start, then a position read failure. Its last fresh feedback still used the stale wrist mapping and showed raw wrist feedback near **3592** while the model target advanced. No complete register snapshot at the precise communication failure was captured. A causal link between the stale target and the read failure was **not proven**.

Code changes in [bus.py](../../webxr_teleop/hardware/bus.py):

- Inspection faults now include torque arrays, hardware-error arrays, and startup permission.
- Runtime health faults include torque, hardware-error, and watchdog arrays.
- Profile-write errors include motor name, ID, communication result, and packet error.
- Group-read errors include the SDK communication result and participating motor IDs.
- Missing-feedback errors identify the motor name and ID.

Code changes in [worker.py](../../webxr_teleop/hardware/worker.py):

- Profile-restoration exceptions are caught and reported as cleanup failures.
- Bus closure still runs.
- The earlier fault can be reported without being replaced by an uncaught cleanup traceback.

These are reporting and cleanup improvements. They do not implement blind retries, automatic hardware recovery, ignored error flags, or a demonstrated fix for intermittent serial communication.

## 10. Final configuration at report time

Active profile: [rig.json](../../webxr_teleop/hardware/rig.json).  
Active left software calibration: [left_follower_wrist_aligned_20261107.json](../../webxr_teleop/hardware/calibration/left_follower_wrist_aligned_20261107.json).  
Calibration SHA-256: `ed8c0fee2338257b6acff151a72f8dd7223457710939feb56c19bcc3bc8322a7`.

Primary joint order: `waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate`.

```text
sign       = [-1, -1, 1, -1, -1, -1]
offset_deg = [-1.318359375, 90.087890625, 93.359375,
              6.943359375, 23.0, -0.52734375]
```

| Primary joint | Opening raw target | Captured sleep raw target |
|---|---:|---:|
| Waist | 2116 | 2045 |
| Shoulder | 1407 | 811 |
| Elbow | 2580 | 3122 |
| Forearm roll | 2049 | 2020 |
| Wrist bend | **1704** | **3555** |
| Wrist rotation | 2083 | 2063 |

Shadow goals are coupled to their primary targets through the existing worker/adapter checks. These rows do not specify a new gripper target. The diagnostic tests preserved current gripper aperture; normal recording-sequence gripper behavior retains its existing configured policy.

Preserved settings include the separate right-arm calibration and port, measured mounting transforms, gripper current cap of **200 counts**, sequence authorization provenance, and the distinction between `verified=true` mapping status and `sequence_verified=false` physical-sequence status. No independent right-arm test was claimed.

Historical provenance is retained under `sequence_sources.original_import`. An opening `operator_target_override` identifies the final selection. A source file's original calibrated degrees must not be treated as the active calibration's degrees without the documented conversion.

## 11. Validation record

The following checks were actually run during the incident; counts refer to separate runs, not one combined certification:

| Check | Result | Scope |
|---|---|---|
| Temporary sleep torque-off fake-bus tests | 3 passed | Bounded retries, failure isolation, unreadable readback |
| Startup regression tests after diagnostic message change | 25 passed | Offline adapters and mock worker startup |
| Startup plus sleep-torque tests after cleanup reporting changes | 36 passed | Offline regression |
| Reconciled mapping, pose import, sequence, speed, startup, and sleep tests | 72 passed | Offline regression before final opening-target restoration |
| Final wrist reconciliation and pose-import tests | 13 passed | Final selected target and calibration representation |
| Final actual LeRobot encode/decode check | 101 sampled poses passed | Sleep-to-opening samples, no calibrated-range fault, encoder quantization bound |
| Final planner path check | Passed | Model joint/plane checks for captured sleep to selected opening |
| Guarded motor reset follow-up | 50 samples clear | Motors 7–9 torque-off and error-free after reset |
| Physical test with corrected twin | Operator confirmed wrist direction/position | Earlier raw-2654 opening only |

[Regression tests](../../tests/test_rig_wrist_reconciliation.py) anchor the final opening target, preserved sleep raw targets, operator-selected wrist alignment, corrected sign, and calibrated decoder range.

The final driver check used actual LeRobot calibration conversion methods on an unconnected adapter; it did not open a serial port. The final planner check was offline and does not establish physical collision clearance or reachability.

Some initial test runs failed because sandbox restrictions prevented mock subprocess socket construction. Another initial run encountered an incompatible auto-loaded ROS `launch_testing` pytest plugin. Relevant tests were rerun outside the sandbox, with plugin autoload disabled, and their passing results are recorded above. These environmental failures were not treated as robot behavior.

## 12. Backups, artifacts, and reproducibility limits

### Configuration snapshots

- [Before the initial sleep re-anchor](../../webxr_teleop/hardware/calibration/rig_before_left_sleep_20261107.json).
- [Before wrist reconciliation into normal Quest configuration](../../webxr_teleop/hardware/calibration/rig_before_wrist_reconciliation_20261107.json).
- [Before final opening target selection and software-coordinate rebase](../../webxr_teleop/hardware/calibration/rig_before_opening_selection_20261107.json).
- [Intermediate sleep-reanchored calibration](../../webxr_teleop/hardware/calibration/left_follower_sleep_20261107.json).

These are historical evidence, not interchangeable launch presets. Restoring a rig snapshot alone while mixing calibration or pose files from another stage can recreate the original mismatch.

### Diagnostic artifacts

- [Post-opening hardware health](../../outputs/diagnostics/opening_test_after_health_20261107.json).
- [Hold sampling after the blocked sleep return](../../outputs/diagnostics/hold_after_failed_sleep_20261107.json).
- [First opening voltage samples](../../outputs/diagnostics/opening_test_health_20261107_175636.jsonl).
- [Successful sleep-return samples](../../outputs/diagnostics/opening_test_health_20261107_180138.jsonl).
- [Faster opening samples](../../outputs/diagnostics/opening_test_health_20261107_180206.jsonl).
- [Corrected-twin opening samples](../../outputs/diagnostics/opening_test_health_20261107_181018.jsonl).

Temporary readers, editors, test scripts, and the powered-hold-preserving diagnostic wrapper were created under `/tmp`, including `/tmp/quest_test_opening.py`. They are not production entry points or guaranteed to survive cleanup/reboot. The persistent JSON/JSONL artifacts and session logs are the primary retained evidence. Several artifact directories and external LeRobot changes were untracked at report time; writing this report did not create a Git commit or archive those external repositories.

## 13. Remaining work and interpretation rules

1. **Physically test the final opening target separately.** Operator visual confirmation and successful tracking in earlier tests applied to raw 2654, not the final raw 1704 target.
2. **Do not declare the full sequence verified.** Loaded elbow and wrist rotation offsets prevented the strict diagnostic arrival check from succeeding. No gains, current caps, or tolerances were raised to conceal that result.
3. **Do not treat the serial fault as solved.** Current successful reads and later fault-free moves are evidence of recovery, not an explanation of the earlier `Present_Position` failure. Use the improved SDK/motor diagnostics if it recurs.
4. **Do not treat a normal sampled voltage as proof of a stable supply during fault onset.** Electrical transients and wiring behavior were not independently measured.
5. **Keep calibration and pose edits separate.** A mapping correction can preserve the physical pose while changing its model angle; restoring a model task angle may deliberately change the physical target.
6. **Keep the external scripts distinct from Quest.** The standalone sleep file and pose correction persist in the external LeRobot checkout. The later operator-selected opening override is stored in Quest's rig provenance and does not silently rewrite the external ceremony JSON.
7. **Production powered-start behavior remains a separate item.** The preserved-existing-goal attachment used for a diagnostic reconnect was not installed into the production startup implementation.
8. **Recheck launch endpoints before future hardware starts.** Inspect the configured control socket, supervisor/worker owners, HTTPS listener, and serial ownership. Only clean an owned, unchanged stale socket after proving connection refusal and no active owner. Never kill motor workers or unlink a live socket to force a second controller to start.

During this incident, control endpoint and HTTPS availability were checked before authorized diagnostic moves, and live diagnostic viewers were requested explicitly with `--twin`. Endpoint cleanup/inspection did not serve as authorization for motion. Documentation of those checks is historical; it is not a statement that endpoints or hardware state are still available now.

No new motor connection, reset, torque operation, or physical movement was performed while writing this report. The final configuration is recorded; **present hardware torque and electrical state must be measured rather than inferred from this document**.

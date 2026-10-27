# Dual-arm Quest hardware control

The hardware supervisor now supports automatic sequence startup with `side: "both"`. The supplied [`launch_dual.json`](../webxr_teleop/hardware/launch_dual.json) selects both arms, the sequence and the measured-feedback MuJoCo twin, with the existing sensitivity and bounded motion settings. It references the hardware profile separately, just like `launch_left.json`.

## Control behavior

- Each arm has its own motor worker, encoder feedback, current-cap readbacks, clutch generation and controller/EEF reference. Each side grip controls the matching arm; each index trigger independently controls its matching gripper. Gripper calibration and electrical caps remain per arm. No current, gain or force setting is copied or increased.
- Each grip press captures that controller's current pose and the matching measured EEF pose. Release captures that arm's powered hold and invalidates its outstanding motion. Controller movement during release is excluded from the next motion. A clutch-generation change on one side no longer discards the other side's valid planned target.
- Both workers must report fresh stable startup holds before the shared opening trajectory begins. Initial opening still has the ten-second handoff to READY. Release both grips and press left X to align and hand over both arms.
- Each worker replaces queued targets after its encoder read and checks the original controller-source deadline before serial writes. A stale source rejects the whole planned dual target; a single-arm tracking loss, stale feedback, motor fault or B/Space/local stop interrupts both arms. Reconnection alone does not resume movement. Deadlines remain 150 ms for controller/commands and 100 ms for feedback.
- Left Y held for one second or terminal S requests synchronized sleep for both arms. Sleep ignores headset loss after acceptance. PARKED requires both arms to reach their configured sleep poses and both workers to confirm torque-off readbacks. Stop/fault handling still takes priority.
- K restarts both arms to their configured opening poses after recoverable IK/hold-drift/gripper-range faults, with fresh stationary healthy encoders on both sides. It uses the existing bounded joint trajectory and waits for measured arrival. It does not enable torque after sleep.
- The twin displays both arms from measured joint feedback using the configured base transforms. It does not drive the robot. JSON tuning settings currently apply to both arms; calibration, mapping, poses and electrical ceilings remain in each arm's hardware-profile entry.

## Timestamp correction

Dual process tests exposed a supervisor race that could falsely stop on fresh feedback: the supervisor sampled time before draining both workers, so a newly received feedback timestamp could appear a few microseconds in the future. The captured failure had right feedback age **−0.011 ms** and left age **6.016 ms**, but entered `Feedback watchdog expired`.

The supervisor now samples time after feedback collection and again after the controller snapshot. It still rejects genuinely future/expired timestamps under the existing deadlines. A deterministic process test produces feedback during every supervisor iteration to reproduce this ordering without physical devices.

## Existing rig commissioning status

The user reported the mounting geometry on **2026-10-27**: base centers **0.815 m apart**, shoulder to shoulder on the same horizontal line and height, both facing forward. In the shared world frame (+X right, +Y forward, +Z up), the horizontal origin is midway between the centers. `rig.json` places the left base at **[-0.4075, 0, 0.02] m** and the right at **[0.4075, 0, 0.02] m**, with identical +90° Z base orientations so robot-local +X forward points along world +Y. The existing model base Z=0.02 m above the table is retained; no new absolute mounting height was supplied.

The hardware planner, measured twin, mirror and recording obtain those transforms from the profile. The default mock profile and standalone dual MuJoCo scene use the same spacing. The dual twin starts from a rear viewpoint so both forward-facing arms appear shoulder to shoulder. Historical diagnostic candidates and archived session geometry are preserved.

New recording manifests include `base_positions`. IK replay restores that source placement before applying its recorded world-space targets. Older recordings without mounting metadata retain their documented original 64 cm dual layout, so changing the live scene does not change the historical singularity regression or silently reinterpret its targets.

The mounting-placement conflict is resolved. On 2026-10-27 the user explicitly requested copying the left joint mapping and mapping/sequence verification settings to the right. The right now matches the left: `verified: true`, `sequence_verified: false`, `sequence_authorized: true`. Notes and provenance identify this as an operator-approved copy, not an independent right-arm physical test. Live dual startup still checks each selected arm's calibration fingerprint and sequence authorization before creating workers.

The right opening/sleep targets retain their imported ALOHA provenance. Right-arm sequence authorization is now copied from the left under the explicit user instruction; actual mode/current-cap readbacks must still satisfy the existing checks. The verified left 200-count current ceiling is not asserted for the right arm. Simulation passing does not verify physical joint directions, current limits or inter-arm collision avoidance; the existing geometry checks cover configured joint limits and workspace planes, not general inter-arm collision detection.

No live launch command is handed off here. Endpoint/process/HTTPS availability must be checked according to AGENTS.md before any subsequent live command or launch. No robot was connected or moved during implementation.

## Offline validation

Regression coverage includes both motor workers' delayed-read replacement/stop/release behavior, independent arm and gripper references, per-side clutch result admission, whole-result source expiry, dual JSON defaults, right commissioning and coincident-base rejection, opening/READY/teleop, either controller's tracking loss stopping both arms, motor-worker loss, synchronized sleep with both torque-off acknowledgements, long dual keyboard recovery after IK failure, and the measured-feedback twin on both sides.

The process tests use mock motor buses and private loopback sockets. The twin test uses a virtual X display. They establish software ordering and state transitions, not physical tracking performance or safe real-world geometry.

Validation result: **352 hardware regression tests passed**, including both twin configurations under Xvfb. The focused dual/sequence/restart run also passed all 29 cases after the timestamp correction. Python compilation in quest_ros and lerobot and `git diff --check` passed. No deadline was relaxed to obtain these results.

After the user mounting update, 66 mounting/planner/pose-import/plane/dual-teleop/sensitivity checks passed, as did 13 focused dual/twin/mock-sequence checks. All 15 mounting and recorded-IK replay checks passed after preserving historical source layouts. Recording-metadata and mock dual-sequence integration checks were then run against the new manifest format. No robot was connected or moved.

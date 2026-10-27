# Increased HOLD interruptions: 2026-10-26 investigation

This investigation used recorded sessions and source inspection only. No physical robot connection, launch, settings change, or watchdog relaxation was performed.

## Established findings

The post-fix session `hardware_20261026_165811_759421` records six tracking-related hold entries: three supervisor controller-age timeouts and three motor-side `Controller target expired at motor execution` holds. There are no gripper range faults or endpoint-tolerance events in this session; recorded gripper feedback is `in_range`. A later sleep settling failure is a separate issue and also occurred in earlier sessions.

Compared with the preceding `hardware_20261026_163032_965319` session:

| Metric during ARMED teleop | Before | After |
|---|---:|---:|
| Controller freshness upper bound, p95 | 61.94 ms | 67.55 ms |
| Controller freshness upper bound, p99 | 73.36 ms | 98.87 ms |
| Planner solve duration, p99 | 4.97 ms | 5.95 ms |
| Serial goal write/readback, p95 | 3.07 ms | 3.09 ms |
| Serial goal write/readback, p99 | 3.17 ms | 3.24 ms |
| Motor feedback period, p99 within continuous ARMED segments | 21.19 ms | 22.30 ms |

Arm settings match: translation/rotation gains 1/1, joint speed 18°/s, acceleration 60°/s² and catch-up disabled. Both sessions use gripper speed 1.0 and acceleration 3.333 aperture/s². The new feedback metadata increased median feedback packet size from approximately 1934 to 2366 bytes. No feedback sequence gaps were observed within active segments. That does not rule out all delays, but there is no evidence here of a feedback queue overflow.

The increased tail of input freshness, rather than a large serial or planning slowdown, is the clearest measured change. These sessions used different operator motions and are not controlled experiments; association with the previous patch does not establish that the patch caused the input jitter. The gripper-feedback patch added classification/math/telemetry, not extra serial register reads or altered arm IK gains/limits.

## A fragile planner-to-motor freshness handoff

Controller `received_t` in hardware mode is actually the server-issued anti-replay challenge timestamp. Challenges are generated every 50 ms. Consequently its age includes challenge-token lifetime and communication/client scheduling, not just pose capture or packet receive age. The same 150 ms bound is checked at server admission, supervisor tracking, planner input and motor execution. Existing logs cannot distinguish Wi-Fi delay from browser/XR stalls or server scheduling within that upstream age.

Supervisor target admission checks `input_t` (planner job time) but does not check `controller_t` (source freshness). A result can have a fresh planner job and an already expired controller source. The worker rejects that source, correctly preserving the stale-input rule, but latches the entire session even when a newer valid controller sample is already available on the PC.

Examples from the post-fix session:

- At elapsed 21.09 s, a forwarded result's controller source was 151.16 ms old while the latest controller sample was 99.93 ms old. The subsequent worker hold reached the supervisor at 21.13 s.
- At 56.63 s, a result source was 147.56 ms old while latest input was 96.67 ms old. The small remaining freshness budget was exhausted before motor execution, producing a hold at 56.67 s.
- At 72.20 s, a forwarded result source was 153.99 ms old while latest input was 52.70 ms old. The motor hold reached the supervisor at 72.23 s.

These demonstrate pipeline amplification of input jitter: newly arrived input does not immediately replace an older in-flight planned target. The motor worker also snapshots incoming commands **before** its blocking feedback/register reads, introducing another window in which a newer replacement can arrive but wait for the next iteration. Queue arrival times and feedback-read duration are not logged, so their exact contribution is not established. Stop-triggered profile restoration delays feedback delivery; an older cached feedback timestamp during those transitions must not be misinterpreted as proof of a long feedback read.

The three supervisor age timeouts reached approximately 150–152 ms for both controllers together. They are separate from the expired-target handoff cases. Since both controllers share the challenge timestamp, this is a shared freshness expiration, not evidence of an individual controller's optical occlusion.

Some holds occurred with side grips released: aligned gripper control remains active independently, so the existing ARMED freshness policy still applies. The last fix did not change this policy.

## Next corrective work

First prevent forwarding targets whose controller source has already expired, consume the newest queued command after feedback reads, and instrument actual packet receipt/interarrival, command admission and feedback-read timing. Verify the near-deadline handoff with delayed-input replay. Changes must retain the 150 ms stale-input limit, bounded powered hold and anti-replay guarantees; simply raising the timeout or relabelling receipt time as capture time would hide the problem.

Increasing challenge frequency could reduce the extra token-lifetime component without increasing the timeout, but its benefit and server/client load should be tested offline before applying it. Genuine input gaps still require stopping. A guaranteed reduction in all six recorded holds is not established from current logs.

## Reproduce

Run `docs/tracking_validation/investigate_holds.py` with the quest_ros Python environment. It reads logs only and prints the structured evidence saved in `hold_investigation_20261026.json`. Feedback intervals are measured within continuous ARMED teleop segments; pauses between re-arming episodes are excluded. Repeated cached feedback/results are excluded from per-message timing metrics. Controller-age samples are supervisor ticks, so cached input ages repeat by design. An incomplete trailing JSONL line is ignored when inspecting an active logger.

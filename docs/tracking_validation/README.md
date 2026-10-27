# Quest tracking redesign — offline validation, 2026-10-21

No physical motor connection or motion was performed for this change. New physical tracking performance remains unverified.

## Diagnosis

Read-only summaries of four recent sessions are in `recent_logs.json`; regenerate with `summarize_logs.py <hardware.jsonl> ...` under quest_ros. Samples are supervisor ticks, so cached messages can repeat.

- Controller freshness was approximately 35 ms median / 59 ms p95. This is **an upper-bound freshness age**, measured from server challenge issuance. Challenge messages are issued every 50 ms. It is not XR capture age, one-way network latency, or a measurement of headset rendering delay. Existing logs cannot separate these components.
- Planner-start to supervisor observation was approximately 18.8 ms median / 19.7 ms p95. Existing logs do not separate computation from scheduler/IPC waiting. A 20 ms supervisor cadence is consistent with this delay, but the logs do not prove that all of it is scheduling. The supervisor now polls at 100 Hz; the motor loop remains 50 Hz. The resulting physical latency reduction still needs measurement.
- Reported goal versus encoder error reached approximately 2.1–2.46° p95 across the sessions (maximum across six joints per sample). This establishes real following error, but cannot identify its cause: profiles, load, backlash, model mapping, gains, or communication. It does not measure optical EEF pose error.
- The old planner bounded differential IK to 10°/s, then applied a second 9°/s, 24°/s² joint shaper to each small IK step. That small step became a new stopping-distance target every cycle. In the controlled replay the final output peaked at only 3.10°/s despite a nominal 9°/s cap. Keeping 9°/s and 24°/s² but enforcing them together inside IK reduces RMS tracking error. The duplication was therefore a demonstrated software bottleneck.
- Existing arm motor profiles were 14 velocity counts (19.236°/s) and 4 acceleration counts (85.8308°/s²). Motor readbacks in the reviewed sessions showed velocity-based Drive Mode (values 0 and 1; reverse-direction bit retained). These old readbacks do not verify a future session.

ROBOTIS documents that new Goal Position writes regenerate the motion profile using current movement velocity. This is **not evidence of a restart from zero velocity on each write**. Profile regeneration may still affect tracking, but attributing the observed lag to it requires physical measurements. Identical quantized goals are now omitted; changed goals retain write/readback verification. For a future diagnostic, compare Present Position/Velocity with Position/Velocity Trajectory registers (140/136) during streamed goals. Do not assume the simplified replay plant models firmware behavior.

Register references: [XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/) and [XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/). Velocity-based Profile Velocity uses 0.229 rpm/count = 1.374°/s/count; Profile Acceleration uses 214.577 rpm/min/count = 21.4577°/s²/count. Drive Mode bit 2 changes these to time-based semantics. Zero counts have special unlimited semantics and are never selected here.

## Implementation

One differential-IK QP simultaneously constrains arm joint speed and acceleration. Its result is the arm command; there is no second arm trajectory filter. Geometry/joint checks remain active and can stop an infeasible motion. Gripper shaping is independent and unchanged. Named opening/sleep trajectories retain their existing analytic timing and checks.

Default teleop speed is 18°/s; acceleration is 60°/s². These are a provisional commissioning starting point, below the existing 30°/s, 90°/s² opening envelope. That comparison is a rationale, not proof of safe operation in every configuration or with payload. Motor profiles use at least 1.5× velocity and 2× acceleration margin, rounded upward to register counts: defaults 20 counts = 27.48°/s and 6 counts = 128.7462°/s². These are execution headroom, not additional trajectory generators in software. Actual servo profiles necessarily still shape physical response.

Current Drive Mode is checked before teleop profile writes; time-based mode and automatic torque-on are rejected without changing Drive Mode. Reverse bits are preserved. RAM profiles are read back, gripper values preserved, and saved values restored on mode transitions, hold/shutdown and before successful sleep torque release. Current, gains, operating mode and EEPROM are unchanged. Profile-write failure prevents continued motion. Stop/communication failure cannot guarantee physical holding.

The supervisor has at most one planner request outstanding, based on its latest controller snapshot. The motor worker coalesces queued position commands to the newest epoch/sequence while preserving control operations and stop priority. Grip generations invalidate planner results across release/re-press. The motor-side release barrier also rejects delayed earlier-motion targets arriving in a later IPC batch. Each press anchors to fresh controller and measured EEF poses; release captures measured powered hold without waiting for IK. Motion while released does not carry forward. The archived catch-up behavior remains disabled by default.

Stale controller data is checked at planning and motor execution, including a final check after profile configuration. Existing command TTL (150 ms), feedback TTL (100 ms), stop priority, geometry checks, sleep arrival/torque-off behavior and encoder-driven MuJoCo twin remain. Release/stop intentionally override smooth deceleration to request the measured hold immediately; physical inertia and stopping distance are not eliminated.

## Telemetry

New planner result fields: `planner_duration_ms`, `planner_queue_age_ms`, `completed_t`; per-side diagnostics include desired position/quaternion, `measured_eef_from_encoders`, controller freshness upper bound and requested joint velocity in rad/s. EEF measurement is **forward kinematics from encoders and the configured model**, not an independent tracker measurement.

New worker feedback `telemetry`: motor execution timestamp; target age from planner start; input snapshot age; controller freshness upper bound; requested joint velocity; finite-difference executed-goal and measured-encoder velocity estimates; preceding executed goal versus encoder following error; serial write/readback duration; and whether a new quantized goal was written. Velocity estimates are explicitly labelled and are not Dynamixel Present Velocity register readings. Timing measures host intervals, not internal firmware actuation time. `motion_profile` events report selected counts, converted units, Drive Mode and verified readback status. Existing session JSONL contains these messages.

## Reproducible comparisons

Run from repository root:

```bash
MUJOCO_GL=egl conda run -n quest_ros python -m webxr_teleop.hardware.replay_tracking \
  --output docs/tracking_validation/synthetic.json
MUJOCO_GL=egl conda run -n quest_ros python -m webxr_teleop.hardware.replay_tracking \
  --session teleop_sessions/hardware_20261021_162511_778874/hardware.jsonl \
  --output docs/tracking_validation/recorded.json
```

The baseline planner is preserved in `tests/fixtures/tracking_baseline_planner.py.snapshot`. The comparison uses identical controller frames, model, gains and initial joints for each case, at the original sample cadence; it does not credit the new supervisor frequency with an unmeasured benefit. Synthetic input has two independent strokes, each 6 cm translation and 15° rotation over 0.6 s, with release, repositioning and re-grip. RMS errors include the time the grip remains held, before each plant update.

| Synthetic, ideal encoder following | Position RMS | Rotation RMS | Peak joint speed |
|---|---:|---:|---:|
| Old two-stage, 10 then 9°/s, 24°/s² | 27.61 mm | 11.41° | 3.10°/s |
| Coordinated, same 9°/s, 24°/s² | 24.72 mm | 10.18° | 9.00°/s |
| Coordinated, 18°/s, 60°/s² | 16.65 mm | 7.01° | 18.00°/s |

The proposed defaults reduce position/rotation RMS by approximately 40%/39%. Acceleration stayed within the configured bounds. Release command error relative to measured joints was zero. These are tracking-error proxies for lag, not measured end-to-end latency in milliseconds.

The explicitly estimated servo plant (60 ms first-order response, velocity/acceleration caps; **no gravity, friction, load, backlash or Dynamixel firmware simulation**) gives 27.93 → 17.06 mm and 11.45 → 7.16°. It is sensitivity analysis, not a hardware prediction or stopping guarantee.

The first uninterrupted aligned segment of the latest recorded session has 870 frames and **4× translation and rotation gains**. Its unconstrained desired poses produce very large errors: ideal position RMS 940.67 → 877.63 mm; rotation 104.94 → 100.73°. Both paths remain geometrically checked. The modest improvement does not solve these aggressive/unreachable targets. This replay intentionally retains the recorded gains; it must not be presented as evidence that the real robot will closely follow all recorded motion. Returning gains to 1× and using reachable, small strokes is appropriate for initial verification.

Offline regressions cover speed/acceleration, reversals, fresh anchors, release hold, delayed commands across release, latest-target coalescing, profile units/margins/readbacks/restoration and stop/stale/sleep behavior. A mock process verifies no old target resumes after release. No offline test proves zero physical overshoot or motor following delay.

## Tuning and staged physical verification

| CLI flag | Profile key | Meaning/default |
|---|---|---|
| `--teleop-joint-speed` | `teleop_joint_speed_deg_s` | 18°/s per arm joint; finite, >0 and ≤60 |
| `--teleop-joint-acceleration` | `teleop_joint_acceleration_deg_s2` | 60°/s² per arm joint; finite, >0 and ≤180 |
| `--translation-scale` | `translation_scale` | metres/metre gain; start at 1 |
| `--rotation-scale` | `rotation_scale` | radians/radian gain; start at 1 |

CLI overrides are session-local and propagate to planner and motor worker. Accepted tuning ranges are software bounds, not certified mechanical limits. No independent motor-profile tuning is needed for these flags; margins are derived automatically. Changing teleop limits does not change opening or sleep speed.

1. Before any launch, follow repository AGENTS.md endpoint/process/8443 checks. Use the left arm with clear workspace, supported start and an operator able to stop. Start at `--teleop-joint-speed 9 --teleop-joint-acceleration 24 --translation-scale 1 --rotation-scale 1`. Opening itself retains its existing faster trajectory; these flags govern teleop only.
2. Wait for READY, release grips, press left X. Check twin versus actual arm orientation and joint directions. Verify the session `motion_profile` event and readbacks. Abort on mapping mismatch or readback failure.
3. Use one short reachable translation, then one rotation, then a reversal. Release during movement; verify measured hold captures promptly with no continued stale goal progression. Reposition while released, then re-grip: no inherited displacement. Confirm B/Space stop and existing recovery; test tracking loss at low speed with support. Physical overshoot must be measured separately from command capture.
4. Inspect controller age, planner duration/queue age, execution target age, encoder velocity estimates, goal error and FK EEF error. Compare like-for-like motion. If goal error grows while planner/output latency is low, investigate motor/load/profile behavior before raising speed further. Use trajectory/velocity registers in a dedicated future diagnostic to establish firmware behavior.
5. Only after the lower setting behaves correctly, repeat at 18°/s and 60°/s², initially with the same small strokes and gains. Check errors, motion direction, stops and release response, then expand reachable motion gradually. Do not infer success from an MP4 alone.
6. Verify left Y held for one second requests the existing sleep sequence and successful arrival disables all selected-arm torque with readback. Ensure physical support as required when torque is removed. Test stop priority before relying on sleep as a routine exit.

## Validation outcome and launch handoff

The full hardware + Mink IK + planar-limit suite passed **224 tests**. After the final generation-priority refinement, the affected motion, sleep recovery, sleep torque and manual-start suites passed **42 tests**. The refinement ensures an obsolete clutch packet cannot consume a newer explicit reset/sleep arm operation. Existing tests with superseded ceremony-speed expectations and mock read signatures were updated; sleep/ceremony behavior itself was retained. `git diff --check` and worker compilation under the separate LeRobot Python environment passed.

Endpoint handoff check: no live hardware supervisor/motor-worker owners; `/tmp/quest-aloha-1005/control.sock` successfully reserved and released with inode/ownership checks; port 8443 successfully bound and released. No serial bus was opened. Availability is only established at inspection time; normal launch ownership checks still apply.

For the first physical verification stage (9°/s, 24°/s²):

```bash
cd /home/King/Desktop/QuestController
conda run -n quest_ros --no-capture-output \
  python -m webxr_teleop.hardware.run \
  --mode live --side left --sequence --twin \
  --profile /home/King/Desktop/QuestController/webxr_teleop/hardware/rig.json \
  --translation-scale 1 --rotation-scale 1 \
  --teleop-joint-speed 9 --teleop-joint-acceleration 24
```

After that stage passes, use the same command with `--teleop-joint-speed 18 --teleop-joint-acceleration 60` for the proposed faster settings. The hardware launch starts the Quest HTTPS server and prints its current URL. Wait for READY, release both grips and press left X; side grip moves the arm and index trigger operates the gripper. B/Space stops, and left Y held for one second requests sleep.

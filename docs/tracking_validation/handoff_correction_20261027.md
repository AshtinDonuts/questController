# Controller target handoff correction — 2026-10-27

Implemented after the [HOLD investigation](hold_investigation_20261026.md). Implementation and validation use source, recorded logs and simulated buses only. No real robot was connected or moved.

## Changes

- Supervisor admission checks both job freshness and original controller-source freshness using the current admission time. A fresh job containing an expired active controller source is discarded as a unit before forwarding to any selected arm. The next job uses the latest available hands. Discarding never refreshes a motor watchdog or replays a target.
- The motor worker merges messages received before and after its blocking encoder read, then selects the newest command. Stops and clutch releases from either batch retain priority over commands. Thus a valid newer target that arrives during that read no longer waits an extra cycle behind an expired target.
- The 150 ms freshness deadlines, pre-write expiry check (including time spent configuring profiles), stop/release barriers, fixed powered hold and explicit re-arm requirements remain in force. No current, gain, motion-limit or challenge-frequency settings changed.
- Logs distinguish accepted packet receipt/interarrival from challenge issuance. `packet_received_t` and `packet_interval_ms` are server measurements, not optical capture time or one-way network delay. Safety still uses the original challenge timestamp. Motor logs add measured feedback-read duration, executed command sequence and admission-to-execution duration. Discards record source and latest-controller age upper bounds.

## Offline evidence

`tests/test_hardware_target_handoff.py` runs the actual worker loop against a deterministic simulated clock/bus and injected IPC batches. It requires no robot, network sockets or wall-clock sleeps.

| Delayed-input scenario | Previous handoff | Corrected handoff |
|---|---|---|
| Old target before a 30 ms encoder read; fresh replacement arrives during the read | Old source reaches 170 ms age and latches HOLD; replacement waits | Fresh replacement executes at 20 ms source-age upper bound; remains ARMED; old target is never written |
| Stop arrives during the read, followed by a new target | Stop waits for another cycle | Current cycle captures measured hold; queued targets are never written |
| Grip release arrives during the read with a queued target | Release waits for another cycle | Current cycle captures arm hold and rejects the queued motion |
| No fresh replacement, or target expires during profile setup | Stale input must stop | Still latches HOLD before a stale goal can be written; subsequent fresh packets do not re-arm it |

The recorded 151.16 ms and 153.99 ms source examples now fail supervisor admission despite fresh job timestamps. A 147.56 ms source is still admissible at that instant: it may expire before execution unless replaced in time. This correction does not claim to eliminate every recorded motor hold. The three separate supervisor input gaps reaching 150–152 ms remain genuine deadline failures under the existing policy and still require stopping.

These checks establish target replacement and stop ordering, not real-world Wi-Fi improvement, physical servo response or disappearance of all interruptions. The added timing fields are intended to distinguish those causes in the next authorized physical session. Restart the application to load the corrected code; keyboard K recreates the planner but does not reload the supervisor/worker code.

Reproduce focused checks with the quest_ros Python environment:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl python -m pytest \
  tests/test_hardware_target_handoff.py tests/test_hardware_freshness.py -q
```

Validation: 329 hardware tests passed, including the 16 focused handoff/freshness checks. Another 59 WebXR, geometry and IK tests passed (six loopback WebSocket/MuJoCo integration cases required socket permission outside the sandbox). The focused checks passed again after the final telemetry addition. `git diff --check` and Python compilation in quest_ros/lerobot passed. These tests use simulated buses; none launches a live hardware controller.

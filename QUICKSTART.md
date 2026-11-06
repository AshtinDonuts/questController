# Quest hardware quickstart

Choose one command below and paste it into a terminal. All paths and camera
settings are already populated; no configuration setup is needed.

Before every launch, check the control socket, hardware supervisor/motor workers
and HTTPS port 8443 as required by [AGENTS.md](AGENTS.md). Resolve an existing
session through powered hold and orderly shutdown before launching another.
The selected arm(s) must be clear to move: these commands enable live hardware
and automatically move to the opening pose. Each includes the measured-feedback
digital twin. Omit `--twin` to hide it; the default is off.
Only an explicit `--twin` enables it; legacy `twin` values in launch JSON are ignored.

With dataset recording

Uses the existing [recording.json](recording.json): left-arm camera
`218622270994` and top camera `218622274350`, both RGB + depth at 640×480,
30 FPS. HDF5 episodes save under `teleop_sessions/dataset`.
Both cameras record regardless of which robot arm is selected.

Recording parameters in `recording.json`:


| Setting               | Current value | Meaning                                                                                                                       |
| --------------------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `episode_time_s`      | `60`          | Automatically save after 60 seconds; `0` disables the timer. Early save/discard still works.                                  |
| `num_episodes`        | `50`          | Target saved episodes in this dataset, including existing ones; `0` means unlimited. Discarded/aborted episodes do not count. |
| `queue_frames`        | `8`           | Acquisition buffer per camera stream.                                                                                         |
| `flush_interval_s`    | `1`           | HDF5 flush interval in seconds.                                                                                               |
| `heartbeat_timeout_s` | `1`           | Recorder heartbeat timeout in seconds.                                                                                        |


At the episode target, further recording/episode-return requests are rejected;
the arms remain held. Request sleep explicitly. Timed saving never starts the
next episode or returns the arms automatically. Restart after changing settings.

### Left arm — recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side left --twin \
  --recording-config recording.json
```



### Right arm — recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side right --twin \
  --recording-config recording.json
```



### Dual arms — recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side both --twin \
  --recording-config recording.json
```

Wait for camera warmup and opening arrival. Release the selected grip(s), point
the **left controller for left-only** or **right controller for right/dual**
forward, then press **Quest left X** or terminal **X** to align. Press terminal **→**
to start; wait for **REC** before moving. Press **→** again to save, **←** to
discard, and **↑** to return to opening for the next episode. Quest **A** also
advances the recording workflow. Saving holds arm joints and opens the grippers fully. Returning restores the
saved ceremony aperture (currently about 50.88% for each arm). Exit recording
with Esc, q or Ctrl+C to sleep and confirm torque-off before closing.

Recording can start from your current pose after alignment, including after
the initial opening times out. Release grips, wait for the arm to settle, focus
the **launch terminal**, then press **→** and wait for **REC** before moving.
Press **X** first if alignment is needed. **↑** is an optional return to opening.
Rerun is a read-only preview: its window does not handle these recording keys.

## See the workspace in Quest 3S

Reload the teleoperation page in **Meta Quest Browser**, then tap
**Enter passthrough**. This requests an AR session so the real room and robot
remain visible behind the controller overlays and recording HUD. The separate
**Enter VR (hides the room)** button remains available.

Release the grips before leaving/reloading a headset session. Reconnecting does
not resume robot motion: follow the status message and press **X** to align
again when permitted. If passthrough entry fails, the page shows the error and
allows retry; it does not silently switch to VR. This is headset passthrough,
not the left-arm/top camera feeds shown in Rerun.

For off-head operation, keep the headset upright at chest/head height with its
front tracking cameras facing the controllers and unobstructed. With USB debugging
authorized, `adb shell am broadcast -a com.oculus.vrpowermanager.prox_close`
applies Meta's [simulated headset-on setting](https://developers.meta.com/vr/documentation/native/android/ts-scriptable-testing/).
Enter passthrough before removing the headset. Keeping the session awake does
not guarantee controller visibility; missing or emulated positions still hold
motion. On 2026-11-06, a motor-free USB diagnostic verified 20 seconds of valid
tracking for both controllers off-head at about 90 packets/s, with maximum
source age 65 ms, after placing the headset upright facing the controllers.
Normal teleoperation uses the PC's network URL; `localhost:8443` was only the
temporary USB-forwarded diagnostic address.

## See progress while collecting

The headset shows a persistent status panel below the center of view, including
while holding a grip. Rerun has a **Recording progress** panel above the camera
views. Both show the current stage, elapsed/remaining episode time, saved count
and the next action. The terminal also prints a status line every second.

Stages distinguish **RECORDING**, **SAVING EPISODE**, **OPENING GRIPPERS**,
**RETURNING TO OPENING**, **ALIGN CONTROLLERS**, **SLEEPING** and faults.
The episode timer freezes when capture ends; saved count increases only after
the file has finalized. Episode attempt numbers include discards; the saved
count is the progress toward the collection target.

## Without dataset recording

These commands save the diagnostic session MP4 and logs, with no HDF5 episodes
or camera acquisition.

### Left arm — no dataset recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side left --twin
```



### Right arm — no dataset recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side right --twin
```



### Dual arms — no dataset recording

```bash
cd /home/King/Desktop/QuestController && \
conda run -n quest_ros --no-capture-output python -m webxr_teleop.hardware.run \
  --config webxr_teleop/hardware/launch.json --side both --twin
```



## Settings already in use

All commands load [launch.json](webxr_teleop/hardware/launch.json).
`--side left`, `right` or `both` overrides its arm selection and activates the
matching entries in [rig.json](webxr_teleop/hardware/rig.json). Only selected
motor buses are opened. Only the selected controller(s) must remain tracked. Left-only aligns from the
left controller; right-only/dual aligns from the right. Quest X/Y/A/B work when
their controller is available; terminal X/S/→/Space provide the same session
controls without requiring the unused controller.

To change sensitivity/speed, edit the launch JSON. To change the task, output
directory or cameras, edit `recording.json`. Restart the session after edits;
repeat endpoint checks first. The recording commands already include
`--recording-config recording.json`; the non-recording commands omit it.
Keep `recording_config` out of the shared launch JSON so this choice stays explicit.

The right mapping is an operator-approved copy of the left. Neither arm's
opening/sleep sequence has independent physical verification recorded. Preserve
per-arm ports, calibrations, electrical current caps and base transforms.

## Operate and shut down

1. Open the printed HTTPS URL in Quest and enter VR with the selected controller(s) tracked.
2. At **READY**, release the selected side grip(s). Point the left controller
  forward for left-only, or the right for right/dual. Press **Quest left X**
   or terminal **X** to align.
3. Hold each **side grip** to move its matching arm; release it to hold position.
  Each **index trigger** controls its gripper independently of the side grip.
4. **Space** stops both arms into powered hold. **Right B** stops only without recording; while recording, **B** discards the current episode and returns to opening. Tracking
  loss also stops motion. **R**, then **left X**, resumes after a nonfault stop;
   **K** explicitly requests recovery to opening when permitted.
5. With both grips released and arms stationary, hold **left Y for one second**
  (or press terminal **S**) to move to sleep. Torque turns off only after verified
   sleep arrival. In recording mode, **Escape**, **q** or **Ctrl+C** requests sleep
   and exits after confirmed torque-off. Without recording, these keys exit in
   powered hold. Stop/fault or failed sleep retains powered hold; a second exit
   request exits immediately in hold.

Optional HDF5 recording, camera configuration and arrow-key episode controls:
[Recording workflow](RECORDING_WORKFLOW.md). Full details:
[Hardware guide](webxr_teleop/hardware/README.md).
While an episode is recording, Quest B or terminal ← discards it, opens the grippers fully, then returns to opening once finalization completes, tracking is fresh and side grips are released. It does not start a new episode. Space, a fault, or shutdown cancels the pending return. Outside recording mode, B retains its powered-stop action.

**Tracking recovery while recording:** tracking loss pauses the current episode. Restore tracking, release side grips and press **X** to align and resume the same episode and timer. No A press is needed. Paused time does not count toward the episode duration. Faults and Space still abort.

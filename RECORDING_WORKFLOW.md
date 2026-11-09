# Dataset recording workflow

For hardware recording, including `lishan_cupstack.json`, use the `quest_ros` Conda environment: `conda activate quest_ros`, or the `conda run -n quest_ros --no-capture-output` prefix in launch examples (no activation required).

The recorder stores lossless HDF5 episodes with native camera and robot timing.
Cameras are selected explicitly in the recording configuration; there are no
default cameras. Accepted episodes can later be exported offline to LeRobot v3.
Native LeRobot v3 recording is also available: use `recording_lerobot_v3.json`
with a separate Python 3.12+ interpreter installed from
`requirements-dataset-lerobot.txt`. Its accepted training dataset is written to
`dataset_dir/lerobot/`; recording controls stay the same. See
[native LeRobot v3 configuration](docs/dataset_recording.md#native-lerobot-v3-recording).

## Enable dataset recording for left, right or dual

Without `--recording-config`, the hardware launcher performs ordinary
teleoperation with diagnostic MP4/logs, **not HDF5 dataset collection**. Recording
controls below apply only when the dataset recorder is enabled. The separate
MuJoCo diagnostic video is not a camera dataset or an accepted episode.

1. Follow [QUICKSTART.md](QUICKSTART.md) to select active arms and complete
  endpoint-ownership checks before launch.
2. Use the populated project-root `recording.json`. Edit its task and dataset
  directory as needed; the left-arm and top cameras are already configured. `cameras` is required: use `{}` explicitly for proprioception-only
   recording. The synthetic `example_mock.json` cannot be used with live motors.
3. Add `--recording-config recording.json` to the selected launch.
  For example, after those checks and with the selected arms clear to move:
   Replace `both` with `left` or `right` for single-arm collection. The preset
   supplies the required `--sequence`; recording does not select the arms.
   Alternatively, add `recording_config` to a custom launch JSON (paths there
   resolve relative to that JSON). Explicit CLI options override launch defaults.
4. Launching prepares the recorder; it does not start an episode. Wait for
  camera warmup, opening arrival and alignment, then start using → or Quest A.

To return to non-recording teleoperation, end the current session orderly and
relaunch without the recording option or launch-JSON key. Repeat endpoint
checks first. There is no `--no-recording` flag or runtime recording-mode switch.
Both modes require tracking only for the selected arm controller(s). Saving holds arm joints and opens grippers fully. Exiting a recording session
requests sleep, waits for measured arrival and torque-off, then closes.

## Episode limits

The populated project-root `recording.json` sets `episode_time_s` to **20** and
`num_episodes` to **2**. Timed episodes save automatically into powered hold;
you can still save earlier or discard. The count includes existing saved episodes
in the dataset directory, excluding discarded/aborted files. Reaching the target
blocks further recording and episode-return requests; request sleep explicitly.
Set either value to `0` for manual duration or unlimited episode count,
respectively. Restart the session after configuration changes.

## Visible progress

A persistent headset panel stays visible while gripping. Rerun's **Recording
progress** panel shows the same stage, episode timer/countdown, saved/target
count, camera health and next action. The terminal prints progress every second.
The timer stops at the end of capture while saving and gripper opening proceed.
Saved count changes after file finalization; discarded attempts do not count.

Focus the launch terminal for recording keys; Rerun is a read-only preview.
After alignment, release grips, wait for the arm to settle, and press **→**
to record from the current pose. This also works after initial opening times
out or after repositioning by teleoperation. **↑** optionally returns to opening;
it is not required to begin capture. Fault, tracking and camera checks still apply.
The twin is opt-in with `--twin`; omitting it uses no twin window.

## Episode-end release

Saving ends capture, holds the arm joints, and commands full gripper aperture.
Completion requires the gripper within 1% of full opening and sustained
stationary feedback from both arm and gripper. Arm joints are held, so release
does not apply the 0.5° arm trajectory-arrival check used by sleep/home.
Motor hold/drift protections remain active. Once **EPISODE SAVED** appears,
press **↑** to request return to opening; return is not automatic.

## After controller tracking loss

Tracking loss pauses the current episode and freezes its timer. Restore tracking,
release the selected side grips, then press Quest X (or terminal X). Successful
alignment resumes the same episode automatically; do not press A to resume.
The timer continues from its paused value, excluding the interruption. Paused
camera/robot/controller samples are omitted; native timestamps and pause/resume
events preserve the gap. Export aligns each active segment separately.
Space, hardware/recorder faults and shutdown still abort the episode.

## Terminal keymap


| Key       | Action                                                                         |
| --------- | ------------------------------------------------------------------------------ |
| **→**     | Start recording when ready; save while recording.                              |
| **←**     | Discard the active episode.                                                    |
| **↑**     | Prepare opening for the first or next episode.                                 |
| **Esc**   | Sleep, confirm torque-off, then exit the recording session.                    |
| **Space** | Immediate powered stop.                                                        |
| **X**     | Align from the left controller for left-only, right controller for right/dual. |
| **S**     | Request sleep.                                                                 |


Quest controls remain unchanged: **A** starts/saves and requests return after
finalization; **B** discards and returns to opening; **Space** stops immediately; **X** aligns with selected grips released;
holding **Y** for one second requests sleep. Side grips move the matching arms,
and triggers control the grippers.

## Arm behavior at each phase


| Phase                      | Arm behavior                                                                                                                                         |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Camera warmup**          | No opening motion yet. Configured cameras initialize before motor workers start.                                                                     |
| **Opening**                | Active arms follow the supervised opening trajectory and move grippers to the saved ceremony aperture.                                               |
| **Ready / alignment**      | Arms hold position. Release selected grips and press Quest X or terminal X to align.                                                                 |
| **Recording**              | Quest grips move the matching arms; releasing a grip holds that arm. Triggers control grippers independently.                                        |
| **Saving / discarded**     | Capture ends; arm joints hold the end pose while grippers open fully. File processing continues separately.                                          |
| **Between episodes**       | Arms remain held until ↑ requests return. There is no automatic return or recording restart.                                                         |
| **Returning**              | Arms move to opening and grippers move to the saved ceremony aperture. Wait for measured arrival and settling, align with X, then press → to record. |
| **Stop / recording fault** | Motion stops into powered hold; the active episode is aborted.                                                                                       |
| **Sleep**                  | Arms follow the configured sleep trajectory. Torque switches off only after confirmed completion.                                                    |


**Saving opens the grippers while retaining arm-joint hold. Exiting recording
(Esc, q, Ctrl+C or SIGTERM) requests sleep and waits for confirmed torque-off.
A stop/fault, stale feedback or interrupted sleep leaves powered hold instead;
a second exit request ends immediately in hold.**

## Collection sequence

1. Before launch, follow the endpoint-ownership checks in [AGENTS.md](AGENTS.md).
  Live launches include the measured-feedback digital twin with `--twin`.
2. Wait for camera warmup, opening arrival and settling. Release selected grips and
  press Quest X or terminal X to align.
3. Press → and wait for **REC** before moving.
4. Teleoperate the demonstration. Rerun shows cameras and joint plots; Quest
  shows recording status, task and timer.
5. Press → to save, or ← to discard. The arm joints hold the end pose while grippers open fully and the file
  finalizes. Wait for gripper settling before requesting return.
6. Press ↑ to return to opening. Reset the objects, wait for settling, align
  with X and press → to begin another episode.
7. Press Esc, q or Ctrl+C to sleep and exit the recording session. S or holding
  Quest Y requests sleep without closing the session.



## Replay saved episodes

From the project root, open the latest saved episode:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.replay
```

The dataset location comes from `recording.json`. To choose an episode, append
its HDF5 path; to browse available episodes, add `--list`:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.replay --list
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.replay teleop_sessions/dataset/accepted/episode_000002.hdf5
```

In Rerun, select the **episode** timeline, press Play, or drag the playhead to
scrub. RGB/depth views and measured/commanded joint plots retain their original
timing. This is camera and telemetry replay, not a 3D robot animation.

Append `--save /path/to/new-recording.rrd` to export without opening a window.
Existing output files are refused. Replay reads saved files only and does not
connect to the robot or cause motion; no hardware launch is needed.

## Recorded data and export

The native recording format remains the source of truth. For the installed
LeRobot ACT training fork, see [ACT training bridge](docs/act_training_bridge.md)
forcd /home/King/Desktop/QuestController && \

conda run -n quest_ros --no-capture-output python -m webxr_[teleop.hardware.run](http://teleop.hardware.run) \

  --config webxr_teleop/hardware/launch.json --side both --twin \

  --recording-config lishan_headphones_real.json

   offline preparation, a loss/backward smoke check, and training commands.

Each episode contains camera frames, timestamps, measured joints, confirmed
motor targets, controller input and rig/calibration metadata. Saving defaults
to an `unknown` outcome; it does not automatically label task success.

Discarded, aborted and unfinished recordings are excluded from normal export.
Mock and live episodes must be exported separately. Offline LeRobot v3 export
normally aligns accepted native streams to 30 Hz while preserving the HDF5
sources.

See [Dataset recording configuration and usage](docs/dataset_recording.md) for
configuration, validation and export details.

While an episode is recording, Quest B or terminal ← discards it, opens the grippers fully, then returns to opening once finalization completes, tracking is fresh and side grips are released. It does not start a new episode. Space, a fault, or shutdown cancels the pending return. Outside recording mode, B retains its powered-stop action.

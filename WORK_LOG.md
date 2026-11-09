# Work log

- 2026-11-07T18:16:12+10:30 | Implementation | Added chronological implementation, fix and debug recording instructions.
  Files: `AGENTS.md`, `WORK_LOG.md`; entries are appended oldest first and limited to 3 physical Markdown lines each.
  Validation: reviewed the instructions and entry format; existing hardware and replay instructions preserved. No runtime changes or tests required.

- 2026-11-07T18:29:40+10:30 | Debug | Recorded the 2026-11-07 left-arm sleep/opening, wrist calibration, voltage-alert, and serial-read incident retrospectively.
  Files: `docs/incidents/2026-11-07-left-wrist-calibration-and-startup-faults.md`, `WORK_LOG.md`; includes physical-test telemetry, operator confirmations, external LeRobot edits, coordinate conversions, backups, and unresolved limits.
  Validation: 35 local evidence links resolve; current calibration SHA-256 and opening/sleep wrist targets match the report. Historical test outcomes are identified; final raw-1704 opening remains physically untested. No hardware operations performed for documentation.

- 2026-11-07T18:31:57+10:30 | Fix | Allow bounded recorder initialization before enforcing runtime heartbeat; reported startup stall's exact cause remains unverified.
  Files: `webxr_teleop/datasets/client.py`, `tests/test_dataset_client_fault.py`; first status gets at least 10 s, then configured 1 s watchdog resumes, including while warming; child exit still fails immediately.
  Validation: 15 recorder/client tests passed outside sandbox (sandbox IPC denied); diff whitespace check passed. Host check found no hardware Python owner or 8443 listener, control socket absent; no hardware launched or cameras opened.

- 2026-11-07T18:41:36+10:30 | Debug | Ran requested left-arm/twin launch; recorder startup heartbeat timed out before motor workers started; isolated cameras using temporary datasets.
  Findings: right_wrist reached idle in 0.8 s; left_wrist failed with frame timeout (1000 ms); top never sent status within 10 s. Concurrent probe stacks stalled in RealSense startup/metadata/frame calls; timeout increase alone does not resolve device initialization.
  Validation: probes exited, no hardware/recorder Python owner or 8443 listener remained, control socket absent. No episode captured or motors started; USB/device cause remains unresolved and needs camera reconnection/retest. Diagnostic scripts reside in `/tmp/quest_recorder_probe.py` and `/tmp/quest_camera_probe.py`.

- 2026-11-07T18:46:03+10:30 | Fix | Operator confirmed top camera was disconnected; added explicit missing-camera errors before stream startup and camera identity/stage context for acquisition, heartbeat and warmup errors.
  Files: `webxr_teleop/datasets/{cameras,recorder,client}.py`, `webxr_teleop/hardware/run.py`, `tests/test_dataset_camera_errors.py`, `tests/test_dataset_client_fault.py`, `docs/dataset_recording.md`; RealSense discovery validates configured serials and propagates failures to supervisor before camera opening.
  Validation: 24 tests passed using mocked/synthetic cameras; one unrelated export regression requires LeRobot 0.6.1 unavailable in recording environment. Whitespace checks passed; no hardware launch or physical camera access performed.

- 2026-11-09T14:55:46+10:30 | Implementation | Extended HDF5-to-LeRobot v3 conversion to dataset/accepted/single-episode inputs; added opt-in native v3 recording with save/discard/pause/resume and provenance.
  Files: `datasets/{config,export_lerobot,hdf5,recorder,lerobot_recording}.py`, `recording_lerobot_v3.json`, `requirements-dataset-lerobot.txt`, recording docs/tests; native SQLite staging publishes accepted data through the official writer to `dataset_dir/lerobot/`, quarantines interrupted attempts/commits, and preserves HDF5 defaults.
  Validation: official LeRobot 0.6.1 loader verified v3.0 schema, state/action parity with converted HDF5, RGB/depth readback and append; no physical hardware or camera connections performed.

- 2026-11-09T14:55:46+10:30 | Fix | Kept healthy long-running saves under heartbeat supervision and allowed 120 s shutdown grace for video finalization; resolved native-recorder NumPy/Rerun dependency conflict.
  Files: `webxr_teleop/datasets/{client,viewer}.py`, native requirements/tests; Rerun 0.22.1 supports NumPy 2 with compatible blueprint argument selection, while existing Rerun 0.16 remains supported; isolated `/tmp/quest-lerobot-v3` environment installed without upgrading motor/HDF5 environments.
  Validation: 72 offline regression tests passed (one unrelated supervisor test deselected), plus two-camera 640x480 RGB/depth 30 FPS synthetic recorder test passed; pip check, old/new preview serialization/blueprints, CLI help and whitespace checks passed. Socket tests required sandbox escalation; physical USB throughput and crash recovery are unverified.

2026-11-09T14:58:29+10:30 Debug — Verified existing offline HDF5-to-LeRobot v3.0 converter and CLI.
Files: webxr_teleop/datasets/export_lerobot.py, requirements-dataset-export.txt; converter validates v3.0 metadata, frame counts and episode boundary readback.
Validation: CLI --help passed in lerobot environment; integration tests could not run because pytest is absent there. No datasets converted or hardware accessed.

2026-11-09T15:00:00+10:30 Debug — Confirmed existing conversion environment at /tmp/quest-lerobot-v3.
Verified Python 3.12.3, LeRobot 0.6.1, torch 2.11.0+cpu, torchvision 0.26.0+cpu, h5py 3.16.0; converter imports passed.
Validation: both exporter integration tests passed with cache access after sandbox cache-write failure; environment is temporary under /tmp. Existing lerobot conda environment has incompatible versions.

- 2026-11-09T15:00:02+10:30 | Implementation | Created operator-requested Lishan cup-stacking configuration with native LeRobot v3.0 output.
  Files: `lishan_cupstack.json`; copied three 640x480 RGB cameras at 15 FPS and 60 s/85-episode limits from `lishan_headphones_real.json`; task is Stack the cups, repo_id local/lishan-cupstack, output FPS 15, dataset_dir ./lishan_data/cupstack.
  Validation: production load_config accepted the JSON; format, FPS and three-camera settings verified offline. No hardware launched or cameras opened.

- 2026-11-09T15:29:14+10:30 | Implementation | Documented the quest_ros Conda environment for hardware launches and cup-stacking recording.
  Files: AGENTS.md, QUICKSTART.md, RECORDING_WORKFLOW.md, webxr_teleop/hardware/README.md; added activation and conda-run alternatives.
  Validation: reviewed documentation diff and checked whitespace; documentation only, no hardware launched.

- 2026-11-09T15:32:36+10:30 | Debug | Ran requested live dual-arm cupstack launch; initial recorder failed because default lerobot environment lacks required LeRobot 0.6.1.
  Files: no source/config edits; retried with --recording-python /tmp/quest-lerobot-v3/bin/python after verifying Python 3.12.3, LeRobot 0.6.1 and RealSense import; endpoint/process checks passed before both launches.
  Validation: recorder warmup passed, twin/HTTPS started and headset connected; session remains running in opening-timeout powered hold (last opening error 2.11 degrees), 0/85 saved; temporary interpreter override required on future launches.

- 2026-11-09T15:38:57+10:30 | Fix | Diagnosed standalone sleep startup drop: generic ManipulatorRobot.connect disables all selected followers before interpolation; aloha.yaml selects both arms.
  Files: external LeRobot sleep.py (backed up), fixes/sleep_torque_preserving_20261109.patch, tests/test_standalone_sleep_connection.py, docs/incidents/2026-11-09-standalone-sleep-startup-torque-drop.md; replaced setup with locked torque-preserving bus attachment and read-only preflight.
  Validation: 7 fake-bus tests passed against installed script, including failure cleanup and post-trajectory torque-off; no motor commands sent. Quest supervisor still present with serial faults; incident torque readbacks and physical validation unavailable.

- 2026-11-09T15:42:03+10:30 | Fix | Resolved control-endpoint conflict after the existing cupstack supervisor exited during inspection.
  Findings: PID 2956608 held the control lock/socket and HTTPS 8443; after exit, no hardware supervisor/workers remained. Removed owned stale socket only after ECONNREFUSED, exclusive lock acquisition and unchanged device/inode checks.
  Validation: control socket and HTTPS 8443 bind probes passed; quest_ros Python and temporary recording interpreter (LeRobot 0.6.1) verified. No hardware launched or motor commands sent; physical motor health unverified.

- 2026-11-09T15:55:10+10:30 | Implementation | Randomly exported four accepted headphones_real HDF5 episodes (31, 41, 44, 98) with the existing offline LeRobot exporter.
  Files: lishan_data/headphones_real_lerobot_sample4_20261109_155254/ (v3.0, 30 FPS), matching sample staging directory and selection manifests; source recordings and exporter unchanged.
  Validation: LeRobot 0.6.1 export exited successfully; 4 episodes / 1810 frames, all episode boundary samples decoded by exporter, metadata counts verified; no hardware access or upload.

- 2026-11-09T16:00:02+10:30 | Implementation | Started full offline LeRobot v3 conversion of accepted headphones_real and bowls_real recordings.
  Files: lishan_data/lerobot_batch_20261109_155940/ manifest, symlink staging, streaming-encoding runner and per-group logs; split headphones into 80 left-arm and 2 dual-arm episodes due to incompatible mappings, bowls has 20 dual-arm episodes.
  Validation: LeRobot 0.6.1 and all 102 default-tolerance alignments verified (65024 planned frames); 84 GB free before start. Three export processes running; final readback pending; no hardware access or upload.

- 2026-11-09T16:08:04+10:30 | Fix | Investigated Quest motion-held reports: latest save rejected a 142.3 ms left-wrist alignment gap; other logs show tracking loss, target expiry, camera staleness and 2.72-degree sleep settling failure.
  Files: datasets/errors.py, lerobot_recording.py, recorder.py, hardware/recording_status.py, tests/test_dataset_lerobot_v3.py, tests/test_recording_status.py, docs/dataset_recording.md; preserve pre-commit rejected attempts without killing recorder or accepting invalid samples.
  Validation: 4 targeted rejection/commit/status tests passed including real offline recorder rejection then successful save; further recorder regressions pending. No hardware commands or threshold changes; load ~29/32 CPUs with background exports is possible timing contributor, not proven cause.

- 2026-11-09T16:08:10+10:30 | Debug | Completed save-rejection regression validation.
  Files: recording status, client fault isolation and recorder lifecycle tests; 31 additional tests passed, 35 total targeted/regression passes; git diff --check passed.
  Validation: all testing offline; live tracking/camera timing interruptions and sleep settling failure remain unresolved; no relaunch performed.

- 2026-11-09T16:11:12+10:30 | Fix | Reverted the standalone sleep torque-preservation fix at the operator request.
  Files: external LeRobot sleep.py restored byte-for-byte from sleep.py.before_torque_preserving_20261109; removed fix-specific patch, regression test and incident report; separate recording-save changes retained.
  Validation: original SHA256 28863796d8f291bbf4106c994360ef95a9615a954a08d22ccc1d1148602138f0 and syntax verified; restores generic connect startup torque-off behavior. No hardware commands or launches performed.

- 2026-11-09T16:11:34+10:30 | Implementation | Completed batch conversion and operator-requested segregation of nonuniform headphones episodes.
  Files: lishan_data/lerobot_batch_20261109_155940/: headphones_real_left (80 episodes/40229 frames), bowls_real_dual (20/22913); excluded_nonuniform/headphones_real_dual contains episodes 0 and 3 as original-HDF5 symlinks plus separate LeRobot output (2/1882), with curation and summary manifests.
  Validation: all three offline exports exited 0 with LeRobot v3.0/30 FPS and decoded boundary samples; independently verified metadata counts and exact source coverage (102 episodes/65024 frames). Original HDF5 unchanged; no uploads or hardware access; excluded data awaits operator choice.

- 2026-11-09T16:13:37+10:30 | Fix | Persisted native LeRobot recorder interpreter in the supplied launch preset so the original cupstack command avoids the default Python 3.10 recorder.
  Files: webxr_teleop/hardware/launch.json, docs/dataset_recording.md; recording_python=/tmp/quest-lerobot-v3/bin/python, verified Python 3.12.3/LeRobot 0.6.1/RealSense import; motor-worker interpreter unchanged.
  Validation: 21 offline launch-config tests and whitespace check passed; host checks found no hardware processes, control socket absent and HTTPS 8443 available. No hardware launched; recording environment remains temporary.

- 2026-11-09T16:16:27+10:30 | Debug | Traced Quest X→A powered hold in hardware_20261109_161406_202600; all three record starts were followed within 100–121 ms by motor target expiry.
  Files: webxr_teleop/hardware/run.py, worker.py and session hardware.jsonl inspected; recorder starting forces planner hold while workers stay armed, leaving the previous teleop target to expire; A itself successfully requests capture.
  Validation: offline event/tick inspection confirms fresh incoming controller samples and stale last dispatched target at first failure; no runtime changes or hardware launches. Transition fix and regression validation remain outstanding; exit also logged sleep settling failure (2.72 degrees).

- 2026-11-09T16:21:07+10:30 | Debug | Rechecked completed batch after operator reported seeing only a few output entries.
  Files: lerobot_batch_20261109_155940 main datasets; independently read Parquet metadata: headphones has 40229 data rows and 80 episode rows, bowls has 22913 data rows and 20 episode rows; success records present.
  Validation: counts match completed exports; few files are expected from LeRobot v3 packed Parquet/video shards, not one file per episode. No dataset changes made.

- 2026-11-09T16:22:10+10:30 | Fix | Prevented X→A recording startup from starving armed workers of stationary targets: apply the recording mode gate before planner-result admission, matching job submission.
  Files: webxr_teleop/hardware/run.py, tests/test_dataset_supervisor.py; added Quest A regression with 400 ms recorder initialization delay and enlarged synthetic-camera queue for that test only. Watchdog deadlines and motion safety checks unchanged.
  Validation: delayed-start and both tracking pause/resume regressions passed (3 tests), git diff --check passed. Initial runs hit test dependency/queue issues, corrected in harness/runtime selection; no live hardware launched or physical validation performed.

- 2026-11-09T16:23:13+10:30 | Debug | Inspected the operator-specified bowls left-wrist video directory and episode-to-video mapping.
  Findings: chunk-000 contains exactly file-000.mp4 (16079 frames, 535.9667 s) and file-001.mp4 (6834 frames, 227.8 s); episode metadata maps indices 0–13 to the first and 14–19 to the second via timestamp ranges.
  Validation: MP4 container counts sum to 22913 frames and match all 20 episode records; two video files are packed storage, not two episodes. No dataset changes.

- 2026-11-09T16:25:09+10:30 | Fix | Diagnosed post-save fault in hardware_20261109_162248_090681: return completed, then 158 ms controller age caused tracking hold followed by fatal planner input-expiry exception.
  Files: webxr_teleop/hardware/planner.py, tests/test_hardware_target_handoff.py; classified input expiry explicitly and report epoch-scoped expired results without terminating the planner; stale commands remain rejected, unrelated planner faults unchanged.
  Validation: 22 target-handoff tests passed, including recovery after both job/controller expiry; git diff --check passed. No hardware launched; underlying tracking latency remains unresolved and still causes a safety hold requiring explicit recovery.

- 2026-11-09T16:26:41+10:30 | Implementation | Repacked batch outputs as LeRobot Dataset v3.0 with exactly one losslessly copied MP4 per episode per camera, per operator request.
  Files: lerobot_batch_20261109_155940/repack_per_episode.py and log, all three outputs video/episode metadata, layout and summary/status manifests; original packed datasets retained in packed_video_backup/, incompatible episodes remain excluded.
  Validation: 306 MP4 frame counts/durations verified; LeRobot 0.6.1 loaded all 102 episodes, every boundary image/action/state exactly matched originals, data Parquet hashes unchanged. Main cameras now have 80 headphone or 20 bowls videos each; no re-encoding, source HDF5 unchanged.

- 2026-11-09T16:29:55+10:30 | Fix | Diagnosed post-save Recorder transport overflow in hardware_20261109_162646_244742; preview telemetry continued after capture ended and socket backpressure incorrectly latched a hardware-session fault.
  Files: webxr_teleop/datasets/client.py, tests/test_dataset_client_fault.py; inactive preview delivery now best-effort, successful sends alone advance sequence/barrier counts; capture and pause/resume delivery remain strict. Dataset metadata reports 5 episodes/2229 frames.
  Validation: 28 client/status tests and 2 mock save/return/exit integration tests passed; git diff --check passed. No live hardware launched; native encoder load was not reproduced in integration, and saved videos were not independently decoded.

- 2026-11-09T16:32:00+10:30 | Debug | Inspected native LeRobot attempt 000013 read-only: rejection comes from this project's 40 ms robot alignment policy, before LeRobot encoding/commit.
  Files: alignment.py, lerobot_recording.py, retained episode_000013.rejected.sqlite; 633 aligned frames, exactly one left observation offset exceeds 40 ms (40.332589 ms); maxima for left action/right observation/right action are 24.866/31.015/22.414 ms.
  Validation: timestamp-only offline alignment inspection; no dataset/config/hardware changes. A 50 ms robot tolerance would pass these robot offsets, but acceptance-policy change and any recovery/export of the retained attempt remain unperformed.

- 2026-11-09T16:35:14+10:30 | Fix | Updated cupstack robot alignment tolerance to 0.1 seconds at operator request.
  Files: lishan_cupstack.json; explicit lerobot.robot_tolerance_s=0.1 (100 ms); motor safety deadlines unchanged.
  Validation: project config loader accepted 0.1; no hardware launched or retained episodes re-exported. Running recorder requires restart to load configuration.

- 2026-11-09T16:36:44+10:30 | Fix | Recovered retained cupstack attempt 000013 under operator-selected 100 ms robot alignment tolerance into LeRobot episode index 5.
  Files: outputs/recovery/recover_cupstack_000013.py, cupstack dataset and attempt 000015 provenance; original rejected SQLite preserved, offline exclusive recording lock held. Initial cache-lock sandbox failure preceded dataset writes; failed attempt 000014 artifacts retained.
  Validation: SQLite quick_check and alignment passed; 633 frames encoded, commit verified, dataset now 6 episodes/2862 frames. No hardware access; videos not independently decoded.

- 2026-11-09T17:09:55+10:30 | Debug | Investigated truncated Conda launch failure; latest two session logs contain shutdown only, without underlying exception.
  Checks: host process scan found no hardware supervisor/worker; control socket absent, HTTPS 8443 bind succeeded, fuser reported no owner for either motor port. No processes stopped, endpoints removed, or hardware launched.
  Validation: endpoint conflict not present at inspection; actual startup error remains unknown pending preceding traceback.

- 2026-11-09T17:10:48+10:30 | Debug | Isolated right-wrist RealSense 218622274350 startup I/O failure from dataset export and motor control.
  Checks: no video-device owner reported by fuser; both D405s enumerate USB 3.2, top D455 enumerates USB 2.1. Camera-only right-wrist 640x480 RGB8/15 FPS start succeeded but waiting for frames timed out after 3000 ms; pipeline stopped in finally.
  Validation: camera failure reproduced independently of recorder/LeRobot; no motor access or configuration changes. Right-wrist USB/camera recovery requires further action; exact cable/power/device cause unconfirmed.

- 2026-11-09T17:16:14+10:30 | Fix | Restored right-wrist RealSense streaming with a targeted hardware_reset of serial 218622274350 after confirming video devices had no reported owners.
  Checks: standalone configured RGB8 640x480/15 FPS test delivered 45 frames; subsequent eight-second concurrent configured-camera test delivered left=110, right=109, top=109 frames with no errors. All pipelines closed afterward.
  Validation: camera-only host tests passed; no motor access, dataset writes, or config changes. Root cause of camera stall and long-session reliability remain unverified; hardware not launched.

- 2026-11-09T17:19:11+10:30 | Fix | Investigated X→A report in hardware_20261109_171637_995310: tracking hold at 157 ms precedes A; A rejected for stale cameras, then right-wrist failure latched recorder hold after next X.
  Files: webxr_teleop/datasets/cameras.py, tests/test_dataset_camera_errors.py; preserve original acquisition exception if RealSense cleanup also fails; normal-stop failures remain visible.
  Validation: 4 targeted camera tests passed, git diff --check passed; no hardware accessed. Persistent camera stall and controller latency remain unresolved; prior short camera tests did not establish session stability.

- 2026-11-09T17:28:21+10:30 | Debug | Reviewed thermal hypothesis for persistent right-wrist disconnect after operator reports cable/port changes did not resolve it.
  Evidence: official D405 product specification lists 0–35 C ambient and 0–55 C case; no camera temperature measurement or thermal fault established.
  Validation: documentation review only; suggested cooled camera-only comparison and swapping cameras between known USB paths to separate device versus path failure. No hardware/configuration changes.

- 2026-11-09T17:35:27+10:30 | Fix | Switched lishan_cupstack_hdf5.json to HDF5 recording at operator request.
  Files: lishan_cupstack_hdf5.json; format=hdf5, removed incompatible lerobot options, set dataset_dir=./lishan_data/cupstack_hdf5 to avoid existing LeRobot dataset conflict.
  Validation: project config loader passed; destination has no LeRobot manifest. No hardware launched or existing recordings modified.

- 2026-11-09T17:43:37+10:30 | Debug | Verified current right-wrist 218622274350 link is USB 2.1/480 Mbps on path 1-3.1.1.2, through three hubs; earlier check reported USB 3.2.
  Checks: left-wrist 218622270994 remains USB 3.2/5000 Mbps; top 231622302829 USB 2.1 is expected per operator. Host SDK enumeration and lsusb topology agree; no camera streams/reset or motor access performed.
  Validation: right-wrist speed downgrade established; exact cause (cable, connector, hub path or camera) remains unconfirmed. Top-camera USB mode is not a requested change.

- 2026-11-09T17:45:58+10:30 | Debug | Traced startup heartbeat expiry with all cameras marked checking connection; this status is published before synchronous SDK device discovery.
  Files: datasets/recorder.py, datasets/client.py, lishan_cupstack.json inspected; first warming heartbeat ends 10-second initial grace and starts configured 1-second watchdog before discovery finishes.
  Validation: code-path inspection only; all-camera listing reflects last known initialization status, not proof all devices failed or USB 2.1 caused the timeout. No configuration/hardware changes; blocked-call location not measured in this failed process.

- 2026-11-09T17:47:08+10:30 | Implementation | Prepared operator-requested checkpoint of current source, tests, documentation, launch/recording configs and calibration provenance before startup-watchdog changes.
  Scope: tracked changes plus relevant untracked project files; recording datasets, training data and generated outputs excluded.
  Validation: git diff --check passed; checkpoint preserves current state, not a claim that all existing changes passed full regression. No hardware launched.

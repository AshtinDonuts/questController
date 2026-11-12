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

- 2026-11-09T17:48:49+10:30 | Fix | After checkpoint 305ead7, kept recorder startup grace active through camera warmup instead of ending it on the initial checking-connection heartbeat.
  Files: datasets/client.py, tests/test_dataset_client_fault.py, tests/test_dataset_supervisor.py, docs/dataset_recording.md; absolute startup deadline prevents indefinite warmup, completed startup latches runtime watchdog even if a later status says warming.
  Validation: 30 client/status tests and delayed 1.5-second discovery mock integration passed; startup timeout, runtime 1-second expiry and child-exit behavior covered; git diff --check passed. No hardware launched; camera disconnection remains a separate issue.

2026-11-09T18:05:45+10:30 Implementation — Added offline cupstack MuJoCo action/state replay and synchronized recorded-video comparison.
Files: lishan_data/replay_cupstack_sim.py, lishan_data/SIM_REPLAY.md, lishan_data/sim_replay/episode_029/*; preserves dataset joint radians and recorded base transforms; no motor/supervisor imports.
Validation: episode 29, 540 frames/36 s at 15 Hz; exact joint assignment, no model joint-limit violations; MP4 inspected; browser control failed, playback URL served on loopback; cups/contact dynamics unmodeled.

2026-11-09T18:08:05+10:30 Implementation — Added action-only episode 29 replay video and autoplay/loop replay page at operator request.
Files: lishan_data/sim_replay/episode_029/action.mp4, action.html; extracted the command panel from the validated simulation video without changing trajectory or timing.
Validation: ffprobe confirms 640x480, 540 frames, 36 seconds; browser automation still fails, so direct local video and replay-page links are provided.

2026-11-09T18:12:56+10:30 Implementation — Extracted cupstack video first frames and generated per-cup spatial heatmaps using operator-specified red/left, yellow/middle, green/right labels.
Files: lishan_data/cupstack_first_frames/ (108 first frames, manifest, annotated montage, center CSV, reproducible analysis, combined and individual heatmaps, README); frozen files 000–035, top view only for spatial counts.
Validation: 108/108 frames decoded; 36 top views each have 3 detections; full annotated montage visually reviewed; each KDE sums to 1 with common scale; pixel cup-opening centers only, no world calibration.

2026-11-09T18:22:30.478221+10:30 Debug — Attempted read-only Quest battery query via ADB.
Affected files: WORK_LOG.md; sandbox blocked ADB startup, and elevated retry reported insufficient device permissions (plugdev).
Validation: battery level unavailable; no device settings or hardware motion changed.

2026-11-09T20:36:46+10:30 — Debug: Verified dataset-recording command and hardware endpoint availability for operator handoff.
Files inspected: QUICKSTART.md, RECORDING_WORKFLOW.md, launch.json, recording.json, lishan_cupstack.json, hardware/run.py; no runtime configuration changed.
Validation: host checks found no hardware supervisor/worker, control socket absent, port 8443 without listener; recorder interpreters exist. No hardware launched.

2026-11-09T20:37:46+10:30 — Debug: Rechecked endpoints before repeating dataset-recording launch command.
Validation: host port 8443 free, control socket absent, no matching hardware supervisor/worker; no cleanup or hardware launch performed.

2026-11-10T10:18:06+10:30 Implementation — Added lishan_eggpack recording preset.
Files: lishan_eggpack.json; LeRobot v3, 60-second episodes, ./lishan_data/eggpack, local/lishan-eggpack; inherited cupstack three-camera 15 FPS setup and 85-episode limit.
Validation: project load_config accepted the preset; format, duration and repo ID assertions passed. No hardware launched.

2026-11-10T13:16:08+10:30 Debug — Reviewed latest run hardware_20261110_130440_802991; right worker Present_Position group read failed at 547.921 s with comm=-3002 (SDK COMM_RX_CORRUPT).
Evidence: hardware.jsonl worker_fault at line 54937; IDs 1–9 are the requested group, not proof each motor failed; supervisor latched FAULT, left entered HOLD, right physical hold unconfirmed.
Validation: read-only log/source/installed-SDK inspection; restart and sleep rejected, shutdown recorded ~33 s later. Physical cause unresolved; no hardware accessed or runtime changes.

2026-11-10T13:20:13+10:30 Debug — Assessed logging gaps for latest right-arm communication fault using bus/worker and installed Dynamixel SDK source.
Findings: group read discards per-ID failure progress; COMM_RX_CORRUPT covers CRC mismatch and partial-packet timeout; exception hold attempt outcome is discarded.
Validation: source inspection only; proposed bounded packet history, transaction/hold diagnostics, USB event correlation and sampled motor health; no runtime changes or hardware access.

2026-11-10T13:29:01+10:30 Implementation — Added low-overhead hardware fault diagnostics: bounded serial history, failed ID/packet classification, transaction timing, explicit emergency-hold outcomes, startup USB/version/clock metadata and fault-only background kernel capture.
Files: hardware/diagnostics.py, bus.py, worker.py, run.py; tests/test_hardware_diagnostics.py; docs/hardware_fault_diagnostics.md. No extra motor polling/retries or control-loop disk writes; history capped at 64×128-byte chunks.
Validation: 142 tests passed, 1 SDK test skipped in quest_ros; installed LeRobot SDK valid/CRC/partial-packet injections passed separately. Synthetic median +31.41 us/20 ms cycle (0.157%); compile/diff checks passed; sandbox socket tests required elevated offline rerun. No hardware accessed; live timing unverified.

2026-11-10T13:33:52+10:30 Debug — Investigated latest unexpected hold in hardware_20261110_133029_376061: right Quest controller missing at 13:32:49.927 (+138.500 s), line 13906.
Evidence: packet 1832024 included both hands; 1832025 retained fresh left but right=None; right recovered after 1.100 s, reset/resume followed ~5.16 s after hold; both motor workers acknowledged HOLD with no worker_fault observed.
Validation: read-only logs/frontend/server inspection; null/invalid/emulated right pose cause is not distinguished in existing telemetry. Later latest holds were normal episode finalization/gripper opening (lines 18440/18504); no hardware accessed or runtime changes.

2026-11-10T13:36:24+10:30 Debug — Attempted read-only Quest controller battery query via ADB.
Evidence: sandbox blocked ADB daemon startup; elevated adb devices -l succeeded but returned no attached devices.
Validation: left/right controller battery levels unavailable; no Quest settings, motor access or runtime configuration changes.

2026-11-10T13:37:23+10:30 Debug — Retried controller battery inspection after operator connected Quest over USB.
Evidence: elevated adb devices -l detects headset but reports no permissions because user King is not in plugdev.
Validation: battery levels remain unavailable; no device settings, host permissions or hardware runtime changed.

2026-11-10T13:37:48+10:30 Debug — Attempted operator-requested addition of King to plugdev with sudo -n usermod -aG plugdev King.
Evidence: sudo refused because an administrator password is required; no group membership was changed.
Validation: USB permission issue remains unresolved; operator must run the command in an interactive terminal and refresh login.

2026-11-10T13:38:33+10:30 Debug — Confirmed King now belongs to plugdev; retried Quest battery access using a separate ADB server on port 5038 under sg plugdev.
Evidence: new-group ADB still reports USB permission denied and suggests missing udev rules; existing port-5037 server was not restarted.
Validation: group update verified, device access and battery levels remain unavailable; no USB rules or device settings changed.

2026-11-10T15:37:55+10:30 Implementation — Created lishan_bowls.json for LeRobot v3 recording, 15 FPS and 65 episodes.
Files: lishan_bowls.json; inherited existing bowls task, three cameras and 60-second episode limit; local/lishan-bowls repo ID. Existing bowls directory contains legacy recording folders, so selected separate ./lishan_data/bowls_lerobot_v3 destination.
Validation: project loader and format/FPS/count assertions passed; destination absent; original bowls preset/data untouched. No hardware launched.

2026-11-10T16:49:47+10:30 Debug — Investigated latest fault in hardware_20261110_162449_240137 at 16:48:12.456 (+1401.205 s), hardware.jsonl line 140504: planner IK/DAQP failed to find a solution.
Evidence: no worker_fault events; both motor feedback streams and controllers fresh; right grip active, left frozen; last successful right target plane_limited=true, IK position error 3.96 mm and rotation error 0.0232 rad. Same planner error also occurred at 16:42:41.
Validation: read-only log/source inspection; restart accepted then returned to starting pose and recording resumed. Workspace constraint involvement is a hypothesis; failing solver inputs/active constraints are not recorded. No hardware access or runtime changes.

2026-11-10T18:51:30+10:30 Debug — Compared first top-camera frames of latest ten saved bowls LeRobot episodes (zero-based 37–46), generating unregistered overlays and static-background displacement analysis.
Artifacts: outputs/top_alignment_20261110/{manifest.json,analyze.py,measure.py,flow.py,first_frames.jpg,key_overlays.png,first_last_blink.gif,registration.json,optical_flow.json,report.txt}; strongest framing changes 37→38 and 38→39; small 41→42 change, 42–46 background stable in sampled frames.
Validation: 10/10 RGB frames decoded at 640×480; overlays visually inspected; SIFT/RANSAC corroborated by optical flow restricted to fixed background. First→last background shift ~10 px right/7 px up; absolute calibration and movement time within episodes unverified. Dataset/hardware unchanged.

2026-11-10T18:52:19+10:30 Debug — Exported individual unregistered top-camera overlays for episodes 37→38 and 38→39 at operator request.
Artifacts: outputs/top_alignment_20261110/red_cyan_037_038.png, red_cyan_038_039.png and overlay_038_039.png; earlier grayscale in red, later in cyan.
Validation: both 640×480 source pairs decoded; no image registration or dataset modifications.

2026-11-12T13:47:37+10:30 Debug — Investigated cupstack dataset location.
Files: lishan_cupstack.json, webxr_teleop/datasets/lerobot_recording.py, lishan_data/wamu_real/cupstack_lerobot.
Validation: configured output is cupstack/lerobot; curated dataset contains 85 episodes; relocation provenance unconfirmed.

2026-11-12T13:48:12+10:30 Implementation — Prepared source, presets, documentation and dataset utilities for commit and push.
Files: staged project changes; large datasets, generated outputs/training artifacts and empty upload placeholder excluded.
Validation: diff whitespace check passed; offline quest_ros tests with plugin autoload disabled: 42 passed, 1 skipped, 4 failed due to missing h5py; no hardware launched.

2026-11-12T13:53:02+10:30 Implementation — Prepared an offline ACT training entry point for the curated eggpack v3 dataset.
Files: scripts/train_eggpack_act.sh, docs/eggpack_act_training.md; uses 79 episodes and documents legacy fork incompatibility and dataset review caveat.
Validation: CLI options inspected; attempted CPU trainer stopped before data loading because accelerate is missing; CUDA driver unavailable, training remains unverified.

2026-11-12T14:16:06+10:30 Fix — Corrected interpreter selection in four supervisor dataset acceptance tests.
Files: tests/test_dataset_supervisor.py; HDF5 assertions run in recording Python; v3 export runs in pinned export Python (QUEST_EXPORT_PYTHON override), preserving all assertions and subprocess error details.
Validation: affected tests 4/4 passed; preceding regression run 44 passed, 1 skipped, 2 export-environment failures now fixed; git diff --check passed. Offline mock hardware only.

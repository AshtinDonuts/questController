# Wamu real-data co-training review

Current status: 2026-11-11, after removing eggpack source episode_index=35 and converting all 30-Hz datasets within wamu_real to 15 Hz. Original versions are retained in dataset_backups.

| Dataset | Episodes | Frames | Hz | State/action dimensions | Duration (min) |
|---|---:|---:|---:|---:|---:|
| bowls_lerobot | 50 | 41,479 | 15 | 14 | 46.09 |
| cupstack_lerobot | 85 | 47,256 | 15 | 14 | 52.51 |
| eggpack_lerobot | 79 | 57,234 | 15 | 14 | 63.59 |
| headphones_lerobot | 80 | 20,129 | 15 | 7 | 22.37 |

Total: 294 episodes, 166,098 frames, approximately 184.55 minutes. All datasets declare LeRobot v3.0, quest_viperx, and three 640×480 RGB cameras: left_wrist, right_wrist, top.

## Verified checks

All data and episode Parquet files were read. Episode/frame totals, dense global and episode indices, 15-Hz timestamps, referenced paths, numeric shapes/finiteness, global min/max, and video intervals passed. All 882 video headers passed dimensions/rate checks.

Eggpack: source episode 35 was removed (34 retained), subsequent episode/global/file indices rebuilt, numeric episode statistics recomputed and global statistics reaggregated. All retained non-index data columns match the original exactly. Every retained episode's first/last sample, including three decoded cameras, passed LeRobot v3 comparison. See its curation_report.json for old/new episode mapping.

Headphones: retained source frame indices 0,2,4,... independently within all 80 episodes. Frames changed from 40,229 to 20,129; all three video streams and action/state rows use the same selection. Rebuilt frame/global indices, timestamps, episode lengths/bounds, video metadata/intervals and all statistics. Videos use lossless H264 CRF 0; all 240 videos were fully decoded and every retained RGB frame matched the source exactly. Video timestamps/counts/durations passed. Numeric statistics were recomputed; RGB statistics use LeRobot image sampling/downsampling on the new decoded videos, with global aggregation through LeRobot. All retained action/state rows match source; first/middle/last samples of every episode passed LeRobot v3 checks. Published-path loading also passed. See resampling_report.json.

Odd-length episodes keep their last even-indexed source frame; nominal encoded duration may increase by 1/30 second for these episodes. No playback speed change. Original headphones source_report.json remains in the backup because its 30-Hz per-sample metadata would be stale in the output.

## Remaining co-training decisions

- Headphones has 7 left-arm values; the other tasks have 14 left/right values. Shared-policy adapters must preserve joint order and mask missing right-arm targets if padding. Check units, action semantics and calibration against the other training source.
- Bowls and headphones share the generic instruction “Move the object to the target”. Agree on task-specific instructions and correctly map local task_index=0 when mixing datasets.
- All datasets declare train-only splits. Hold out complete episodes before fitting training statistics and check for source duplicates across splits/sources.
- Original headphones provenance reports radians, calibrated aperture (0 closed / 1 open), host-reception alignment, and unknown outcomes for all 80 episodes. Sampling and structural validation do not establish task success or hardware exposure synchronization.
- Choose explicit task and source sampling weights and a physical action horizon appropriate to 15 Hz. The target policy and other co-training dataset have not been supplied, so training-pipeline compatibility remains unverified.

No hardware access or training was performed. Other tasks' videos were header-checked and sampled in the earlier tail review, not exhaustively decoded. Machine-readable structural results are in wamu_real_audit.json. Historical tail statistics in wamu_tail_audit.json describe the original pre-curation/pre-resampling versions.

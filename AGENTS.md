# Implementation, fix and debug records

- Record every implementation, fix and debugging activity in the project-root `WORK_LOG.md`, including documentation and configuration changes.
- Append entries in chronological order (oldest first); never reorder or rewrite historical entries. Use an ISO 8601 timestamp with timezone and label each entry `Implementation`, `Fix` or `Debug`.
- Limit each entry to at most 3 physical Markdown lines: timestamp/type and summary; affected files and findings or changes; validation outcome and any unresolved issue. Keep entries concise and distinguish verified results from assumptions.
- Update the log as part of the work before handing it back to the user; do not record secrets or sensitive data.

# Standing launch instructions

- Use the `quest_ros` Conda environment for hardware launches, including `lishan_cupstack.json`: run `conda activate quest_ros` first, or use `conda run -n quest_ros --no-capture-output` as in the launch examples.

The user explicitly requests that recurring `OSError: [Errno 98] Address already in use` failures be checked and resolved before each Quest hardware launch or launch-command handoff.

- Check the configured control socket (default `/tmp/quest-aloha-<uid>/control.sock`), existing hardware supervisor/motor-worker processes, and HTTPS listener (default port 8443).
- Remove a leftover control socket only after confirming it is an owned socket with no live listener (connection refused), and no active hardware supervisor/worker owns the session. Check that the endpoint did not change during inspection.
- Do not blindly unlink an active socket, kill motor workers, or start a second hardware controller. Identify an active owner and resolve the conflict through an orderly powered-hold/shutdown workflow.
- Confirm the endpoints are available before giving the launch command. Checking or cleaning stale endpoints does not authorize starting physical motion.
- Live hardware launch examples should include the measured-feedback MuJoCo digital twin with `--twin` explicitly. Per the operator request on 2026-11-03, the runtime default and supplied launch preset are now twin-off; omitting `--twin` must leave the twin disabled, including when legacy/custom launch JSON contains `twin: true` (JSON twin values are ignored).

# Offline saved-episode replay

- From the project root, run `/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.replay` to open the latest saved HDF5 episode from the dataset configured in `recording.json`.
- Append an HDF5 path to select an episode, or a dataset/`accepted` directory to select its newest episode. Add `--list` to list episodes without opening the viewer.
- In Rerun, select the **episode** timeline and use Play or drag the playhead. Replay shows RGB/depth frames and measured/commanded joint plots, not a 3D robot animation.
- Add `--save /path/to/new-recording.rrd` for a headless export. Existing output files are refused.
- Replay is offline and read-only: it does not connect to motors, cameras or the hardware supervisor, and does not initiate physical motion. Hardware-launch endpoint checks are not required for this offline command.
- See `RECORDING_WORKFLOW.md` and `docs/dataset_recording.md` for usage details.

# Robot mounting geometry

- User measurement on 2026-10-27: left/right robot base centers are **0.64 m apart**, shoulder to shoulder on the same horizontal line and height, with both arms facing forward.
- World frame: +X right, +Y forward, +Z up. With the horizontal origin midway between bases, left X=-0.32 m and right X=+0.32 m; both Y=0. Both base orientations map robot-local +X forward to world +Y (quaternion WXYZ approximately [0.70710678, 0, 0, 0.70710678]).
- The existing base Z=0.02 m above the model table is retained; this report did not establish a new absolute base/table height. Mounting geometry does not verify joint mappings, electrical current caps or opening/sleep trajectories.
- Canonical geometry is recorded in `webxr_teleop/hardware/rig.json`, the default mock profile and `scenes/dual_viperx.xml`. Historical diagnostic candidates/session logs retain their original geometry.

# Operator-requested right-arm status copy

- On 2026-10-27 the user explicitly requested copying the left joint mapping and verification/sequence settings to the right in `rig.json`. Right now matches left: mapping `verified=true`, physical `sequence_verified=false`, operator `sequence_authorized=true`.
- Preserve provenance: this is an operator-approved copy, not an independent right-arm physical test. Keep per-arm ports, calibration files/fingerprints, electrical current caps and measured base transforms distinct. No motor connection or physical verification was performed for this configuration edit.

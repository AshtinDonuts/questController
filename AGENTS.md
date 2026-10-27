# Standing launch instructions

The user explicitly requests that recurring `OSError: [Errno 98] Address already in use` failures be checked and resolved before each Quest hardware launch or launch-command handoff.

- Check the configured control socket (default `/tmp/quest-aloha-<uid>/control.sock`), existing hardware supervisor/motor-worker processes, and HTTPS listener (default port 8443).
- Remove a leftover control socket only after confirming it is an owned socket with no live listener (connection refused), and no active hardware supervisor/worker owns the session. Check that the endpoint did not change during inspection.
- Do not blindly unlink an active socket, kill motor workers, or start a second hardware controller. Identify an active owner and resolve the conflict through an orderly powered-hold/shutdown workflow.
- Confirm the endpoints are available before giving the launch command. Checking or cleaning stale endpoints does not authorize starting physical motion.
- Live hardware launches should include the measured-feedback MuJoCo digital twin (default enabled; use `--twin` explicitly in user-facing commands).

# Robot mounting geometry

- User measurement on 2026-10-27: left/right robot base centers are **0.815 m apart**, shoulder to shoulder on the same horizontal line and height, with both arms facing forward.
- World frame: +X right, +Y forward, +Z up. With the horizontal origin midway between bases, left X=-0.4075 m and right X=+0.4075 m; both Y=0. Both base orientations map robot-local +X forward to world +Y (quaternion WXYZ approximately [0.70710678, 0, 0, 0.70710678]).
- The existing base Z=0.02 m above the model table is retained; this report did not establish a new absolute base/table height. Mounting geometry does not verify joint mappings, electrical current caps or opening/sleep trajectories.
- Canonical geometry is recorded in `webxr_teleop/hardware/rig.json`, the default mock profile and `scenes/dual_viperx.xml`. Historical diagnostic candidates/session logs retain their original geometry.

# Operator-requested right-arm status copy

- On 2026-10-27 the user explicitly requested copying the left joint mapping and verification/sequence settings to the right in `rig.json`. Right now matches left: mapping `verified=true`, physical `sequence_verified=false`, operator `sequence_authorized=true`.
- Preserve provenance: this is an operator-approved copy, not an independent right-arm physical test. Keep per-arm ports, calibration files/fingerprints, electrical current caps and measured base transforms distinct. No motor connection or physical verification was performed for this configuration edit.

# Standing launch instructions

The user explicitly requests that recurring `OSError: [Errno 98] Address already in use` failures be checked and resolved before each Quest hardware launch or launch-command handoff.

- Check the configured control socket (default `/tmp/quest-aloha-<uid>/control.sock`), existing hardware supervisor/motor-worker processes, and HTTPS listener (default port 8443).
- Remove a leftover control socket only after confirming it is an owned socket with no live listener (connection refused), and no active hardware supervisor/worker owns the session. Check that the endpoint did not change during inspection.
- Do not blindly unlink an active socket, kill motor workers, or start a second hardware controller. Identify an active owner and resolve the conflict through an orderly powered-hold/shutdown workflow.
- Confirm the endpoints are available before giving the launch command. Checking or cleaning stale endpoints does not authorize starting physical motion.
- Live hardware launches should include the measured-feedback MuJoCo digital twin (default enabled; use `--twin` explicitly in user-facing commands).

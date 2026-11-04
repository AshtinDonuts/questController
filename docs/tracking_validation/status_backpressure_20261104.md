# Controller timeout: status backpressure mitigation

## Evidence

In `hardware_20261104_115615_358913`, the left controller's freshness age
reached 150 ms following a 115 ms packet gap. The prior packet's challenge was
already approximately 43 ms old when received. The supervisor continued ticking
at approximately 10 ms, and new input arrived about 8 ms after the hold.
This establishes an input freshness timeout, not a confirmed optical tracking
loss. Non-recording sessions also contain packet gaps and watchdog holds.
Recording-related UI load is a plausible contributor, not a proven sole cause.

## Design

Previously each supervisor status publication (approximately 100 Hz) scheduled
a new WebSocket send coroutine, and each incoming browser status immediately
updated the DOM and A-Frame text. Three pose text labels also updated every XR
frame. Recording statuses are larger and contain additional HUD content.

Now each connection has one status sender, limited to 10 Hz, reading one
latest-value snapshot. Intermediate statuses are replaced, not queued. The
sender is cancelled on disconnect. Freshness challenges remain independent at
20 Hz; controller receive processing and the 150 ms freshness limit are unchanged.
A blocked WebSocket/network can still delay delivery; this bounds application
backlog rather than promising network isolation.

The browser coalesces statuses and renders at most 10 Hz. It sends controller
packets before HUD work on each XR frame. Pose text is also limited to 10 Hz;
controller models and alignment aids retain frame-rate updates. A 10 Hz timer
keeps the non-immersive page current. Disconnect warnings discard pending status
renders. Haptics retain their immediate send path. Robot stopping and recording
abort decisions remain in the supervisor, independent of display refresh.

## Validation

- Flood 1,000 status publications while blocking the status sender: only one
  status send is in flight; it is followed by the newest snapshot, with no
  intermediate backlog. Freshness challenges and controller receive continue.
- Expired tokens are still rejected after that flood.
- Browser test: 90 XR ticks send 90 sequential controller packets and no more
  than 10 pose-text updates. Status bursts render their latest value only;
  stale pending UI cannot overwrite a disconnect warning.
- Local TLS/WebSocket lifecycle and mock dataset tracking/recovery checks cover
  integration. No physical robot motion is used for validation.

Restart the supervisor and reload the Quest page to use both sides of the fix.
Physical headset timing still needs a before/after capture; unavoidable stalls
remain fail-closed rather than being hidden by a longer watchdog timeout.

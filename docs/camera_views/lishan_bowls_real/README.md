# Bowls camera identification

Captured 2026-11-06; preset: `lishan_bowls_real.json` at the project root.

| Recording name | Model | Serial | View |
| --- | --- | --- | --- |
| left_wrist | D405 | 218622270994 | [Captured view](left_wrist.png); left wrist confirmed by operator |
| right_wrist | D405 | 218622274350 | [Captured view](right_wrist.png); remaining wrist camera |
| top | D455 | 231622302829 | [Captured view](top.png); overhead bowls and both arms |

Both wrist cameras face upward in the captured arm pose. The previous two-camera preset called the second D405 `top`; this new preset names cameras for their current mounting.

All three request RGB and depth at 640×480, 30 FPS. The D455 currently negotiates USB 2.1; a preceding short simultaneous RGB/depth test delivered about 30 FPS RGB and 24.8 FPS depth, so 30 FPS is requested rather than guaranteed. The two D405s negotiated USB 3 links.

The preset inherits the task text, 60-second episode length, three-episode target and disabled Rerun preview from `lishan_bowls.json`, with a separate `lishan_data/bowls_real` dataset directory.

# Near-stationary wrist motion investigation

Source: `teleop_sessions/hardware_20261103_183408_084004/hardware.jsonl`,
left arm, live session. Investigation and validation were offline; no motor
commands were sent.

At 16–17 seconds after startup, desired position spans only approximately
4.5 / 1.4 / 1.1 mm across XYZ, while commanded forearm roll and wrist rotate
span 9.9 and 10.8 degrees. The controller motion gate reports stationary in
only 4 of 100 planner samples during that second. Position alone does not
establish a constant orientation; both recorded target position and orientation
were used in the following probes.

The acceleration-constrained IK used unit task gain, requesting complete pose
error correction each tick. At approximately 100 Hz this requests corrections
much faster than the 40 degrees/s² acceleration constraint can stop. Small
continuing controller changes keep the solver active, permitting repeated
overshoot and correction.

Changed acceleration-constrained IK to a 0.10-second task convergence time,
with gain proportional to the actual solver timestep. Frame and posture tasks
share that gain, preserving their relative costs. Speed, acceleration, joint
and planar constraints, and the stationary-input brake remain in place.
The path without acceleration constraints retains its previous behavior.

Offline comparison using recorded targets and stationary flags from seconds
15–34, starting at the first recorded command (this is a target replay, not a
physical motor simulation):

| Metric | Previous gain | Time-based gain |
| --- | ---: | ---: |
| Median position error | 8.77 mm | 2.85 mm |
| Median orientation error | 2.38° | 1.07° |
| Forearm-roll total travel | 115.77° | 75.89° |
| Wrist-rotate total travel | 126.08° | 112.19° |

A second probe holds the target fixed from the first command after 16 seconds,
including its initial joint velocity, and keeps IK active for six seconds.
Previously the shoulder continued oscillating over 0.639° during the final
three seconds; the changed solver settles to less than 0.01° range on every
joint. The committed fixture and `tests/test_hardware_ik_settling.py` exercise
this at 50, 100 and 200 Hz and verify speed, acceleration and pose error.

These results establish an offline improvement, not physical verification of
all reported circling. The earlier joint-limit braking infeasibility and this
session's sleep settling timeout (4.48° maximum error) are separate unresolved
findings. No safety limits or settling tolerances were relaxed.

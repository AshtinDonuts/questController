# Hardware fault diagnostics

Hardware workers retain a bounded serial flight recorder in memory automatically.
It performs no extra motor reads, retries, USB scans or disk writes in the control
loop. Existing hardware-error, torque and watchdog checks continue unchanged;
voltage, temperature and current polling have not been added.

## Where to look

- `hardware.jsonl` startup: wall-clock/monotonic anchor, Python/package versions
  and source commit (the commit does not describe uncommitted edits).
- Worker preflight: resolved serial port, sysfs path, USB identity and speed when
  available, and worker clock anchor. These are startup snapshots.
- `worker_fault.diagnostics`: failing group transaction `(register, requested IDs,
  start monotonic ns)`, last reply `(expected ID, communication result, motor
  status error, returned data length)`, total attempted group transactions, maximum
  group transaction duration, failed packet and recent serial chunks.
- `worker_fault.hold_attempt`: whether emergency hold was attempted, feedback age,
  skipped unchanged goal, write/readback success or exception. Success does not
  establish physical settling; `hold_confirmed` retains its original meaning.
- `kernel_fault.log`: up to 80 kernel journal entries from the preceding two
  minutes, collected once in a background thread after a nonrecoverable non-mock worker fault.
  Journal access failures appear in the output; missing events do not establish
  that no USB reset occurred. Capture has a two-second subprocess timeout and is
  best effort if the supervisor exits immediately.

`COMM_RX_CORRUPT` (-3002) can mean CRC mismatch or a timeout with partial data.
The fault-only decoder distinguishes these when the SDK retains sufficient
packet bytes; other cases remain `packet_validation_failure` or
`sdk_receive_failure`. Raw packet bytes help inspect ambiguous cases.

The rolling buffer holds at most 64 nonempty TX/RX chunks, each truncated to 128
bytes, retaining original lengths and monotonic timestamps. It covers recent
traffic, not a fixed number of seconds; empty polling reads add no entries. A
failed packet is separately capped at 1024 bytes. No per-packet hex encoding or
CRC analysis is performed on successful transactions. The fault snapshot is
frozen before hold/cleanup traffic can overwrite it.

Existing tick feedback contains cycle read latency and motor health. The new
maximum is for individual group transactions, not whole control cycles.

## Offline validation

`tests/test_hardware_diagnostics.py` checks bounded capture, IPC message size,
unchanged transport results/exceptions, failure classification and SDK parsing
(the SDK case skips in environments without the package).

On 2026-11-10, a synthetic benchmark in the LeRobot Python environment measured
median added cost of 31.41 microseconds per cycle (0.157% of the 20 ms budget),
using ten group transactions with nine replies each, 10,000 cycles per run and
five alternating baseline/instrumented runs. This measures Python bookkeeping,
not live USB timing or scheduling jitter. No hardware was accessed during testing.

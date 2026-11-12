"""Bounded in-memory serial flight recorder; no additional bus transactions."""
from collections import deque
from datetime import datetime
from importlib.metadata import version, PackageNotFoundError
import os
from pathlib import Path
import subprocess
import threading
import time


def clock_anchor():
    return dict(wall_time=datetime.now().astimezone().isoformat(),
                monotonic_ns=time.monotonic_ns())


def session_metadata(port=None):
    result = dict(clock_anchor(), python=os.sys.version, pid=os.getpid())
    for package in ('dynamixel-sdk', 'pyserial', 'lerobot'):
        try:
            result[package] = version(package)
        except PackageNotFoundError:
            result[package] = 'unknown'
    if port:
        resolved = os.path.realpath(port)
        result.update(port=port, resolved_port=resolved,
                      sysfs_device=os.path.realpath('/sys/class/tty/'+Path(resolved).name+'/device'))
        device = Path(result['sysfs_device'])
        for parent in (device, *device.parents):
            if (parent/'idVendor').exists():
                result['usb'] = dict(path=str(parent))
                for name in ('idVendor', 'idProduct', 'serial', 'speed', 'busnum', 'devnum'):
                    try:
                        result['usb'][name] = (parent/name).read_text().strip()
                    except OSError:
                        pass
                break
    else:
        try:
            revision = subprocess.run(['git', 'rev-parse', 'HEAD'],
                                      cwd=Path(__file__).resolve().parents[2],
                                      capture_output=True, text=True, timeout=1, check=False)
            result['source_revision'] = revision.stdout.strip() or 'unknown'
        except (OSError, subprocess.TimeoutExpired):
            result['source_revision'] = 'unknown'
    return result


class SerialDiagnostics:
    """Wrap this connection only. Retain at most 64 chunks of 128 bytes.

    Empty SDK polling reads allocate no records. Hex encoding/CRC analysis happens
    only after failure. Wrappers preserve SDK results and exceptions unchanged.
    """
    def __init__(self, port, packet):
        self.packet = packet
        self.history = deque(maxlen=64)
        self.last_reply = None
        self.failed_packet = None
        self.transactions = 0
        self.max_transaction_ns = 0
        self.transaction = None
        for name, direction in (('readPort', 'rx'), ('writePort', 'tx')):
            original = getattr(port, name)
            def capture(*args, _original=original, _direction=direction):
                result = _original(*args)
                data = result if _direction == 'rx' else args[0]
                if data:
                    self.history.append((time.monotonic_ns(), _direction, len(data), bytes(data[:128])))
                return result
            setattr(port, name, capture)
        original_read = packet.readRx
        def read_reply(port, motor_id, length):
            result = original_read(port, motor_id, length)
            self.last_reply = (motor_id, result[1], result[2], len(result[0]))
            return result
        packet.readRx = read_reply
        original_rx = packet.rxPacket
        def receive_packet(*args, **kwargs):
            result = original_rx(*args, **kwargs)
            if result[1] != 0:
                raw = bytes((result[0] or [])[:1024])
                self.failed_packet = (result[1], raw)
            return result
        packet.rxPacket = receive_packet

    def begin(self, register, ids):
        self.last_reply = None
        self.failed_packet = None
        self.transaction = (register, ids, time.monotonic_ns())

    def finish(self):
        self.transactions += 1
        self.max_transaction_ns = max(self.max_transaction_ns,
                                      time.monotonic_ns()-self.transaction[2])

    def snapshot(self):
        packet = None
        if self.failed_packet:
            code, raw = self.failed_packet
            expected = (int.from_bytes(raw[5:7], 'little')+7) if len(raw) >= 7 else None
            packet = dict(result=code, received_bytes=len(raw), expected_bytes=expected,
                          raw_hex=raw.hex(), category='sdk_receive_failure')
            if code == -3002:
                packet['category'] = ('partial_packet_timeout' if expected is None or len(raw) < expected
                                      else 'packet_validation_failure')
                if expected is not None and len(raw) == expected and len(raw) >= 11:
                    crc = self.packet.updateCRC(0, list(raw), len(raw)-2)
                    packet['crc_matches'] = crc == int.from_bytes(raw[-2:], 'little')
                    if not packet['crc_matches']:
                        packet['category'] = 'crc_mismatch'
        return dict(clock_anchor(), transaction=self.transaction,
                    last_reply=self.last_reply, failed_packet=packet,
                    transactions=self.transactions, max_transaction_ms=self.max_transaction_ns/1e6,
                    history=[dict(t_ns=t, direction=d, length=n, hex=b.hex())
                             for t,d,n,b in self.history])


def capture_kernel_events(output):
    """Fault-only background capture. Permissions failures remain visible."""
    def capture():
        path = Path(output)/'kernel_fault.log'
        try:
            with path.open('x') as stream:
                subprocess.run(['journalctl', '-k', '-b', '--since=-2min', '-n', '80',
                                '--no-pager', '-o', 'short-monotonic'],
                               stdout=stream, stderr=subprocess.STDOUT, timeout=2, check=False)
        except Exception as exc:
            # Never let diagnostics alter the controller's fault handling.
            print('Kernel fault capture unavailable: '+str(exc), flush=True)
    try:
        threading.Thread(target=capture, name='kernel-fault-capture', daemon=True).start()
    except RuntimeError as exc:
        print('Kernel fault capture could not start: '+str(exc), flush=True)

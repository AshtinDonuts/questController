"""Offline diagnostics checks: real SDK parser and fake serial transport."""
import json
from types import SimpleNamespace

import pytest

from webxr_teleop.hardware.diagnostics import SerialDiagnostics
from webxr_teleop.hardware.ipc import pair, send, receive


def connection():
    port = SimpleNamespace(readPort=lambda n: b'', writePort=lambda b: len(b))
    packet = SimpleNamespace(readRx=lambda *a: ([1, 2], 0, 0),
                             rxPacket=lambda *a: ([], -3001),
                             updateCRC=lambda *a: 123)
    return port, packet


def test_bounded_capture_and_empty_polling():
    port, packet = connection()
    diag = SerialDiagnostics(port, packet)
    for _ in range(10000):
        assert port.readPort(20) == b''
    assert not diag.history
    for _ in range(100):
        assert port.writePort(bytes(1024)) == 1024
    report = diag.snapshot()
    assert len(report['history']) == 64
    assert all(len(h['hex']) == 256 and h['length'] == 1024 for h in report['history'])
    # Worst-case normal snapshot fits comfortably in the supervisor's datagram.
    a, b = pair()
    try:
        assert len(json.dumps(report)) < 30000
        assert send(a, dict(type='fault', diagnostics=report))
        assert receive(b)[0]['diagnostics'] == report
    finally:
        a.close(); b.close()


def test_failed_id_and_original_results_preserved():
    port, packet = connection()
    packet.readRx = lambda p, i, n: ([7], -3002, 0)
    diag = SerialDiagnostics(port, packet)
    diag.begin('Present_Position', [1, 2, 3])
    assert packet.readRx(port, 2, 4) == ([7], -3002, 0)
    diag.finish()
    report = diag.snapshot()
    assert report['last_reply'] == (2, -3002, 0, 1)
    assert report['transactions'] == 1
    assert report['max_transaction_ms'] >= 0


@pytest.mark.parametrize('raw,category', [
    (b'\xff\xff', 'partial_packet_timeout'),
    (bytes([255,255,253,0,2,8,0,85,0,1,2]), 'partial_packet_timeout'),
    (bytes([255,255,253,0,2,4,0,85,0,0,0]), 'crc_mismatch'),
])
def test_fault_packet_classification(raw, category):
    port, packet = connection()
    packet.rxPacket = lambda *a: (list(raw), -3002)
    diag = SerialDiagnostics(port, packet)
    assert packet.rxPacket(port, False) == (list(raw), -3002)
    assert diag.snapshot()['failed_packet']['category'] == category


def test_serial_exceptions_are_unchanged():
    port, packet = connection()
    exc = OSError('disconnected')
    def fail(n):
        raise exc
    port.readPort = fail
    diag = SerialDiagnostics(port, packet)
    with pytest.raises(OSError) as caught:
        port.readPort(10)
    assert caught.value is exc
    assert not diag.history


def test_real_sdk_crc_and_partial_packet():
    dxl = pytest.importorskip('dynamixel_sdk')
    packet = dxl.PacketHandler(2.0)
    raw = [255,255,253,0,2,4,0,85,0,0,0]
    data = [bytes(raw)]
    port = SimpleNamespace(readPort=lambda n: data.pop(0) if data else b'',
                           writePort=lambda b: len(b), isPacketTimeout=lambda: True,
                           is_using=False)
    diag = SerialDiagnostics(port, packet)
    packet.readRx(port, 2, 0)
    report = diag.snapshot()
    assert report['last_reply'][0:2] == (2, -3002)
    assert report['failed_packet']['category'] == 'crc_mismatch'
    assert report['failed_packet']['crc_matches'] is False

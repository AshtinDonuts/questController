import socket
import pytest
from webxr_teleop.hardware.endpoint import bind_control


def endpoint(tmp_path):
    p=tmp_path/'private';p.mkdir(mode=0o700)
    return p/'control.sock'


def test_recovers_stale_socket(tmp_path,monkeypatch):
    p=endpoint(tmp_path)
    old=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);old.bind(str(p));old.close()
    monkeypatch.setattr('webxr_teleop.hardware.endpoint.hardware_processes',lambda:[])
    server,lock=bind_control(p)
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:client.connect(str(p))
    finally:server.close();p.unlink();lock.close()


def test_live_listener_not_replaced(tmp_path):
    p=endpoint(tmp_path)
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as server:
        server.bind(str(p));server.listen(4);inode=p.stat().st_ino
        with pytest.raises(RuntimeError,match='live supervisor'):bind_control(p)
        assert p.stat().st_ino==inode


def test_lock_excludes_second_launcher(tmp_path):
    p=endpoint(tmp_path);server,lock=bind_control(p)
    try:
        with pytest.raises(RuntimeError,match='Another supervisor'):bind_control(p)
    finally:server.close();p.unlink();lock.close()


def test_orphan_worker_prevents_cleanup(tmp_path,monkeypatch):
    p=endpoint(tmp_path)
    old=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);old.bind(str(p));old.close()
    monkeypatch.setattr('webxr_teleop.hardware.endpoint.hardware_processes',lambda:[123])
    with pytest.raises(RuntimeError,match='still running'):bind_control(p)
    assert p.exists()


def test_regular_file_not_removed(tmp_path):
    p=endpoint(tmp_path);p.write_text('preserve')
    with pytest.raises(RuntimeError,match='non-socket'):bind_control(p)
    assert p.read_text()=='preserve'


@pytest.mark.parametrize('args,expected',[
    (['/env/bin/python','-m','webxr_teleop.hardware.run','--mode','live'],True),
    (['python3.12','-u','-m','webxr_teleop.hardware.worker','--mode=live'],True),
    (['python','-X','dev','-m','webxr_teleop.hardware.run','--mode','observe'],True),
    (['/bin/conda','run','python','-m','webxr_teleop.hardware.run','--mode','live'],False),
    (['python','/bin/conda','run','python','-m','webxr_teleop.hardware.run'],False),
    (['bash','-c','python -m webxr_teleop.hardware.run'],False),
    (['python','-m','webxr_teleop.hardware.worker','--mode','mock'],False),
    (['python','-m','webxr_teleop.hardware.run','--mode=mock'],False),
    (['python','-m','webxr_teleop.hardware.run','status'],False),
    (['python','-c','print(1)','-m','webxr_teleop.hardware.run'],False),
])
def test_process_detection_distinguishes_wrappers_and_mock(args,expected):
    from webxr_teleop.hardware.endpoint import is_hardware_process
    assert is_hardware_process([s.encode() for s in args]) is expected

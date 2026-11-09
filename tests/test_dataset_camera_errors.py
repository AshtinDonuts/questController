"""Disconnected devices must identify themselves before acquisition starts."""
import os
import sys
from types import SimpleNamespace

import pytest

from webxr_teleop.datasets.cameras import check_realsense_devices
from webxr_teleop.datasets import recorder
from webxr_teleop.hardware.ipc import pair, receive


def fake_sdk(monkeypatch, serials):
    devices = [SimpleNamespace(get_info=lambda key, serial=serial: serial) for serial in serials]
    monkeypatch.setitem(sys.modules, 'pyrealsense2', SimpleNamespace(
        camera_info=SimpleNamespace(serial_number='serial'),
        context=lambda: SimpleNamespace(query_devices=lambda: devices)))


def test_missing_camera_failure_reaches_supervisor_before_any_camera_opens(tmp_path, monkeypatch):
    fake_sdk(monkeypatch, ['wrist-serial'])
    cfg = dict(dataset_dir=str(tmp_path), cameras={
        'left_wrist': dict(backend='realsense', device='wrist-serial'),
        'top': dict(backend='realsense', device='top-serial')})
    monkeypatch.setattr(recorder, 'Camera', lambda *a: pytest.fail('Camera opened before preflight'))
    control, child_control = pair()
    data, child_data = pair()
    try:
        with pytest.raises(RuntimeError, match=r'top \(RealSense serial top-serial\).*USB'):
            recorder.run(cfg, {}, child_control, child_data, os.getppid())
        messages = receive(control)
        assert messages[0]['cameras']['top']['device'] == 'top-serial'
        assert messages[-1]['phase'] == 'failed'
        assert 'top (RealSense serial top-serial)' in messages[-1]['error']
        assert 'left_wrist' not in messages[-1]['error']
    finally:
        for sock in (control, child_control, data, child_data):
            sock.close()


def test_connected_cameras_pass_preflight(monkeypatch):
    fake_sdk(monkeypatch, ['top-serial', 'extra-device'])
    check_realsense_devices({'top': dict(backend='realsense', device='top-serial')})


def test_non_realsense_configuration_needs_no_sdk(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pyrealsense2', None)
    check_realsense_devices({'mock': dict(backend='synthetic')})


@pytest.mark.parametrize('acquisition_error',[True,False])
def test_realsense_cleanup_preserves_acquisition_error(monkeypatch,acquisition_error):
    from webxr_teleop.datasets.cameras import Camera
    def fail_wait(timeout):raise RuntimeError('Original frame timeout')
    def fail_stop():raise RuntimeError('stop() cannot be called before start()')
    pipeline=SimpleNamespace(
        start=lambda settings:SimpleNamespace(get_device=lambda:SimpleNamespace(get_info=lambda key:'serial')),
        wait_for_frames=fail_wait,stop=fail_stop)
    monkeypatch.setitem(sys.modules,'pyrealsense2',SimpleNamespace(
        config=lambda:SimpleNamespace(enable_device=lambda serial:None),pipeline=lambda:pipeline,
        camera_info=SimpleNamespace(serial_number='serial')))
    camera=Camera.__new__(Camera)
    camera.config=dict(device='serial',streams=[])
    camera.stop=SimpleNamespace(is_set=lambda:not acquisition_error)
    expected='Original frame timeout' if acquisition_error else r'stop\(\) cannot'
    with pytest.raises(RuntimeError,match=expected):camera._realsense()

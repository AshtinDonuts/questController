"""Motor workers must not inherit the supervisor's Python ABI paths."""
import os
from webxr_teleop.hardware.run import spawn_worker, DEFAULT_PROFILE


def test_spawn_isolates_python_paths(monkeypatch):
    for name in ('PYTHONPATH','PYTHONHOME','PYTHONUSERBASE'):
        monkeypatch.setenv(name,'/another-python/site-packages')
    captured={}
    def popen(argv, **kwargs):
        captured.update(kwargs)
        return object()
    monkeypatch.setattr('webxr_teleop.hardware.run.subprocess.Popen',popen)
    sock,_=spawn_worker(DEFAULT_PROFILE,'left','mock','python')
    sock.close()
    for name in ('PYTHONPATH','PYTHONHOME','PYTHONUSERBASE'):
        assert name not in captured['env']
        assert name in os.environ
    assert captured['env']['PYTHONNOUSERSITE']=='1'


def test_spawn_passes_session_gripper_speed_to_motor_worker(monkeypatch):
    captured=[]
    monkeypatch.setattr('webxr_teleop.hardware.run.subprocess.Popen',lambda argv,**kwargs:captured.extend(argv))
    sock,_=spawn_worker(DEFAULT_PROFILE,'left','mock','python',gripper_speed_limit=.7)
    sock.close()
    assert captured[captured.index('--gripper-speed')+1]=='0.7'


def test_spawn_passes_gripper_acceleration_to_worker(monkeypatch):
    captured=[]
    monkeypatch.setattr('webxr_teleop.hardware.run.subprocess.Popen',lambda argv,**kwargs:captured.extend(argv))
    sock,_=spawn_worker(DEFAULT_PROFILE,'left','mock','python',gripper_acceleration_limit=3.)
    sock.close()
    assert captured[captured.index('--gripper-acceleration')+1]=='3.0'

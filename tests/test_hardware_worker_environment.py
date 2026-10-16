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

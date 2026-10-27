import argparse
import json
from pathlib import Path
import pytest
from webxr_teleop.hardware.launch_config import configured_argv


def parser():
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path)
    p.add_argument('--mode',choices=['live','mock'],default='mock')
    p.add_argument('--profile',type=Path)
    p.add_argument('--sequence',action='store_true')
    p.add_argument('--twin',action=argparse.BooleanOptionalAction)
    p.add_argument('--translation-scale',type=float)
    return p


def test_config_defaults_paths_and_cli_override(tmp_path):
    file=tmp_path/'launch.json'
    file.write_text(json.dumps(dict(mode='live',profile='rig.json',sequence=True,twin=True,translation_scale=.5)))
    p=parser();args=p.parse_args(configured_argv(p,['--config',str(file),'--mode','mock','--no-twin']))
    assert args.mode=='mock' and args.twin is False and args.sequence
    assert args.profile==tmp_path/'rig.json' and args.translation_scale==.5


@pytest.mark.parametrize('value',[[],{'unknown':1},{'mode':'invalid'},{'sequence':'true'},{'translation_scale':True},{'translation_scale':float('nan')},{'profile':3}])
def test_invalid_config_rejected(tmp_path,value):
    file=tmp_path/'launch.json';file.write_text(json.dumps(value))
    p=parser()
    with pytest.raises(SystemExit):p.parse_args(configured_argv(p,['--config',str(file)]))


def test_no_config_keeps_cli_defaults():
    p=parser();assert p.parse_args(configured_argv(p,[])).mode=='mock'

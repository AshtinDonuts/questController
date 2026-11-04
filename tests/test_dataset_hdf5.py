"""Native source fidelity and deterministic, causal training alignment."""
import json
from pathlib import Path

import h5py
import numpy as np
import pytest

from webxr_teleop.datasets.config import load_config
from webxr_teleop.datasets.hdf5 import Episode, validate
from webxr_teleop.datasets.export_lerobot import alignment, export

ROOT = Path(__file__).resolve().parents[1]


def make_episode(root, sides=('left','right'), cameras=True, identity=0, status='saved'):
    origin = 9_000_000_000_000_000
    cfg = dict(task='Put the cube in the tray', cameras={})
    if cameras:
        cfg['cameras']['front'] = dict(width=64,height=48,fps=30,streams=['rgb','depth'])
    metadata = dict(sides=list(sides), config=cfg, mode='mock',
                    profile=json.loads((ROOT/'webxr_teleop/hardware/example_profile.json').read_text()),
                    cameras={'front':dict(depth_scale_m=.001)},calibrations={})
    episode = Episode(root,identity,origin,metadata)
    for index in range(51):
        timestamp = origin+index*20_000_000
        for side in sides:
            episode.robot(side,dict(t=timestamp/1e9,seq=index,epoch=1,q=[index*.001]*6+[.5],
                health=dict(mock=True),action=dict(t_ns=timestamp+3_000_000,qpos=[index*.002]*6+[.6],
                confirmed=True,confirmed_ns=timestamp+4_000_000,raw_goal=None)))
    if cameras:
        for index in range(31):
            timestamp = origin+round(index*1e9/30)
            for stream in ('rgb','depth'):
                shape = (48,64,3) if stream=='rgb' else (48,64)
                image = np.full(shape,index,dtype=np.uint8 if stream=='rgb' else np.uint16)
                episode.camera('front',stream,dict(t_ns=timestamp,device_ms=index*1000/30,
                    frame_number=index,image=image))
    return episode.finish(status,origin+1_010_000_000)


def test_config_requires_explicit_cameras_and_rejects_synthetic_live(tmp_path):
    path=tmp_path/'record.json'
    path.write_text(json.dumps(dict(dataset_dir='dataset',task='test')))
    with pytest.raises(ValueError,match='cameras must be explicit'):load_config(path,mock=True)
    cfg=json.loads((ROOT/'webxr_teleop/datasets/example_mock.json').read_text())
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError,match='synthetic requires mock'):load_config(path)
    assert load_config(path,mock=True)['cameras']['test_front']['width']==64


@pytest.mark.parametrize('count',[-1,1.5,True,'50'])
def test_config_rejects_invalid_episode_count(tmp_path,count):
    path=tmp_path/'record.json'
    path.write_text(json.dumps(dict(dataset_dir='dataset',task='test',cameras={},num_episodes=count)))
    with pytest.raises(ValueError,match='num_episodes'):load_config(path)


@pytest.mark.parametrize('sides',[('left',),('left','right')])
def test_source_preserves_native_rates_lossless_pixels_and_units(tmp_path,sides):
    path=make_episode(tmp_path,sides)
    summary=validate(path)
    assert summary['counts']['cameras/front/rgb']==31
    assert summary['counts'][f'robot/{sides[0]}/observations']==51
    with h5py.File(path,'r') as file:
        np.testing.assert_array_equal(file['streams/cameras/front/rgb/image'][17],np.full((48,64,3),17,dtype=np.uint8))
        np.testing.assert_array_equal(file['streams/cameras/front/depth/image'][17],np.full((48,64),17,dtype=np.uint16))
        assert file[f'streams/robot/{sides[0]}/observations/qpos'][20,0]==pytest.approx(.02)
        assert file[f'streams/robot/{sides[0]}/actions/qpos'][20,0]==pytest.approx(.04)


def test_alignment_uses_prior_observations_following_actions_and_retains_origin_precision(tmp_path):
    path=make_episode(tmp_path)
    with h5py.File(path,'r') as file:
        grid,indices,offsets=alignment(file)
        assert len(grid)==30
        assert np.all(np.diff(grid)>0)
        for key,delta in offsets.items():
            assert np.all(delta>=0) if key.endswith('actions') else np.all(delta<=0)
        robot='streams/robot/left/observations'
        first=indices[robot][0]
        assert file[robot+'/t_ns'][first]<=grid[0]
        assert grid[0]>=9_000_000_000_000_000


def test_alignment_refuses_robot_gap_instead_of_filling(tmp_path):
    path=make_episode(tmp_path,cameras=False)
    with h5py.File(path,'r+') as file:
        for side in ('left','right'):
            for stream in ('observations','actions'):
                times=file[f'streams/robot/{side}/{stream}/t_ns']
                values=times[:];values[10:]+=200_000_000;times[:]=values
    with h5py.File(path,'r') as file:
        with pytest.raises(ValueError,match='gap exceeds tolerance'):alignment(file)


def test_motor_sequence_gap_is_visible(tmp_path):
    episode=Episode(tmp_path,0,0,dict(sides=['left'],config=dict(cameras={})))
    episode.robot('left',dict(t=1,seq=1,epoch=0,q=[0.]*7))
    with pytest.raises(ValueError,match='sequence gap'):
        episode.robot('left',dict(t=2,seq=3,epoch=0,q=[0.]*7))
    episode.close_partial()
    with pytest.raises(ValueError,match='Unfinished'):validate(episode.path)


def test_discard_and_abort_are_retained_but_not_accepted(tmp_path):
    assert make_episode(tmp_path,identity=0,status='discarded').parent.name=='discarded'
    assert make_episode(tmp_path,identity=1,status='aborted').parent.name=='aborted'
    assert not list((tmp_path/'accepted').glob('*.hdf5'))


def test_unequal_stream_lengths_fail_validation(tmp_path):
    path=make_episode(tmp_path)
    with h5py.File(path,'r+') as file:
        file['streams/robot/left/observations/qpos'].resize(2,axis=0)
    with pytest.raises(ValueError,match='stream lengths'):validate(path)


def test_end_boundary_excludes_later_hold_samples(tmp_path):
    path=make_episode(tmp_path,cameras=False)
    with h5py.File(path,'r') as file:
        assert file['streams/robot/left/actions/t_ns'][-1]<file.attrs['end_ns']


@pytest.mark.parametrize('sides,cameras',[(('left',),False),(('left','right'),True)])
def test_official_lerobot_export_and_loader(tmp_path,sides,cameras,monkeypatch):
    pytest.importorskip('lerobot.datasets.lerobot_dataset')
    monkeypatch.setenv('HF_HUB_OFFLINE','1')
    source=tmp_path/'native';source.mkdir()
    path=make_episode(source,sides,cameras)
    original=path.read_bytes()
    target=tmp_path/'export'
    report=export(source,target,'local/quest-test')
    assert len(report['episodes'])==1
    assert report['episodes'][0]['frames']>=29
    assert path.read_bytes()==original
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset=LeRobotDataset('local/quest-test',root=target,video_backend='pyav',depth_output_unit='m')
    assert dataset[0]['action'].shape==(len(sides)*7,)
    assert dataset.meta.info.codebase_version=='v3.0'
    if cameras:
        assert dataset[0]['observation.images.front'].shape==(3,48,64)
        depth=dataset[15]['observation.depth.front'].numpy()
        assert float(depth.max())==pytest.approx(.015,abs=.003)
    with pytest.raises(FileExistsError):export(source,target,'local/quest-test')

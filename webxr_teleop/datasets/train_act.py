"""Offline ACT smoke check or training using a prepared Quest bridge bundle."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def load_bundle(bundle):
    bundle = Path(bundle).resolve()
    if (bundle / 'INCOMPLETE').exists():
        raise ValueError('Bridge preparation is incomplete; create a new bundle')
    report = json.loads((bundle / 'source_report.json').read_text())
    if report.get('schema') != 'quest_act_bridge.v1':
        raise ValueError('Not a supported ACT bridge bundle')
    config = json.loads((bundle / 'act_config.json').read_text())
    if (config['training']['online_steps'] != 0 or config['training']['eval_freq'] != 0
            or config['policy']['name'] != 'act' or config['dataset_repo_id'] != report['repo_id']
            or config['wandb']['enable']):
        raise ValueError('Expected an offline ACT configuration with local logging')
    return bundle, report, config


def smoke_check(bundle, *, batch_size=1, device='cpu', small_model=False):
    """Decode real chunks, run ACT loss/backward, and check finite gradients."""
    import torch
    from torch.utils.data import DataLoader
    from .act_bridge import _legacy_api
    bundle, report, config = load_bundle(bundle)
    dataset = _legacy_api()(report['repo_id'], root=bundle,
                           delta_timestamps=config['training']['delta_timestamps'])
    from lerobot.common.policies.act.configuration_act import ACTConfig
    from lerobot.common.policies.act.modeling_act import ACTPolicy
    policy_config = {k: v for k, v in config['policy'].items() if k != 'name'}
    policy_config['pretrained_backbone_weights'] = None
    if small_model:
        policy_config.update(dim_model=32, n_heads=4, dim_feedforward=64,
                             n_encoder_layers=1, n_decoder_layers=1, n_vae_encoder_layers=1)
    policy = ACTPolicy(ACTConfig(**policy_config), dataset.stats).to(device).train()
    batch = next(iter(DataLoader(dataset, batch_size=batch_size, num_workers=0)))
    batch = {k: v.to(device) for k, v in batch.items()}
    loss = policy(batch)['loss']
    if not torch.isfinite(loss):
        raise RuntimeError('ACT produced a nonfinite training loss')
    loss.backward()
    gradients = [p.grad for p in policy.parameters() if p.grad is not None]
    if not gradients or not all(torch.isfinite(g).all() for g in gradients):
        raise RuntimeError('ACT produced missing or nonfinite gradients')
    return dict(loss=float(loss.detach()), batch_size=len(batch['index']),
                action_shape=list(batch['action'].shape), small_model=small_model,
                backward=True, optimizer_step=False)


def train(bundle, output, *, steps=100000, batch_size=8, device='cuda', pretrained_backbone=False):
    """Use a fresh process so the fork binds DATA_DIR before importing its loader."""
    bundle, _, _ = load_bundle(bundle)
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Training output must be new')
    if steps <= 0 or batch_size <= 0:
        raise ValueError('steps and batch_size must be positive')
    env = dict(os.environ, DATA_DIR=str(bundle), HF_HUB_OFFLINE='1', WANDB_MODE='disabled')
    args = [sys.executable, '-m', 'webxr_teleop.datasets.train_act', str(bundle), '--output', str(output),
            '--steps', str(steps), '--batch-size', str(batch_size), '--device', device, '--worker']
    if pretrained_backbone:
        args.append('--pretrained-backbone')
    subprocess.run(args, env=env, check=True)


def _worker(args):
    from omegaconf import OmegaConf
    from .act_bridge import _legacy_api
    bundle, _, config = load_bundle(args.bundle)
    if Path(os.environ.get('DATA_DIR', '')).resolve() != bundle:
        raise ValueError('Worker DATA_DIR does not match bundle')
    _legacy_api()
    config['device'] = args.device
    config['training'].update(offline_steps=args.steps, batch_size=args.batch_size)
    if args.pretrained_backbone:
        config['policy']['pretrained_backbone_weights'] = 'ResNet18_Weights.IMAGENET1K_V1'
    from lerobot.scripts.train import train as lerobot_train
    lerobot_train(OmegaConf.create(config), out_dir=str(args.output), job_name='quest_act')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--smoke', action='store_true', help='One loss/backward check; no training run')
    parser.add_argument('--small-model', action='store_true', help='Smaller transformer for smoke checks')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--steps', type=int, default=100000)
    parser.add_argument('--batch-size', type=int, default=1)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--pretrained-backbone', action='store_true',
                        help='Use ImageNet backbone weights (may download if not cached)')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.batch_size <= 0 or args.steps <= 0:
        parser.error('steps and batch-size must be positive')
    if args.smoke:
        print(json.dumps(smoke_check(args.bundle, batch_size=args.batch_size, device=args.device,
                                    small_model=args.small_model), indent=2))
    elif args.output is None:
        parser.error('--output is required for training')
    elif args.small_model:
        parser.error('--small-model is only supported with --smoke')
    elif args.worker:
        _worker(args)
    else:
        train(args.bundle, args.output, steps=args.steps, batch_size=args.batch_size,
              device=args.device, pretrained_backbone=args.pretrained_backbone)


if __name__ == '__main__':
    main()

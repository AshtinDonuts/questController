# ACT training from native recordings

Keep recording `quest_hdf5.v1` episodes as described in
[dataset_recording.md](dataset_recording.md). The offline ACT bridge creates a
separate training copy for the installed `alohaX_lerobot_fork` dataset API
**v1.6**. It never opens motor, camera, or supervisor connections. Hardware
launch endpoint checks are not needed for these commands.

Use the existing fork interpreter:
`/home/King/miniconda3/envs/lerobot/bin/python`. The bridge checks its dataset API
before creating output. It does not install or upgrade packages in that
environment. The existing `export_lerobot` command remains the separate path
for modern LeRobot v3 in its isolated Python 3.12+ environment; v3 output is
not readable by this older fork.

## Prepare a training bundle

Run from the project root, choosing a destination that does not exist:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.act_bridge \
  teleop_sessions/dataset training_data/quest_act \
  --repo-id local/quest-act
```

The source can be a dataset directory, its `accepted` directory, or one
accepted `.hdf5` file. Aborted, discarded, and unfinished episodes are excluded.
By default, all recorded arms and RGB cameras are selected, and the output FPS
is the slowest selected RGB camera rate. Depth stays in the native recording
and is omitted from the ACT copy.

Selections are explicit, for example:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.act_bridge \
  lishan_data/bowls_real training_data/bowls_left \
  --repo-id local/bowls-left --sides left \
  --cameras left_wrist top --fps 15 --chunk-size 30
```

`--sides left` projects the recorded state and actions to seven left-arm
coordinates; it does not invent right-arm demonstrations. Omit it to retain
both arms. Per arm, order is `waist, shoulder, elbow, forearm_roll, wrist_angle,
wrist_rotate, gripper`; dual-arm order is left then right. Arm values remain
radians and gripper values remain calibrated aperture (`0=closed`, `1=open`).

Use `--outcomes success` to select only explicitly success-labelled accepted
episodes. Without an outcome filter, all accepted outcomes are included and
unknown/failure labels are reported as warnings. An accepted file is a completed
recording, not a task-success certification. The bridge also reports constant
action dimensions, small demonstration counts, multiple task labels, and output
rates above camera rates. This ACT configuration is not language conditioned.

## Alignment and episode boundaries

All selected sources are validated before output is created. Different mock/live
modes, arm mappings, calibration snapshots, mounting geometry, camera shapes,
or camera rates require separate bundles. All selected camera images must have
the same resolution. `--fps` sets the output resampling rate; it does not relax
the camera-schema compatibility check.

At each uniform-grid time, the bridge uses preceding measured joint states and
RGB frames, then the following motor cycle's confirmed target. It trims to common
coverage and refuses gaps beyond `--robot-tolerance-s` (default 0.04 seconds) or
`--camera-tolerance-s` (default two camera periods). It does not interpolate
missing data. Timing is based on host receipt, not synchronized camera exposure.

**Every active segment separated by a recording pause becomes a separate training
episode.** Future action chunks stop at that segment's end, repeat its final
action for padding, and carry `action_is_pad`. They cannot cross a tracking
interruption or a source episode boundary. Reset frame indices/timestamps in
the training copy do not replace native timestamps in the source or provenance.

## Check and train

Decode a batch and run ACT loss and backward propagation without an optimizer
update:

```bash
OMP_NUM_THREADS=2 /home/King/miniconda3/envs/lerobot/bin/python \
  -m webxr_teleop.datasets.train_act training_data/quest_act \
  --smoke --small-model --device cpu
```

`--small-model` reduces transformer width/layers for a faster pipeline check;
it still uses the real ResNet-18, selected cameras, state dimensions, action
chunks, normalization, and padding. Omit it to check the full configured model.

Start offline training with a new output directory:

```bash
/home/King/miniconda3/envs/lerobot/bin/python -m webxr_teleop.datasets.train_act \
  training_data/quest_act --output outputs/train/quest_act \
  --device cuda --batch-size 8 --steps 100000
```

The runner uses the fork's existing trainer, optimizer, and checkpoint writer.
It starts a fresh process with `DATA_DIR` pointing to the bundle before the
fork imports its dataset loader. Simulation evaluation, online rollouts, Hub
access, and remote logging are disabled. These commands do not deploy a policy
or operate hardware.

Backbone weights default to random initialization so checks and training work
without downloading weights. For training with ImageNet initialization, add
`--pretrained-backbone`; torchvision may download those weights if uncached.
Other ACT/training settings are editable in the bundle's `act_config.json`.
The runner requires ACT, zero online steps, zero evaluation frequency, and
disabled remote logging.

## Bundle contents and failure behavior

The bundle contains:

- `<namespace>/<name>/train`: Arrow dataset with losslessly embedded RGB images.
- `<namespace>/<name>/meta_data`: fork-compatible FPS, normalization statistics,
  and episode ranges. Statistics are computed over the selected aligned copy.
- `act_config.json`: camera shapes, joint dimensions, chunk timing, and offline
  training settings derived from the selected data.
- `source_report.json`: source paths, SHA-256 hashes, UUIDs, full native metadata,
  task/outcome labels, native grid timestamps, source indices, timing offsets,
  sample reuse, and selection warnings.

RGB loading and writing are batched; no episode's full image array is loaded
into memory. Embedded PNG/Arrow storage can be larger than video exports.
Source HDF5 files are never modified. Existing destinations are refused. A failed
preparation leaves an `INCOMPLETE` marker and the runner refuses that bundle;
inspect the failure and prepare a new destination. Successful preparation reopens
the copy with the actual fork loader and decodes segment boundaries before
removing the marker.

The bundle can be moved as a unit: the runner resolves its data root from the
provided bundle path. Native source paths in the report retain their provenance.

## Regression checks

In an environment with the installed fork and pytest:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OMP_NUM_THREADS=2 \
  /home/King/miniconda3/envs/lerobot/bin/python -m pytest \
  tests/test_act_bridge.py tests/test_dataset_hdf5.py -q
```

Checks include source preservation, causal alignment, selection and schema
rejection, automatic FPS, pause isolation, exact RGB readback, action padding,
finite ACT loss/gradients, and a one-step training/checkpoint round trip. Modern
v3 loader tests are skipped in the legacy fork environment and run separately
with `requirements-dataset-export.txt`.

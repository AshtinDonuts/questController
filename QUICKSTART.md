# Start dual-arm teleoperation

Run from `/home/King/Desktop/QuestController`:

```bash
conda run -n quest_ros --no-capture-output python -m webxr_teleop.run \
  --scene dual --show-plane-bounds --base-y-range -0.7 0.7
```

- `conda run`: run inside a Conda environment.
- `-n quest_ros`: use the installed `quest_ros` environment.
- `--no-capture-output`: show live terminal messages.
- `python -m webxr_teleop.run`: start Quest-to-MuJoCo teleoperation.
- `--scene dual`: use both arms; left hand controls left, right controls right.
- `--show-plane-bounds`: display the enforced workspace boundaries.
- `--base-y-range -0.7 0.7`: allow 70 cm left and right of each arm’s base—a 1.4 m wide workspace.

The trailing `\` continues the command onto the next line.

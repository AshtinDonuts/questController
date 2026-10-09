# Single ViperX calibration scene

`single_viperx_scene.xml` contains one centered ViperX 300S arm with the existing ALOHA gripper, on a plain tabletop with no overhead frame. It uses the right Quest controller and `/aloha_sim/right/*` topics.

The arm's base is rotated so its neutral reach is along world +Y. The initial camera looks from behind the base: +X is right, +Y is forward, +Z is up. The single-arm target bounds are X [-0.45, 0.45] m, Y [-0.10, 0.65] m and Z >= 0.12 m; the dual-arm centerline rule does not apply. Existing tracking, clutch, rotation mapping and IK behavior are otherwise retained; axis calibration is still pending.

Run `quest_aloha_sim.py --scene single` (the default), or `--scene dual` for the original scene. Stop the previous simulator before launching another instance to avoid duplicate ROS publishers.

`single_viperx.xml` is derived from `../mujoco_menagerie/aloha/aloha.xml`, retaining its right arm, actuator definitions and neutral keyframe, and referencing its mesh assets. Source attribution and BSD-3-Clause license are in `../mujoco_menagerie/aloha/README.md` and `../mujoco_menagerie/aloha/LICENSE`.

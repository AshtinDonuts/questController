# Updated Quest browser teleoperation

## Planned real-robot architecture

The [Quest → ALOHA / dual ViperX control pipeline](docs/quest_aloha_control_pipeline.svg) is the project reference for the planned real-robot architecture (15 October 2026). It covers exclusive teleoperation/homing control, independent motor workers, hardware feedback, stop handling, and latched powered hold. Implementation and hardware validation are pending; the diagram describes planned behavior.

The updated single-arm setup uses StairVLA's WebXR browser interface, one-button axis alignment, filtered relative motion, IK reachability checks, headset feedback, and automatic MP4 recording. Start here: [WebXR setup and controls](webxr_teleop/README.md).

```bash
cd /home/King/Desktop/QuestController
conda run -n quest_ros --no-capture-output python -m webxr_teleop.run
```

Open the printed HTTPS URL in the Quest browser. Release right grip, point the drawn red controller X arrow forward, and press left X to align. Then hold right grip to move. This path directly drives the existing single ViperX MuJoCo scene; it does not require the old Quest app or ROS bridge. The sections below document the retained legacy ROS workflow and earlier calibration experiments.

---

# Quest → ALOHA 2 双臂 MuJoCo 仿真

此项目用 Meta Quest 3S 的左右手柄通过 ROS 2 控制 MuJoCo 中的 ALOHA 2 双臂。不会向真机的任何话题发布命令。

仿真模型来自 [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/aloha)，本地文件在 `mujoco_menagerie/aloha/`，下载时的提交为 `c96a32d28fb5da84da38c1da4d749e7a13212855`。模型是 **ALOHA 2 的简化模型**，与用户实际 ALOHA 型号和动力学参数可能不同。

## 运行

### Keep Quest awake during development

With the headset connected through ADB, bypass wear detection so removing it does not put it to sleep:

```bash
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close
```

Reapply after restarting the headset. Restart the headset to restore normal wear detection. To check whether it is awake:

```bash
adb shell dumpsys power | rg 'mWakefulness='
```

The expected state is `Awake`. Keeping the headset awake uses more battery.

电脑连接 Quest，启动头显内已安装的 `com.rail.oculus.teleop` 应用。Quest→ROS 桥接已设置为用户级后台服务，可这样启动和检查：

```bash
systemctl --user start quest-pose-bridge.service
systemctl --user status quest-pose-bridge.service
journalctl --user -u quest-pose-bridge.service -f
```

也可不使用服务，在终端 1 手动运行：

手动运行前请先执行 `systemctl --user stop quest-pose-bridge.service`，避免两个发布节点同时发送相同话题。

```bash
conda run -n quest_ros --no-capture-output python /home/King/quest_pose_test/quest_ros_bridge.py
```

终端 2：

```bash
conda run -n quest_ros --no-capture-output python /home/King/Desktop/QuestController/quest_aloha_sim.py
```

双臂 MuJoCo 窗口会打开。握住左/右手柄的**握把扳机**（中指位置，数值超过 0.6）时，对应模拟臂开始跟随手柄的相对运动；松开后停止跟随，再按住会重新对齐起点。**食指扳机**控制同侧夹爪：0 为打开，1 为闭合。

仿真采用位置级逆运动学和 MuJoCo 运动学更新，当前用于验证数据与末端运动映射；尚未验证真实接触力或电机动态。输入在 0.35 秒内没有更新时会停止跟随。手柄单帧跳变超过 0.12 m 时，机械臂会保持当前位置，跟踪稳定后自动重新对齐。单次离合的位移超过 0.30 m 时目标位移会被限制在 0.30 m 内。

**当前方向映射尚未校准。** 根据上抬后反向的反馈只反转平移 Z 轴，用户复测发现更差且移动很小。请先做下面的原始位姿动作测量，再决定映射矩阵；不要把当前行为用于真机。当前代码的映射为 `仿真 X=Quest X`、`仿真 Y=-Quest Z`、`仿真 Z=-Quest Y`，旋转仍使用另一矩阵。正常手柄移动量按 **1:1** 映射到末端目标。模拟臂另有关节限位、桌面上方高度限制、左右臂不越过中线的限制。

## 测量手柄实际方向

佩戴头显并让 Quest 采集应用保持前台，在终端运行以下命令。运行后先保持右手柄不动 2 秒，看到提示后**竖直抬高约 10 cm 并保持**到程序结束；此步骤只读 ROS 位姿，不操作机械臂。

```bash
conda run -n quest_ros --no-capture-output python /home/King/Desktop/QuestController/measure_quest_motion.py --side right --motion up
```

脚本会同时在终端打印并自动保存到桌面的 `quest_motion_right_up.txt`。把文件内容贴给我即可。若 `Largest frame jump` 超过 0.12 m，先重置 Quest 跟踪后重测。对前、右方向可把 `--motion up` 分别改为 `--motion forward` 和 `--motion right`，动作也对应改变。根据三个正方向的实测位移向量确定基座坐标变换，再统一用于位移和旋转。

## 观察 ROS 输出

```bash
conda run -n quest_ros --no-capture-output ros2 topic echo /aloha_sim/right/ee_pose
conda run -n quest_ros --no-capture-output ros2 topic echo /aloha_sim/right/engaged
conda run -n quest_ros --no-capture-output ros2 topic echo /aloha_sim/right/gripper_position
conda run -n quest_ros --no-capture-output ros2 topic echo /aloha_sim/right/ee_target
conda run -n quest_ros --no-capture-output ros2 topic echo /aloha_sim/right/position_error
```

左臂把 `right` 替换为 `left`。`engaged: true` 表示握把离合已启动。夹爪值范围约为 0.002 m（闭合）至 0.037 m（打开）。

若只想在无图形界面环境中运行：

```bash
conda run -n quest_ros --no-capture-output python /home/King/Desktop/QuestController/quest_aloha_sim.py --headless
```

脚本依赖隔离的 Conda 环境 `quest_ros`，其中已安装 ROS 2 Jazzy 与 MuJoCo 3.13。与真机对接前，需要另外验证实际机器人的型号、零位、限位、坐标系及驱动接口。

## D405 + screen recording for IK calibration

Use [CALIBRATION.md](CALIBRATION.md) for camera placement, setup commands, the guided 275-second motion sequence, and the coordinate/IK calibration procedure. `record_ik_calibration.py` records RealSense RGB-D, a MuJoCo screen region, Quest and simulator ROS telemetry, and timed cues into a timestamped session folder. It does not change the simulator mapping.

```bash
python record_ik_calibration.py --plan-only
conda run -n quest_ros --no-capture-output python record_ik_calibration.py --side right
```

## Simple direction-only calibration (no camera)

For translation/rotation axis identification, run the interactive six-motion recorder:

```bash
conda run -n quest_ros --no-capture-output python /home/King/Desktop/QuestController/record_quest_axes.py --side right
```

The recorder uses the large middle-finger grip pedal. Stop MuJoCo during capture because this grip also engages simulator teleop. Return to neutral with the pedal released; squeeze and hold, stay still for the one-second baseline, then move when prompted. Hold the endpoint for one second and release to finish. No keyboard input is required. The default sequence is forward, right, up; add `--include-rotations` for tilt upward, turn right and clockwise twist. Exact distances are unnecessary. Only grip-held poses are saved; stale streams and short trials are rejected and repeated. Results and raw samples are saved under `calibration_recordings/axes_*`. No mapping is applied automatically.


## Single forward-facing ViperX scene (new default)

The simulator now starts with one centered arm, operated by the right Quest controller, on a plain table without the black frame. The initial view is from behind the arm: right = world +X, forward = +Y, up = +Z. Stop any old simulator before launching:

```bash
conda run -n quest_ros --no-capture-output python /home/King/Desktop/QuestController/quest_aloha_sim.py --scene single
```

The original two-arm scene remains available with `--scene dual`. Direction mapping still requires calibration. See [scene details](scenes/README.md).

## Git checkout

The MuJoCo Menagerie models are pinned as a submodule. After cloning this project, fetch the model checkout with:

```bash
git submodule update --init --recursive
```

Calibration JSON and pose traces are versioned for reproducibility. Generated videos, camera bags, logs, and Python caches remain local.

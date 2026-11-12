# Cupstack 离线模拟器回放

从本目录运行：

```bash
/home/King/miniconda3/envs/lerobot/bin/python replay_cupstack_sim.py
```

默认选取有数据文件的最新已保存 episode；`--episode 0` 可选指定编号。
生成 `sim_replay/episode_NNN/index.html`、`comparison.mp4` 和
`simulation.validation.json`。已有视频会拒绝覆盖；可直接打开已有播放页。
通过本机 HTTP 服务打开播放页，可循环播放、拖动、慢放和逐帧查看。

对照视频从左到右是 MuJoCo 指令关节回放、MuJoCo 实测关节回放、原始俯视录像。
关节直接使用数据中的模型弧度；不重复套用电机 sign/offset。
夹爪 0–1 按 rig.json 的闭合/张开指尖位置转换。
双臂安装位置和方向取自该数据集的 recording signature。

这是 15 Hz 运动学回放，通过 `mj_forward` 重现关节位姿，无电机、相机或 supervisor 连接。
不运行动力学，不生成杯子或推断物体轨迹，因此不能验证抓取成功、接触力或堆叠稳定性。
可检查轨迹、左右臂顺序、夹爪时序及 action/state 与录像的对应关系。
录制采用主机接收时间对齐，相机曝光不保证硬件同步。
验证报告检查模型关节范围和关节赋值误差；这不等同于验证真实机器人校准。

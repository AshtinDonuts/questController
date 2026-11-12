# 视频尾段冗余初筛

2026-11-11；原始数据未修改、未裁剪。

检查全部 295 条 episode 的 measured state 和 action，寻找末尾最长稳定区间（最多回看 30 秒）：所有旋转关节在该区间的极差不超过 0.03 rad，所有夹爪极差不超过 0.05；状态和命令都须满足。该指标仅筛选停止运动候选，不等于任务完成时刻；微动也可能导致漏检冗余。

同时抽看 15 条不同 episode 的 top 相机，在结尾前 12、8、4、2 秒及最后一帧取样。各任务对比图保存在 wamu_tail_review/；bowls 图中 ep25 重复出现一次。未连续播放全部视频，未逐帧审核三路相机。

| 任务 | 稳定尾段 ≥2 秒 | ≥5 秒 | 最长稳定尾段 |
|---|---:|---:|---:|
| bowls_lerobot | 0 | 0 | 1.73 秒 |
| cupstack_lerobot | 5 | 0 | 2.87 秒 |
| eggpack_lerobot | 0 | 0 | 1.67 秒 |
| headphones_lerobot | 7 | 0 | 4.50 秒 |

## 优先复核候选

| 任务 | episode（0 起算） | 总时长（秒） | 稳定尾段（秒） |
|---|---:|---:|---:|
| cupstack_lerobot | 68 | 47.13 | 2.87 |
| cupstack_lerobot | 63 | 56.27 | 2.40 |
| cupstack_lerobot | 79 | 47.87 | 2.40 |
| cupstack_lerobot | 12 | 55.93 | 2.00 |
| cupstack_lerobot | 16 | 50.93 | 2.00 |
| headphones_lerobot | 9 | 14.57 | 4.50 |
| headphones_lerobot | 33 | 18.00 | 2.77 |
| headphones_lerobot | 15 | 34.33 | 2.30 |
| headphones_lerobot | 4 | 17.60 | 2.13 |
| headphones_lerobot | 22 | 16.50 | 2.13 |
| headphones_lerobot | 36 | 12.27 | 2.10 |
| headphones_lerobot | 29 | 16.23 | 2.07 |

## 画面观察与处理建议

- headphones ep9：最后约 4 秒取样画面几乎不变，与 4.5 秒稳定尾段一致，是最明确的停留候选；仍应确认耳机放置结果，再确定裁剪点。ep33 最后约 2 秒取样也基本不变，但末端遮挡影响成功判断。
- cupstack ep68、ep63：物体已放下、机械臂撤回后，最后约 2 秒取样基本不变，存在短尾段候选。
- bowls 抽样末尾仍有放置/撤离动作；最长严格稳定尾段仅 1.73 秒。
- eggpack 抽样末尾包含放置/撤离动作；ep17 的画面仍有多颗物体留在盒外，不能仅因末尾稳定就视为成功示范。需按任务完成标准复核。

不建议统一裁掉最后 5 秒。对确认完成的 episode，可保留完成后约 0.5–1 秒作为停止示范，再去掉多余等待；具体切点需逐条复核。任务完成后的撤臂是否保留，取决于训练目标。任何正式裁剪都应同步处理 state/action、三路视频、时间戳、episode 索引和统计信息，而不只裁 MP4。

## 后续数据变更

已按用户要求删除 eggpack 原 episode_index=35，并重新编号；本文和 wamu_tail_audit.json 的尾段筛查使用删除前编号。原 36–79 现在对应 35–78，原 34 保留。详见正式数据集的 curation_report.json 映射。

Headphones 后续已从 30 Hz 降采样至 15 Hz，episode 编号不变，但帧编号和统计需以 resampling_report.json 为准；本尾段报告保留原 30 Hz 筛查结果。

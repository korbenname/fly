# 第2步 EKF2 / Position 只读基线核对（2026-11-18）

本审查只读主机现有原始 ULog、先前证据与固定 PX4 源码；没有连接香橙派/PX4、修改参数、串口、固件、服务或飞行程序。每份 ULog 独立分析，不以文件名日期证明同次试飞，不对齐不同文件的 boot 时间。本轮结果不能称为当前板端读回。

复现：`/usr/bin/python3 /tmp/fc01-step2-ekf-audit.py`；详细数据 `/tmp/fc01-step2-ekf-baseline.json`，stdout `/tmp/fc01-step2-ekf-audit.log`。脚本对原始文件 SHA 与旧清单比对，并在解析后重算确保未改变。脚本的 topic-filter 时间不是完整 ULog duration；新 log_82 另用全 topic parse 给出完整时长。

## 先前证据边界和本次新增发现

- 旧清单 `docs/investigations/20261116_stage1/evidence/ULOG_INVENTORY.json` 有 30 个去重 SHA，本次其中 24 份原路径仍可读取，6 份原桌面路径已不存在，均如实记录为 unavailable，而不冒称重读通过。未删除/修改旧证据。
- 本次用 `rg --files /home/jikl62/DroneProjects/05_飞行日志与测试证据 /home/jikl62/桌面 /home/jikl62/Desktop 2>/dev/null | rg '\.ulg$'` 在局部路径发现旧清单之外的 `/home/jikl62/桌面/log_82_UnknownDate.ulg`。不是全文件系统穷尽搜索。
- 共 31 个记录条目：25 份当前原文件实际解析，6 unavailable。实际解析中 23 份为 DAMIAO_DM_FC01/NuttX metadata、2 份为 PX4_SITL。
- 旧 22 份可读硬件记录没有 Position(nav_state=2)。新增 log_82 确有硬件 Position，但仅约 2 秒且 land detector 标为已接地，不证明用户所述稳定空中 Position 飞行。
- 另一份 Position 来自旧归档 `FC01_M1_S10_failure.ulg`，metadata 明确 PX4_SITL，5.004 秒区间处于 failsafe/landing；仍不能作物理 MTF 基线。

## 新的真实硬件记录 log_82

| 身份/范围 | 直接读取结果 |
|---|---|
| 原始路径 | /home/jikl62/桌面/log_82_UnknownDate.ulg |
| SHA256 / 大小 | a5f07db65ea44433af07fd20fb9bd47880d23c325802a73c715de9a99dac6a8e / 178768 B |
| 固件 | 07983bfb894b9372d7520d43ac10dc792d0b4b9f，DAMIAO_DM_FC01 / NuttX |
| sys_uuid | 0006000000003638373135335106004b0021；与旧记录中另一 FC01 UUID 不同，不能把它们自动合为同一硬件配置 |
| 全解析窗口 | boot 691.839367 → 694.034528 s，2.195161 s |
| 模式 | nav=2 六条、nav=15 一条；Position 持有区间 [691.824607,693.694838) s，1.870231 s（第一状态比header早14.760 ms） |
| 解锁 | Position 内3条 armed、3条 disarmed；RC解锁消息691.824487 s、RC上锁消息693.049089 s；按状态为约1.224497 s armed窗口 |
| 飞行/接地 | landed/ground_contact/maybe_landed均3/3 true；cs_in_air 3/3 false；不是稳定悬停证据 |
| 失效状态 | 7条 vehicle_status 的 failsafe 均false；无 ULog dropout 记录，不能据此证明所有 topic 完整 |
| 角速度 | 原始 ULog vehicle_angular_velocity 93条；只能证明PX4内部有记录，不证明DDS导出 |

本文件全窗口 20 条 local 样本，Position内16条：

- `z` NED m 范围 +2.220780134 → +2.224488258，span 0.003708124 m；**local原点不是地面，高度不能读成+2.22m离地**。
- `vz` 向下正 m/s 范围 +0.008067828 → +0.013174905；`z_deriv` 范围 −0.003331984 → +0.003210760，两者不是同一估计量。
- xy/z/vxy/vz_valid、dist_bottom_valid 全部20/20 true；dead_reckoning全部false；heading_good_for_control全部false。
- xy/z/vxy/vz/heading/dist_bottom reset counters恒40/1/32/1/3/2，姿态quat_reset_counter恒3；本短窗口未观察到变化，不等于更早没有重置。
- dist_bottom范围0.093377829～0.095575571m。唯一distance_sensor报0.02m、quality100、orientation25、max12m、device_id9699590；不能单凭这些字段认定物理传感器就是MTF或其测量精度。

### 本文件的参数与实际融合

`EKF2_HGT_REF=2`（固定源码定义：Range sensor reference），`EKF2_RNG_CTRL=1`（conditional）、`EKF2_OF_CTRL=1`、`EKF2_BARO_CTRL=1`、`EKF2_GPS_CTRL=7`、`EKF2_EV_CTRL=9`、`MPC_ALT_MODE=2`。`SYS_HAS_NUM_DIST/OF=0`，表明这些配置数字并非实际“是否在融合”的充分条件。

三条 estimator_status_flags：`cs_rng_hgt=1`、`cs_baro_hgt=1`、`cs_ev_pos=1`、`cs_ev_yaw=1`；`cs_opt_flow=0`、`cs_gnss_pos/gps_hgt=0`、`cs_ev_hgt/vel=0`。range-height、baro-height、EV-position、EV-yaw aid-source各3条均 `fused=1`、`innovation_rejected=0`。原始视觉odometry、optical-flow topic没有记录，**不能把启用光流参数当作本短窗口光流在融合，也不能定位EV输入来源**。

因此可确认这份短硬件记录的高度配置与旧记录不同，并有range高度及EV位置/航向融合观察；不能声称这是用户当前配置，或证明稳定空中Position效果。JSON保存本文件全部初始参数、原样稀疏fusion/status/land/distance/aid-source样本、全部topic计数及日志消息。

## 旧硬件证据仅作历史背景

22份可读旧硬件记录HGT_REF中21份为1(GPS reference)、1份0(baro reference)，OF/RNG_CTRL均1，但这些记录里被记录的estimator实例全部 `cs_opt_flow=0`、`cs_rng_hgt=0`、`cs_baro_hgt=1`。它们不能替换log_82的HGT_REF=2/EV_CTRL=9，更不能推断用户现在MTF线路失效。

所有原始记录的z/vz有效率、数值范围、reset计数及实际观测变化均写入JSON。没有把不同实例/不同文件的“有效”合并成一个连续飞行。多EKF实例的flags分实例保留；缺selector时不猜哪个是被选中的主估计器。

## PX4源合同及对本轮桥的影响

固定源目录 `/home/jikl62/fc01_px4_dds_build/PX4-DM-FC01-v1.16` HEAD 07983bfb…；既有dirty为dds_topics.yaml与uxrce_dds_client.cpp，未修改。EKF2.cpp及参数定义为只读。

- `EKF2.cpp:1538 PublishLocalPosition`：位置与速度是body原点local NED；timestamp_sample用EKF更新timestamp，timestamp为发布时hrt；不是相机/MTF原始采集时刻。
- `EKF2.cpp:1570–1571`：z_valid与v_z_valid采用垂向位置有效 **或** 垂向速度有效兼容逻辑，不能将两个true当成两个独立物理观测证明。
- `EKF2.cpp:1601`附近：dist_bottom为独立 terrain 估计及valid，不等于local z；reset counters显式给出。
- `params_optical_flow.yaml:5` OF_CTRL只是允许光流融合；`params_range_finder.yaml:5` RNG_CTRL=1为条件模式；`module.yaml:87` HGT_REF=2才是range reference。**本轮不改变任何这些参数。**
- 现有 `dds_topics.yaml:47` **vehicle_angular_velocity仍注释**，local/attitude/estimator_status_flags已列入。ULog里有角速度不等于DDS有角速度；生产桥若要求该输入应明确报缺失，而不能以零假造已测得值。是否用户已刷不同DDS固件需现场图核实。
- 相机目标relative FRD不是PX4自机NED。mission原 `_message_age`截断年龄至0..0.15s、`_on_state`重复seq无拒绝、`_on_velocity/_on_omega`没有独立stamp/接收TTL，是当前源码可确认的时效缺口；root正在本步骤实现回归与修复。

## 当前仍缺、后续只读取证要求

1. 用户**当前**那次稳定Position实飞的完整ULog及参数快照，能同时看到armed/in_air、MTF distance/flow aid source、fusion flags和固件UUID；当前log_82只证明接地短片段。
2. 当前香橙派runtime.yaml、fc01-navigation.service/实际进程路径、ROS域和完整topic端点；无板连接不能更新现场基线。
3. 实际PX4 DDS angular_velocity导出、时间同步与sample推进；不能替换为模拟流或伪造字段。
4. 同一相机标定结果、body安装旋转/杆臂、曝光/采集延迟实测与来源ID，不能用上游作者环境数值填“已确认”。

这不阻止在开发机继续完成坐标、时效、无效撤销的可重复验证，也不构成修改用户已飞稳MTF参数的理由。

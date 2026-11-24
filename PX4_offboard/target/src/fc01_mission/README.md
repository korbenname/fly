# FC01 任务与水平制导

实机任务节点消费 PX4 的融合自机状态和相机目标测量，生成水平运动意图。
原生 `fc01_system_node` 独占人工 RC 准入、起飞、`h = z0 - z` 的 1.5 m 高度环、
姿态／推力发布和故障保护。任务节点不请求解锁、自动起飞或切换 Offboard。

这是尚需现场标定与实机验证的水平导航候选。仓库默认 `control_enabled=false`，
原生 `visual_navigation_enabled=false`，新增 `px4_horizontal_enabled=false`。
仅启动视觉任务不会改变已有定高控制权。开关和标定记录必须在现场配置中明确确认。

## 实机数据流

| 输入 | 类型与用途 |
| --- | --- |
| `/fc01/platform/state` | `PlatformState`：真实 PX4 DDS 位置、速度、姿态、可用角速度；内部使用 ENU／FLU，原始时间、有效位和重置身份分别保留 |
| `/fc01/native/flight_state` | `NativeFlightState`：当前原生进程在最终发布边界提供的真实离地、连续到高和 RC／Offboard 授权；旧状态不能授权 |
| `/fc01/vision/target_observation` | `TargetObservation`：目标相对相机光心的 FRD 测量；不是自机里程计，不能作为 PX4 高度反馈 |

MTF‑02P 继续直连 PX4，经 EKF2 融合后由 DDS 回传自机状态；任务不改其配置。
相机安装旋转由视觉节点应用，FLU 杆臂在任务 `_ingest()` 中加一次。
目标测量使用图像源时刻对应的平台历史姿态／位置对齐，不把“最新姿态”冒充拍摄姿态，
也不靠重复发布或猜测延迟给旧图像续鲜。

## 实机状态与切换

1. 未取得新鲜且来源一致的原生连续到高证据时，状态为 `wait_native`，所有运动意图失效。
2. 原生允许导航后，任务保持 `native_height_hold`，或发布已确认原点且范围内的水平航点。
3. 连续有效新图像达到锁定帧数，且外参、安全间距和源／接收时效均有效，才进入 `track`。
   此时航点通道发 `valid=false` 正常撤销，独立水平速度封套接续。
4. 实际跟踪发布后失锁、过期、重置或来源切换，撤销水平和偏航需求并锁存任务故障。
   不自动恢复旧航点；原生控制器按现有保护受控下降，RC 接管／Kill／失联／电池／PX4 failsafe 优先。
5. 同一当前原生进程开始新的任务 epoch 后才解除该任务锁存。仅相机恢复或任务计时不能解除。

起飞到高前、航点、定点和正常跟踪均没有任务垂向权限。航点 z、目标 z、视觉 vz、
`takeoff_altitude_m` 和 `push_altitude_m` 不接管实机高度目标。

## 输出合同

| 输出端点（生产 launch 默认值） | 语义 |
| --- | --- |
| `/fc01/navigation/command` | `NavigationCommand`，只有当前水平航点或定点意图。生产 frame 为 `px4_local_ned:<PlatformState.origin_id>`，reset 为任务生产者 epoch；XY 为北／东米，z=0 占位无高度权限 |
| `/fc01/vision/horizontal_command` | `HorizontalVelocityCommand` V1，生产执行端点；内含 V2 指令、独立姿态采样戳和 PX4 原点身份 |
| `/fc01/vision/velocity_command_v2` | 原 `VelocityCommandV2` 兼容诊断输出，不作为本候选生产执行入口 |
| `/mission/state`、传感器状态页 | 任务阶段、缺测／标定／锁定原因，仅显示，不授予飞行控制权 |

`VelocityCommandV2.schema_version=2`，`stamp` 保持指令生成 ROS 时刻。
封套的 `attitude_sample_stamp_ns` 精确标识把世界水平速度编码到 FRD 时使用的实际姿态样本，
原生必须匹配相同样本和原点，再完整旋回 NED 取 XY。倾斜时 body z 分量可能非零，
它只是完整三维编码的一部分，不能先删掉，也不代表获得垂向控制权。
纯 body-Z 请求（vx=vy=0）按明确的水平接口策略视为无水平需求。

封套 `command.wz` 是期望 NED 航向变化率，单位 rad/s，顺时针为正；
它由负的 ENU 航向变化率得到，不是 PX4 实测机体角速度。
`wz=0` 由原生保持已提交航向。退出跟踪时速度和偏航速率全部置零，并发布失效。
任务禁用或反馈过期同样显式撤销，不能把 `valid=false` 解读为继续沿用旧速度。

原生每周期只选择一个水平意图，当前跟踪优先于航点／定点。
航点正常撤销与已进入跟踪后的故障撤销分别处理，跨周期消息不能留下旧偏航速率。

指令 QoS 为 reliable、volatile、keep-last 1；传感器反馈为 best-effort、keep-last 1。
话题均可配置／remap，旧 `/navigation_command`、`/visioncontrol` 在生产 launch 中映射到上表端点。

## 限幅与现场配置

任务和原生均执行限制：水平速度不超过 0.5 m/s、偏航速率不超过 0.35 rad/s、
航点水平半径不超过 2 m，并保留原生加速度、倾角、推力、总限高和最终发布安全检查。
任务按世界 XY 模长限幅后再完整旋转，生产不执行破坏投影的 body 逐轴裁剪。

航点需要 `waypoint_alignment_confirmed` 和与当前 PX4 完整原点相同的
`waypoint_alignment_origin_id`。任务的范围锚点是本轮首个有效平台位置；
原生另在本轮首次合格水平控制周期捕获锚点，两道范围门取保守交集。

跟踪需要现场核验的 `camera_extrinsics_confirmed/id` 和
`standoff_calibration_confirmed/id`。仓库的 1 m 安全间距只是未确认候选值，
不能代替相机标定、机架测量或现场安全距离；缺项会在状态页明确显示。

## 构建与候选启动入口

唯一自定义消息来自 `native/src/jikl_fc01_system`，不要再构建上游同名消息快照。
使用工程统一的 Humble 构建／安装入口及现有 PX4 消息 underlay。
以下命令要求已 source 对应候选安装树，仅用于启用观察入口：

```bash
ros2 launch fc01_bringup horizontal_first_test.launch.py \
  site_mission_config:=/absolute/path/to/site_mission.yaml
```

`site_mission_config` 必须显式提供完整现场配置；入口固定 `track`，默认任务关闭。
此入口启动视觉、PX4 实际状态桥、任务和只读诊断接收器；不启动第二个原生控制节点。
真实原生控制器仍由已有 systemd 服务管理，现场依原人工 RC 流程确认就绪。
硬件上未验证相机标定、时延和水平闭环前，不能把离线通过解释为实飞通过。

## 验证

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_mission_*.py' -v
# source 当前候选 Humble 安装树后，在独立本机域运行合成输入通信验证：
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=174 /usr/bin/python3 \
  tests/ros_mission_horizontal_smoke.py --output /tmp/mission-horizontal.json
```

第二条使用实际任务节点和 DDS 消息、合成 PX4／原生授权／目标观测；
不启动飞控控制器或 `/fmu/in/*` 发布者，也不产生真实视觉性能结论。

## 保留的上游模拟分支

只有显式 `platform_input_mode=simulation` 才消费 `/sim/platform_*`、接触力、
impact 和 reset 事件，并保留上游 `takeoff → seek → strike/push → retreat/hold` 算法背景。
实机 launch 强制 PX4 输入和 `track`；`push/impact` 在实机配置中会明确报错。
上游的绝对模拟高度、冲撞速度与接触模型不能作为 FC01 真机操作步骤。

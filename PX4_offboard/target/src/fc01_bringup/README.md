# FC01 视觉任务启动入口

`robot.launch.py` 启动实际 PX4 自机状态桥、相机目标检测、mission 和独立只读接口接收器。
原生 `fc01_system_node` 继续由现有 systemd 服务独占；本入口不另启飞控控制节点，
不请求解锁、切换 Offboard 或启动电机。默认任务关闭，原生视觉开关保持关闭。

观察入口：

```bash
ros2 launch fc01_bringup robot.launch.py start_camera:=false control_enabled:=false
```

首次水平候选验证入口：

```bash
ros2 launch fc01_bringup horizontal_first_test.launch.py \
  site_mission_config:=/absolute/path/to/site_mission.yaml
```

后一入口要求显式现场 mission 配置，固定 `track`，默认 `control_enabled=false`。
现场外参、安全间距、航点原点未经确认时，任务会明确报告缺项；不能将仓库候选值视为标定。
具体授权、消息和故障行为见 [mission 合同](../fc01_mission/README.md)，
工程构建见 [target/README.md](../../README.md)。

| robot launch 参数 | 默认 | 作用 |
| --- | --- | --- |
| control_enabled | false | 任务水平意图开关，不替代 RC／原生到高授权 |
| engage_mode | track | 实机只允许 track，push／impact 只在显式模拟分支存在 |
| start_camera | true | 启动 headless USB 目标检测；观察时可关闭 |
| start_px4_bridge | true | 启动真实 PX4 DDS → PlatformState 桥，不伪造缺失字段 |
| vision_config | fc01_vision/config/vision.yaml | 相机配置；不改原生 runtime.yaml |
| mission_config | fc01_mission/config/mission.yaml | 完整任务配置，默认不确认现场标定 |
| bridge_config | fc01_mission/config/platform.yaml | PX4 DDS 端点、时效与质量检查 |
| target_topic | /fc01/vision/target_observation | 相对相机光心目标量，不是自机位置 |
| platform_state_topic | /fc01/platform/state | PX4 融合自机实际反馈 |
| native_flight_topic | /fc01/native/flight_state | 当前原生离地／到高／控制权与发布证据 |
| navigation_topic | /fc01/navigation/command | 对齐并限域的水平航点／定点意图 |
| velocity_topic | /fc01/vision/velocity_command_v2 | 兼容诊断速度消息，不作为新增生产执行入口 |
| horizontal_topic | /fc01/vision/horizontal_command | 携带精确姿态采样戳与原点的生产速度封套 |
| read_only_receiver | true | 字段接收器，发布 /fc01/interface/status，无飞行控制权 |
| control_debug_hz | 0.0 | mission 调试日志频率 |
| publish_debug_image / show_tuning_window | false / false | 不打开桌面图像窗口 |

所有端点参数进入实际发布／订阅者。指令 reliable、volatile、keep-last 1，
目标和自机测量 best-effort、volatile、keep-last 1。
旧 V1 速度诊断与自机 VisionState 端点保持兼容，不被相机相对目标占用。

实际飞行仍需原人工 RC 起飞流程和原生 1.5 m 连续稳定授权。
mission 只生成水平意图，不能改高度目标；当前 ARM64、现场标定、真实时延和水平闭环
尚需现场验证，离线测试通过不能代替这些证据。

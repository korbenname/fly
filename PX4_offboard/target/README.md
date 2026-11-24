# FC01 目标视觉接口工作区（队列第1～5步）

本目录导入 korbenname/fly 固定提交 `c4b32b92455a9c25fb1bce1262e8426f249f8521` 的三个业务包，原文件身份保存在 UPSTREAM.json。包元数据的 Apache-2.0 声明保留，未补造上游不存在的 LICENSE 文件。

已完成接口统一、PX4实际状态桥、相机测量时效、原生到高接续和水平执行候选；尚未进行真实相机辅助飞行。原生控制器独占人工RC起飞和1.5m垂向闭环，视觉开关仍关闭。目标识别程序不是MTF光流里程计，不能把它的目标相对位置填入自机NED状态。

## 唯一消息与构建

不包含第二个 `target/src/jikl_fc01_system`。十一种消息统一来自本工程 `native/src/jikl_fc01_system/msg`，同一colcon安装树生成C++/Python类型支持。

```bash
FC01_PX4_PREFIX=/你的现有SDK/install/px4_msgs bash target/build.sh
source target/install/setup.bash
```

SDK若为merged-install，把前缀指向其中包含share/px4_msgs的目录。本阶段在开发机Ubuntu/Humble x86构建通过；不要拿这些x86产物替换香橙派AArch64程序。ARM64发行留给队列第6步。

| 用途 | 类型 | 默认端点 | 语义 |
|---|---|---|---|
| 目标观测 | TargetObservation V1 | /fc01/vision/target_observation | 相机中心原点、FRD分量的目标相对位置/变化率；不是自机里程计 |
| 兼容速度诊断 | VelocityCommandV2 schema2 | /fc01/vision/velocity_command_v2 | 保留 vx/vy/vz/wz 字段供旧调用方诊断；不直接进入新增执行通道 |
| 生产水平速度 | HorizontalVelocityCommand V1 | /fc01/vision/horizontal_command | V2载荷加编码姿态原采样戳/PX4原点；完整FRD→NED后只取XY，wz为NED期望航向速率 |
| 水平航点/定点 | 原NavigationCommand | /fc01/navigation/command | 新选择器只消费匹配PX4原点的XY/yaw；valid=false撤销本通道，z无垂向权限；旧视觉合同保留 |
| PX4自机反馈 | PlatformState | /fc01/platform/state | 原子位置/速度/姿态/角速度及各自有效性和源时刻 |
| 原生任务接续 | NativeFlightSnapshot | /fc01/native/flight_snapshot | 原生状态与其已消费的精确raw姿态、原点、首次接收时间同包；只读 |
| 兼容原生诊断 | NativeFlightState | /fc01/native/flight_state | 本次地面参考、真实离地/连续到高、当前发布证据；旧类型和端点保留 |
| 旧诊断 | 原VelocityCommand V1 | /fc01/vision/velocity_command | 无wz；旧布局和诊断调用方保留 |
| 自机状态 | 原VisionState | /fc01/vision/state | 原有世界NED实际自机状态/姿态/质量；目标相机不发布这里 |
| 接收证据 | std_msgs/String | /fc01/interface/status | 只读接收字段/计数/本地实际QoS，control_authority=false |

速度和导航意图使用reliable / volatile / keep-last1，目标、平台与原生接续使用best-effort / volatile / keep-last1。原始测量stamp/sequence保真，源时间、单调接收时间与样本推进各自受限；重复帧不能续鲜。

第5步让任务按原子快照中的原生精确姿态编码速度，避免200Hz原始姿态与50Hz平台桥各自丢帧/降采样造成错误匹配。平台桥仍用于图像时刻的自机反馈插值；相机高帧率不能反复覆盖唯一待对齐帧。新消息、原生和任务必须成套构建，不能只替换一个Python文件；原生最终精确匹配和TTL没有放宽。

## 生产入口与配置

```bash
ros2 launch fc01_bringup robot.launch.py start_camera:=false control_enabled:=false
```

默认 headless。`start_camera:=true` 启动现有相机采集；相机设备/标定沿导入配置，尚未现场核验。该入口启动mission和**只读**原生接口接收器，不启动第二个飞行控制节点，原控制服务仍由systemd管理。

`vision_config:=/绝对路径/相机.yaml` 可选择现有相机配置，不改原生runtime。可传 `target_topic:=... navigation_topic:=... velocity_topic:=...`，实际相机/mission发布者和接收器同步remap。默认将视觉旧名 `/visioncontrol` 和 `/navigation_command` 映射到上表端点；这是已运行测试的启动代码。

生产 launch 强制 `platform_input_mode=px4`，任务从 `wait_native` 开始。原生人工RC流程先建立本轮地面零点 `h=z0-z`，完成真实零推力预流后才准备人工解锁；8秒内选导航槽位1并取得PX4实际Offboard确认，才进入爬升。任务等待真实离地及连续高度/垂速稳定和当前发布授权；检测成功或任务计时不能替代这些证据。

生产任务只计算世界水平意图，不进入旧simulation的起飞／冲撞高度分支。独立 `HorizontalVelocityCommand` 封套携带编码姿态的原始采样戳与PX4原点；倾斜时body-FRD的z分量可非零，原生完整旋回NED后只取XY，始终由原高度环保持1.5m。旧V2保留诊断用途。新增原生 `px4_horizontal_enabled` 与 `visual_navigation_enabled` 均需显式选择，仓库默认关闭，任务默认关闭。旧模拟高度参数不参与实机高度。详见[水平任务合同](src/fc01_mission/README.md)。

当前完整软件链及故障回归见 [第5步报告](../docs/investigations/20261118_vision_step5/REPORT_zh.md)，水平控制合同见 [第4步报告](../docs/investigations/20261118_vision_step4/REPORT_zh.md)。MTF直连PX4、现有Position参数、串口和维护安装流程没有被本步修改。ARM64发行与现场标定／闭环性能仍需后续验证。

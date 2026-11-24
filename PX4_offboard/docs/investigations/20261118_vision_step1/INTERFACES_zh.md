# 第1步：两套代码的真实接口

## 基线与实际数据流

本轮开始于 `51acc74f9634e7beb631434d5035b6786abe8a2d`，分支 `readability/fc01-chinese-20261111`。`4c85a9e`（现用包源码）、`adc55d0`（原生候选）、`41e1532`（维护安装修复）均为祖先；从原生候选至发行源码、发行源码至本轮基线的 `native/src` / `native/config` 均无差异。完整身份见 [BASELINE.json](evidence/BASELINE.json)。

参考仓库固定为 [korbenname/fly@c4b32b9](https://github.com/korbenname/fly/tree/c4b32b92455a9c25fb1bce1262e8426f249f8521/target)。上游target的54份原文件经过Git blob身份核对；仅导入三个业务包，不导入第二份消息包。原文件身份在 [UPSTREAM.json](../../../target/UPSTREAM.json)。根目录PX4_offboard快照的六个关键文件与本轮基线blob全部相同，见 [REFERENCE_SNAPSHOT.json](evidence/REFERENCE_SNAPSHOT.json)。它是对照来源，不是另一套要启动的控制器。

现有飞行路径：MTF-02P直连PX4 → EKF2融合 → uXRCE/DDS → 原生SystemNode回调/反馈快照 → Runtime/Core高度闭环 → 最终发布边界 → PX4姿态控制。外部目标检测没有接管这条路径。

新增只读路径：V4L2图像或sensor_msgs/Image → VisionPipeline → fillTargetObservation → mission目标回调/制导输出 → fc01_target_interface_node → String接收证据。最后一步没有Runtime，也没有PX4发布器。后续真实状态桥、到高授权和外环仍未接入。

## 已复现的差异与本轮处理

| 原问题 | 本轮处理 |
|---|---|
| 同名/fc01/vision/state、同名VisionState，但target填的是相机原点的目标相对量，本地期待自机NED状态 | 新增TargetObservation类型及/fc01/vision/target_observation；两种实际状态明确隔离 |
| 上游VelocityCommand多wz却仍schema1，本地V1无wz | 原V1原字节保留；新增VelocityCommandV2、schema2与独立端点；不是把缺wz解释成0 |
| 航点默认/navigation_command与本地/fc01/navigation/command不匹配 | 生产launch对真实发布者/订阅者同步remap |
| target独立消息包会与native同名生成 | 只保留native/src/jikl_fc01_system；同一colcon树构建四包 |
| 原速度排队10、目标5 | 新指令pub/sub均keep-last1、volatile；目标输出/mission接收keep-last1 |

去注释后的上游VisionState/NavigationCommand/VisionHealth字段与原本地版本相同；相同字段不能证明语义相同。原生五种旧消息均未修改，SHA见BASELINE。新消息和真实ros2 interface show输出见 [evidence/ros-smoke](evidence/ros-smoke)。

## 当前端点合同

表中KL表示KEEP_LAST，所有新接口为VOLATILE。节点名称和默认topic可由参数/生产launch remap；不是固定串口或RC通道。

| 端点/类型 | 发布者 → 接收者 | 量、坐标与原点 | 时间/来源 | QoS与执行边界 |
|---|---|---|---|---|
| /fc01/vision/target_observation · TargetObservation | USB fc01_vision_node或vision_image_node → mission、只读C++接口 | 目标相对相机中心位置m、滤波变化率m/s，FRD前右下；名称不代表杆臂已移至机体原点 | USB为读帧完成的system时刻；ROS图像保留header；sequence为pipeline帧序号，source_id为相机节点配置 | BE/KL1；不是自机状态。ATTITUDE/ANGULAR_VELOCITY/QUALITY未估计，有效位清零 |
| /fc01/vision/state · 原VisionState | 原Odometry+Health适配器 → 原生视觉Inbox（开关启用时） | 实际自机fc01_vision_ned位置/速度、base_link_frd到世界的wxyz姿态、质量/协方差 | 成对同测量stamp；source/reset/sequence由原适配合同检查 | 原默认BE/KL5，可配置1..20；视觉false时原生不订阅 |
| 显式health_topic · 原VisionHealth（无默认输入话题） | 外部定位健康发布者 → 原Odometry适配器 | 同一Odometry的质量/跟踪/尺度/外参/对齐/时钟承诺，不含目标运动命令 | stamp须与对应Odometry完全相等；独立source/reset有效声明 | 原工具配置；不从“收到报文”猜测定位健康 |
| 显式input_topic · nav_msgs/Odometry（无默认输入话题） | 外部定位器 → 原适配器 | 实际自机；输入world/body约定显式配置，原工具转ENU/FLU至NED/FRD | 原始header.stamp，不能以回调接收时间续鲜 | 原适配器sensor QoS；本轮自机回归用隔离/fc01_demo |
| /fc01/vision/velocity_command_v2 · VelocityCommandV2 | mission → 只读接口 | 期望机体FRD vx/vy/vz，m/s；期望wz rad/s，向下轴正，俯视顺时针 | 生成时ROS时钟stamp；会话source/reset/sequence；不是测量戳 | RELIABLE/KL1；只读，没有原生速度执行器 |
| /fc01/vision/velocity_command · 原VelocityCommand | 旧调用方 → velocity_command_diagnostic.py | 期望FRD vx/vy/vz，无wz | 保留旧来源/周期/序号/时间合同 | 原默认BE/KL5可配置；诊断无控制权 |
| /fc01/navigation/command · 原NavigationCommand | mission → 只读接口；原生Inbox仅视觉开启时订阅 | ENU任务坐标转换NED绝对目标m、yaw rad；世界原点尚需与PX4对齐 | 生成时ROS时钟；mission source/reset/sequence；和自机测量身份不能混为一谈 | 新mission与接口RELIABLE/KL1；旧原生默认RELIABLE/KL5未改，当前视觉false |
| /fc01/interface/status · std_msgs/String | 只读接口 → 观察者/测试 | 最近三类收到的原字段、计数、control_authority=false和本地实际速度QoS | 每200ms诊断；缓存不表示仍有效，没有时效授信 | RELIABLE/KL1；禁止解析后/fmu端点及端点碰撞 |
| /fc01/vision/target_relative · PointStamped | vision_image_node → mission兼容路径 | 相机原点目标相对FRD m；无效NaN；USB节点不发布此替代端点 | 原图像stamp | BE/KL1；use_vision_state名称为兼容保留，其true现在消费TargetObservation |
| /fc01/vision/target_valid · Bool | vision_image_node → 观察者 | 本帧detected && position_valid，无自机定位意义 | Bool本身无stamp，不能单独用于授信 | BE/KL1 |
| /fc01/vision/debug_image · Image | 两相机节点 → 可视化 | 标注图像；生产默认关闭 | 原图像/采集stamp | RELIABLE/KL1 |
| /sim/platform_pose · PoseStamped | 尚缺的真实状态桥或sim → mission | 实际平台世界ENU位置、body FLU到世界xyzw姿态；不是目标位置 | header为源戳，mission另记monotonic接收时刻；独立时效缺口待后续处理 | mission现RELIABLE/KL10；没有生产PX4桥 |
| /sim/platform_velocity · Vector3Stamped | 同上 → mission | 实际平台世界ENU m/s | 当前上游未独立检查该源时效 | RELIABLE/KL10 |
| /sim/platform_angular_velocity · Vector3Stamped | 同上 → mission | 实际机体FLU rad/s；PX4 FRD需翻转y/z | 同上 | RELIABLE/KL10 |
| /sim/target_contact_force · Float32 | sim → mission | 接触力模长N，没有已证实硬件来源 | 无header | RELIABLE/KL10，历史模式不激活 |
| /sim/impact · Bool；/sim/reset_event · Empty | sim → mission | 碰撞/任务重生事件，没有已证实硬件来源 | 无header；不能视作人工RC授权 | RELIABLE/KL10 |
| /mission/state · String | mission → 观察者 | 上游任务阶段；不是原生飞行状态/到高授权 | 发布时状态，无测量有效性 | RELIABLE/KL10 |

原Odometry适配器强制显式提供input_topic和health_topic，空值拒绝启动；输出默认/fc01/vision/state。现场参数可能覆盖，不能据表推断板端存在这些发布者。

### 字段及有效性

VelocityCommandV2逐字段为schema_version=2、uint64 sequence、Time stamp、uint32 reset_counter、source_id、frame_id、valid、float64 vx/vy/vz/wz。valid=false是撤销意图；单纯收到消息不授权旧值继续生效。原V1布局不动，新旧不能在同一topic冒充。

NavigationCommand保留schema、sequence、stamp、reset、source、frame、valid、mode、position_valid/position[3]、yaw_valid/yaw、sticks_valid/sticks[4]及原常量。NED目标不是body相对位移。

TargetObservation包含schema1、sequence/stamp/reset、frame/body/source/angular_velocity_source、initialized/tracking/calibration_valid、valid_fields、position[3]/velocity[3]/attitude[4]/angular_velocity[3]、pose/twist_covariance[36]及quality。矩阵行优先xyz后角分量；未声明的位不能用默认0推断已测得。当前pipeline并不估计目标姿态、角速度或经标定质量。相机内参“数值可用”也不能等同现场标定已确认。图像节点安装俯仰旋转与速度一致性仍须后续审查。

VisionState是实际自机状态，原有效位与质量要求不变；VisionHealth是对同条Odometry的显式承诺，不是目标置信度替身。时间域：图像/命令用ROS/system测量或生成时刻；本机超时用monotonic；PX4 ULog boot微秒、DDS同步微秒不能直接与二者相减。收到重复帧、未来帧或旧帧的规则尚未在上游mission补齐。

## 原生及PX4接收/发布边界

`fc01_system_node.cpp::SystemNode` 创建PX4订阅；`snapshot/populate_attitude_feedback`形成有效/独占/源时间事实；`tick`经Runtime与Core再走既有最终发布检查。`visual_navigation_enabled=false`时不创建原VisionState/NavigationCommand订阅、不读取视觉Inbox。本轮未改这些代码。

| 方向 | 端点及px4_msgs类型 | 用途及坐标/时间 | 原生QoS |
|---|---|---|---|
| PX4→FC01 | /fmu/out/vehicle_local_position · VehicleLocalPosition | EKF实际NED z/vz、z_deriv、reset、valid、dist_bottom；m/m/s、向下正；相对高度z0-z；保留timestamp和timestamp_sample | sensor BE/volatile/KL10 |
| PX4→FC01 | /fmu/out/vehicle_attitude · VehicleAttitude | 实际FRD→NED Hamilton wxyz、reset/采样时间 | 同上 |
| PX4→FC01 | /fmu/out/vehicle_status_v1 · VehicleStatus | armed、nav_state、preflight、failsafe等真实PX4状态 | 同上 |
| PX4→FC01 | /fmu/out/vehicle_land_detected · VehicleLandDetected | landed/at_rest新地面样本 | 同上 |
| PX4→FC01 | /fmu/out/manual_control_setpoint · ManualControlSetpoint；manual_control_switches · ManualControlSwitches | 实际RC来源/轴/开关/模式槽；不按提示字符串猜物理通道 | 同上 |
| PX4→FC01 | /fmu/out/battery_status · BatteryStatus；failsafe_flags · FailsafeFlags；vehicle_command_ack · VehicleCommandAck | 电池/失效/命令回执，不能由视觉代造 | 同上 |
| FC01→PX4 | /fmu/in/offboard_control_mode · OffboardControlMode | 已提交心跳；现姿态/推力架构，连续发送要求不变 | 原生RELIABLE/volatile/KL10，PX4兼容订阅 |
| FC01→PX4 | /fmu/in/vehicle_attitude_setpoint · VehicleAttitudeSetpoint | 实际姿态目标和FRD thrust_body，z为负幅值；原边界独占 | 同上 |
| FC01→PX4 | /fmu/in/vehicle_command · VehicleCommand | 原有受限命令；不新增程序解锁 | 同上 |
| FC01→PX4 | /fmu/in/trajectory_setpoint · TrajectorySetpoint | 仅原其他profile的路线；当前姿态高度路线不用它替换控制算法 | 同上 |

本机选定px4_msgs SDK与既有PX4源码13种所用消息的去注释字段全部匹配，VehicleStatus版本1，其余有版本字段者0；OffboardControlMode/FailsafeFlags/ManualControlSwitches无MESSAGE_VERSION，模板按0处理。固件DDS模板utilities.hpp为非零版本加_v后缀，所以源码yaml的vehicle_status对应ROS vehicle_status_v1。证据 [PX4_SOURCE_SDK.json](evidence/PX4_SOURCE_SDK.json)。

既有PX4源码HEAD为07983bfb，DDS topics及client有先前未提交修改；本轮未更改。不能推断该dirty树已刷入实际飞控。本轮无香橙派连接，未执行板端ros2 topic info或读取最新runtime。用户已成功安装20261117_stage3_fix1、报告1.5m，这是会话事实；11月14日runtime历史原件保留在前一阶段，不冒充最新读回。真实固件图/版本仍属待现场核验。

官方依据已实际取回且摘要在evidence/sources.json：[MTF-02P](https://micoair.com/optical_range_sensor_mtf-02p/)、[PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)、[uXRCE DDS](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)。文档支持融合/DDS和心跳合同，不证明用户当前板端已满足它们。

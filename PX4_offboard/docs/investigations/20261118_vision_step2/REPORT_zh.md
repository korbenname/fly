# 第 2 步：相机目标与 PX4 自机反馈

已完成源码实现、离线复现、真实 Humble 传感器链验证、独立复核和本地源码提交。没有连接飞控或香橙派，没有部署、推送或修改已飞稳的 MTF/PX4 参数。本步骤不授予视觉飞行权限。

## 版本与范围

- 分支：`readability/fc01-chinese-20261111`。
- 开始提交：`9e9fb3ef4c4e71d8b1b646bfe0d125a0e71c59a8`（第 1 步成果）。
- 本步源码提交：`f27c733b3682b14e373beeccfa858beafb5049fb`。
- 实机已安装版本的发行源码 `4c85a9e7e0c66193104164f03f4bfe28de709b9d`、原生候选 `adc55d0b7c67e80bf9d77b1082208c898f95b756`、维护安装器 `41e1532e73d36005a80cd134807a70b0c0263675` 均为当前历史祖先。本机未读取到更新的实机 runtime.yaml，不能声称开发机配置就是当前板端配置。
- 本步原生 `src/`、`include/`、`native/config/`、服务和安装代码与开始基线无差异；原有 7 个消息逐字节不变。原生 CMake 仅新增 `PlatformState` 消息生成项。证据：`evidence/no-flight-diff.json`。
- 现场人工 RC 解锁、1.5 m 定高、推力限制、接管、Kill、Failsafe 和安全事件处理没有被本步改动。原生视觉仍关闭。任务层 `control_enabled=false`，只读接收器不执行 V2 意图。
- 保留既有非本轮未跟踪内容：`tests/test_micovio_clock_candidate.py`、`tests/test_micovio_uart0_reader.py`、`tools/`；未纳入提交。

## 真实数据路径与改动位置

| 路径/函数 | 本步行为 |
|---|---|
| `platform_feedback.py::PlatformFeedback` | 三路真实 PX4 DDS 接收校验，保留独立测量时刻、有效位、来源和 reset；NED→ENU、FRD→FLU、wxyz→xyzw；缺字段为 NaN/无效 |
| `px4_state_bridge.py::PX4StateBridge` | 新只读 ROS 桥，无 `/fmu` 发布器；生产默认订阅 PX4 融合状态，按 50 Hz 发布有效性快照 |
| `PlatformState.msg` | 原子自机状态，位置/速度/姿态/角速度四个 mask；保留原 DDS timestamp/sample、单调接收时刻及唯一图端点 GID |
| `sensor_contract.py::PlatformHistory` | 每字段双时效、有界历史、图像源时刻插值；原点/来源/reset 变化清历史；非法快照先消耗水位再统一校验和提交 |
| `mission_node.py::_on_platform_state/_on_state/_consume_pending/_ingest` | 生产不订阅模拟平台或旧 relative 回退入口；校验目标 mask/跟踪/标定/有限值/来源；连续 3 个新帧才锁定；图像时刻实际自机状态对齐 |
| `mission_node.py::_clear_target/_update_mission/_publish_*` | 失效撤销估计与前馈；冻结终端/strike/push/retreat 需求清除并锁存，反馈恢复不自动续跑旧任务；默认关闭输出零值且 valid=false |
| `camera_source.cpp` | 可配置 device_path 或旧编号；成功 read 的应用序号和读取完成戳保持到下游；记录 read 耗时、重连周期；线程异常可观测 |
| `vision_node.cpp/vision_image_node.cpp/frame_guard.hpp` | headless、独立来源会话；源/单调接收时效；重复/旧/未来/空图/断流/尺寸不符失效；Image 无序号，严格推进的原始 stamp 对应应用序号 |
| `vision_state_adapter.cpp/target_estimator.cpp` | 安装旋转同时作用于位置、速度、协方差；首帧不伪造速度有效；标定/模型/外参/延迟/零畸变适用性必须显式确认 |
| `robot.launch.py/mission.launch.py` 与两边 YAML | 生产启动真实桥且不启动第二原生控制节点；统一平台/目标/指令 remap；传感器与任务状态输出缺项 |

MTF-02P 继续沿现有接线进入 PX4，由 EKF2 融合。相机目标量不能冒充自机里程计。旧自机 `VisionState`/Odometry 适配器仍在独立话题，真实 ROS 回归通过。

## 坐标、原点、时间和有效性

- 平台位置/速度是 PX4 **原 local 原点**下的 ENU，米/米每秒；不减第一条位置，也不把 ENU z 当离地高度。原生起飞零点仍由原有 z0 处理。
- PX4 `q` 是 FRD→NED 的 Hamilton wxyz。桥输出 FLU→ENU 的 xyzw，使用完整基变换，不是只换数组顺序；角速度由 FRD `(x,y,z)` 变为 FLU `(x,-y,-z)`，单位 rad/s。
- 目标是相机光心处的相对量。检测器先从 optical 右/下/前换到 camera-FRD；安装旋转转到 body-FRD 方向。mission 再转 FLU，并**仅一次**加入配置的 FLU 杆臂，使用图像时刻的真实位置/姿态得到绝对 ENU 目标。
- 有界字段历史保留最多 160 点；用真实样本夹住图像时刻后线性插值位置/速度、slerp 姿态。默认字段间隔预算 80 ms，不用当前姿态或小角度猜测去外推旧图像。待对齐目标只有 1 个，不能积压图像任务。
- 桥默认源及单调进展 TTL 为 250 ms，mission 为 300 ms；未来容差 20 ms。桥快照 stamp 是观察时刻，各 `*_stamp_ns` 才是原始同步测量时刻。重复测量不更新 TTL。单调时刻仅允许同一主机比较。
- 固定 PX4 v1.16 的 `Tools/msg/templates/ucdr/msg.h.em:138–144` 对 timestamp 和 timestamp_sample 同时加同步 offset；桥要求与 Agent/ROS 同步时域。无同步数据明确失效，不猜测时钟偏移。
- Image 保留 header stamp，不用接收时刻替代。USB stamp 是 `cap.read()` 完成时刻，seq 是应用成功读帧次数；没有声称取得硬件曝光戳。状态显示 `stamp_kind`、`read_seconds`、`exposure_timestamp_measured=false`。实际曝光/缓冲延迟需要现场证据，配置上界不会改写原始时间。
- 新帧但 tracking=false、缺 POSITION、标定未确认、NaN/非法协方差都会撤销目标。同戳重标“好”不能复活。会话重启、源回退、坐标重置后重新锁定，已退休来源不能夺回。
- 航点原点必须显式绑定当前 `origin_id`；未知标定、杆臂、模型、延迟和航点原点在状态页列出。原点重置后不能沿用旧确认。
- 当前几何估计仅支持针孔/零畸变模型。`image_rectified_confirmed=false` 默认关闭；现场需提供已矫正图像及对应 K/尺寸，或可查的模型适用证据。CameraInfo 非零/NaN D、非法 K、尺寸/主点改变或未知模型会锁存失效，随后旧“好标定”不会自动恢复。没有实现 raw 镜头自动畸变校正。

## 实际复现并修复的问题

| 等级 | 缺陷及限定影响 | 复现/回归证据 |
|---|---|---|
| P1（未来启用任务前） | 旧 mission 把任意旧图年龄压为 0.15 s、未来图压为 0；不检查 tracking/calibration/序号，重复帧刷新接收时刻 | `mission-before.log`、`bridge-before.log`；当前纯模块及真实完整链 |
| P1（未来启用任务前） | 缺实际速度/角速度默认零，分离回调无独立时效；失效自机状态可被当成静止 | 实际模块和 bridge ROS 独立过期场景 |
| P1（未来启用任务前） | R1：目标/原点撤销后保留旧 terminal 冻结速度，平台恢复后可继续 valid 非零需求 | `r1-before.json`；`test_mission_cached_demand`、最终发布方法检查、真实 ROS 完整链第 13 场景 |
| P2 | 图像 pipeline 相同/回退 stamp 仍推进新有效序号，首帧速度冒充可用 | `camera-before.cpp/.log`；C++ contract、实际图像 ROS |
| P2 | R2：相同 K 的 CameraInfo 非零畸变被忽略，仍声称标定有效 | `calibration-before/RESULT.json`；当前 8 项实际 ROS 标定检查 |
| P2 | R3：ready=true 的 NaN 快照拒绝后，同戳修复载荷可复活 | `r3-before.json` 对比 `r3-after.json`；原子提交/部分恢复回归 |

这些评级针对后续接入飞行时的潜在影响。本步默认任务关闭，未将其作为已部署飞控实际失效证据。没有发现本步新增 P0；这不是全工程无 P0/P1 的安全声明。

## 最终源码的实际验证

环境：Ubuntu 22.04.5 **x86_64**，ROS 2 Humble，`/usr/bin/python3` 3.10，既有 SDK px4_msgs。真实 ROS 图都使用显式隔离域和 `ROS_LOCALHOST_ONLY=1`；数据为合成 PX4 消息、合成图像，无飞控控制节点。

| 检查 | 结果与证据 |
|---|---|
| 统一 colcon | 4 包 PASS，最终 13.6 s，`build-final.log` |
| 相关 unittest | **41/41 PASS**：平台 16、mission 传感器 12、冻结需求 2、ROS/发行合同 11；`unit-static-final.log` |
| 原生主机 C++ | **85 个可执行程序 PASS**；`native-cpp-final.log`，原生源码未改 |
| 视觉 CTest | **2/2 PASS**；旧 core + 新 contract；`vision-ctest-final.log` |
| 8 个生成消息 | MSG 与安装字节一致、C/Python typesupport 加载、字段/默认值/CDR 往返 PASS；`generated-interface.json` |
| 实际桥 DDS | **8 组有名检查 PASS**：独立有效位/过期/reset/退出，无 `/fmu/in` 发布；`bridge-smoke.json` |
| 相机→实际 mission 完整链 | **13 组有名检查 PASS**：三新帧锁定、重复/旧/空图/断流、yaw90、倾斜、左右、杆臂、NaN、标定、独立速度/omega/local失效、两种reset、来源重启、冻结需求恢复后不复活；`mission/RESULT.json` |
| 相机标定实际 ROS | **8 组有名检查 PASS**；`calibration/RESULT.json` |
| 原第 1 步接口/生产 launch | PASS；真实四节点启动/退出、四话题 remap、无模拟平台订阅、无 `/fmu` 发布、默认撤销、V1/V2隔离；`interfaces/RESULT.json` |
| 旧 V1 / 自机 Odometry | 原真实 ROS 回归各 PASS；`legacy-v1.json`、`self-odometry.json` |
| 缺相机生命周期 | 3 次启动/退出 exit0，5 项标定缺失可见、全部无效、会话不同；`camera-missing-final.log` |
| 独立复核 | 1000 随机姿态最大矩阵误差 8.88e-16；最终相关 **30/30** 重跑（是 41 的子集，不叠加）；R1/R2/R3 限定复验通过，`final-independent-review.md` |
| 静态/安装身份 | Python compile、git diff --check PASS；Python 安装文件与已提交源码逐字节一致，`TESTED_INSTALL.json`；改动源码 SHA 见 `TESTED_SOURCES.json` |

有名场景、生成消息和 executable 数量分别列出，不把重复验证合成一个“总测试数”。没有重做与本步无关的全工程审查或宣称 ARM64/实机/ASan 已通过。

### 真实失败和修正记录

- C++ 首轮构建遇到 `rclcpp::Time(0, enum)` 重载歧义，已改 int64；原失败保留 `camera-build-failed-overload.log`。
- 大图 best-effort 单帧夹具曾接收超时，未据此指认相机算法故障；改为 160×120 合成夹具并等待实际端点，失败保留 `image-dds-first-timeout.log`。不能推导实机大图/带宽性能通过。
- root 完整链首轮订阅 V2 的规范名，而直接构造 mission 沿用旧 `/visioncontrol` 参数，最终断言未收到消息；已显式给夹具传正确输出参数，保留 `mission-fixture-topic-mismatch.log`。
- 开发中单调时刻浮点转换产生负亚纳秒差，改为整数纳秒相减并重跑边界测试；没有保存该首轮独立日志，不捏造证据文件。
- 独立复核确实发现 R1/R2/R3，先保存失败再修复。早期 review 的“待修复”文字保留历史身份，以最终复核为关闭依据。

## EKF / Position 只读核对结果

详见 `EKF_BASELINE_zh.md` 及 `evidence/EKF_AUDIT.json`。31 个条目中实际解析 25 份 ULog，6 份旧桌面路径已不可读；没有删除旧证据。

新发现的硬件 `log_82_UnknownDate.ulg` SHA256 `a5f07db65ea44433af07fd20fb9bd47880d23c325802a73c715de9a99dac6a8e`：仅 2.195 s，Position 约 1.870 s，land detector 始终接地。它不是用户稳定空中 Position 的性能对照。该窗口 local z 约 +2.22 m，而测距约 0.094 m，印证 local 原点不能冒充地面零点。

该日志的 HGT_REF=2，观测到 range/baro 高度及 EV 位置/航向融合，窗口内 optical flow active=false；不能由此断言用户当前 MTF 配置异常或改动已飞稳参数。不同文件/UUID 的参数和 boot 时间没有拼接成同一次飞行。

固定 PX4 源码的 DDS 清单仍注释 `vehicle_angular_velocity`。ULog 内部有角速度不证明 DDS 出口存在。默认要求 omega 时桥会明确报缺；本步不修改固件清单，不能用零代替。

## 剩余问题及交给第 3 步

1. **P1，视觉飞行准入尚未完成。** 上游任务含 2.78 m、冲撞/顶住及不同高度计划，仍默认关闭。本步没有把它接到原生执行或把模拟任务当作用户 1.5 m 流程。第 3 步应在真实原生到高/RC 门后接水平意图，并统一唯一高度目标；不能直接把 control_enabled 改 true 就试飞。
2. **P1，旧任务恢复边界。** 冻结阶段失效已锁存；生产显式重新准备/任务重启的协调入口尚待后续步骤。不能绕过原生 RC/安全准入解除。
3. **P2，现场角速度/同步/标定待证。** 当前 PX4 DDS angular_velocity、Agent 同步域、当前服务/runtime/进程身份、相机设备和标定均未板端读取。
4. **P2，资源边界。** OpenCV/驱动 read 可能阻塞，stop/join 没有已证明的硬截止；视觉进程与原生控制分开，尚需实机断连和退出测量。
5. **P2，采集与图像模型。** USB 无硬件曝光戳、不能从应用 seq 证明没有重复传感器曝光；未知缓冲延迟/畸变必须保持未确认。完整 raw 畸变校正未实现。
6. **P3，来源和兼容结构债。** Humble 图中唯一 GID 是来源独占图证据，非逐包认证；兼容 PoseStamped 的单戳不能表示各字段时差，生产必须继续使用 PlatformState。旧 simulation/contact/push 逻辑尚待后续收敛。

需要现场只读取证：当前稳定 Position 实飞完整 ULog/参数及 UUID、实际 DDS topic/type/QoS/采样推进与时间同步、相机稳定设备名、图像是否已矫正、内参/模型/安装旋转/杆臂及可查 ID、采集延迟测量、当前 runtime.yaml/服务入口。缺这些不影响本次离线结果，但不能给出实机定位精度、曝光延迟、水平漂移或可飞结论。

## 回退

本步是独立源码提交 `f27c733b3682b14e373beeccfa858beafb5049fb`。后续尚无依赖修改时，可在本分支用 `git revert f27c733b3682b14e373beeccfa858beafb5049fb` 保留历史回退；先保留工作区已有用户文件。部署版本和实机安装目录本步未更改，因此无设备切换/回滚操作。

# FC01视觉接入第1步交付

本轮完成“接口核对与统一消息构建”。代码提交 **619736483a69df691eca49a5fbdb3135082fd8ed**，开始基线51acc74f9634e7beb631434d5035b6786abe8a2d。继续在readability/fc01-chinese-20261111本地开发，未推送、未部署、未改main或PX4参数。本轮没有开始用户尚未入队的第2～6步。

## 实际修改

逐文件清单见 [CHANGED_FILES.txt](CHANGED_FILES.txt)，逐源码SHA及测试提交身份见 [TESTED_SOURCES.json](evidence/TESTED_SOURCES.json)。

| 范围 | 修改与原因 |
|---|---|
| native/msg的两个新文件 | TargetObservation隔离相对目标/实际自机；VelocityCommandV2带wz、schema2，原V1保持兼容 |
| native/CMakeLists.txt、package.xml | 唯一七消息生成与C++/Python类型支持；新增std_msgs及只读接收器编译安装 |
| native/src/fc01_target_interface_node.cpp | 有限队列接收三类意图/观测，输出字段证据；无Runtime、无PX4控制发布器 |
| target三个包 | 从上游固定提交导入视觉、mission、bringup；更换独立消息类型、指令QoS、同步真实remap、默认任务关闭；保留原业务数学，未以此接管飞行 |
| target/build.sh、README、UPSTREAM、.gitignore | 同一树构建四包；记录来源、准确接口/限制；不重复构建消息包、不纳入构建产物 |
| tests/ros_target_interface_smoke.py | 真实Python→C++字段、V2正负wz、C++图像链、实际mission输出、生产launch映射/缺相机/退出及remap边界 |
| 三份原测试 | 七消息生成默认值按定义解析；既有构建接线/载荷计数随新增消息更新 |

完整接口表、字段/坐标/原点/时钟/来源/QoS及文件函数见 [INTERFACES_zh.md](INTERFACES_zh.md)。

## 确认与修复的缺陷

| 等级 | 复现及修复 | 证据 |
|---|---|---|
| P2（未来视觉集成阻断） | 上游与本地VelocityCommand同类型不同字段；多wz且均称schema1。拆为独立V2，保留V1。DDS真实拒绝旧类型冒充V2 | MESSAGE_DIFF.json、ros-smoke/RESULT.json、interface-v2.txt |
| P2（未来视觉集成阻断） | 目标相对量复用自机VisionState入口、航点默认topic不匹配、重复消息包。分类型/端点，统一安装树并真实remap | UPSTREAM_AUDIT、接口表、生成身份、生产topic图 |
| P2（本轮新增，只读节点边界） | 独立审查发现二次remap可绕过/fmu限制。消息类型是String，不匹配PX4 VehicleCommand，但破坏显式端点边界。改为只解析一次并拒绝解析后冲突 | INDEPENDENT_REVIEW中的修前/修后真实Humble 3场景；smoke自动回归 |
| P2（导入上游生命周期） | 生产launch正常SIGINT关闭Context后，mission再次shutdown报rcl_shutdown already called。捕获正常ExternalShutdownException、节点销毁后try_shutdown，不吞业务异常 | launch-double-shutdown-defect.log；最终production-launch.log三节点cleanly退出 |
| P3 | 上游注释/日志仍写VisionState或depth5、默认启用等，与改后事实冲突；同步中文说明，删去会误导真机操作的bringup旧说明 | 当前源码/README |

没有在本步骤发现并证实新增P0/P1飞控缺陷。未对整个导入mission作完整安全认证；未完成的时效、坐标和仲裁问题见交接清单，不因接通接口而降级。

## 真实验证结果

环境Ubuntu22.04.5 x86_64、ROS2 Humble、/usr/bin/python3.10、FastDDS。本机已有匹配px4_msgs SDK；不是ARM64或用户相机实测。

| 验证单元 | 实际结果 | 原始证据 |
|---|---|---|
| 原生+视觉+mission+bringup统一构建 | 4包PASS；最终源码安装树 | unified-build.log；native-build.log为最初独立原生构建 |
| 生成身份/CDR/typesupport | 7消息PASS，源/安装MSG、C/C++/Python类型支持、默认值/常量/序列化 | generated-interface.json |
| 新ROS smoke | PASS：三端点字段、正负/零wz、valid=false、NaN诊断为null、V1隔离；实际mission ENU→FRD、wz双符号；实际C++黑图/红圆；生产相机+mission+接收器自定义remap、无自机topic污染、无控制节点；缺相机正常待机、清洁退出 | interface-tests.log、ros-smoke/RESULT.json及ros2输出 |
| 旧V1真实DDS | PASS，11项有名检查，不重复统计成11个unittest | legacy-velocity.json、legacy.log |
| 自机Odometry真实DDS | PASS，7项有名检查，含转换/来源冲突撤销/资源预算/退出，无fmu发布 | self-vision.json、self-vision.log |
| 针对性Python单元 | 11/11 PASS | static-tests.log |
| C++视觉core CTest | 1/1 PASS，内部4组有意义场景 | vision-ctest.log |
| 独立remap复验 | 3场景PASS | INDEPENDENT_REVIEW_zh.md |
| Python/Shell语法、差异空白 | PASS | 复验命令文档；仅语法检查不充当逻辑测试 |
| 既有PX4源与SDK字段比较 | 所用13种去注释字段一致；不是当前实机固件读回 | PX4_SOURCE_SDK.json |

上述计数单位不同且存在重叠，不合成一个夸大的“总通过数”。ROS图远端depth显示UNKNOWN/0；接收器自身get_actual_qos确认1/KeepLast/volatile。没有假称能从远端图读出队列深度。

保留失败：一次Humble QoS枚举编译错误（已修）、首次best-effort图像单发/发现窗口超时（夹具改为真实图像流并等待端点）、错误要求无平台数据也必须有mission命令导致测试超时（源码正常提前返回，夹具改为只审查未发布有效命令）、测试先向整个launch进程组发SIGINT造成双信号（改为只向launch转发），以及真实重复shutdown缺陷。均有原日志，最终检查没有跳过这些被修复的场景。

ASan/UBSan、完整动力学仿真、真实视觉性能、ARM64新构建、本轮整工程全量回归未执行：本步骤仅统一接口，不变更既有飞控逻辑；不能把历史通过记录当作新功能实机验收。本轮未重打包现用ARM64包。

## 原飞行路线与现用发行身份

本轮60份既有控制头文件/主节点/配置/原消息/服务安装文件与基线逐字节一致，明细BASELINE.json。1.5m、人工RC解锁、Offboard确认、原推力参数、接管/Kill/Failsafe/最终发布时序均无代码改动。本步骤没有执行新的飞行状态机差分仿真；行为保持的依据是相关源码/配置未变与新接收器不连Runtime，不声称进行了飞行性能验证。

视觉开关仍false；相机目前**不能辅助飞行**。上游mission默认control_enabled=false，没有实际平台反馈时不发布指令；即使收到V2意图，当前原生控制器也没有V2执行订阅。本轮没有把2.78m上游任务值写进原生runtime，也没有把目标相对z用于定高。

现用包仍为FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz，6498104字节、SHA256 `5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373`；维护安装器41e1532不变。本机origin/main缓存377a945d9cd1147baad8871cd5387b9e63cd774d，未接触远端；本地没有main分支，不创建或修改它。

本机无法确认用户当前板端分支、DDS图与runtime读回；历史11月14日runtime与用户成功安装输出明确分开。具体配置仍由后续拆桨/未解锁现场核对。用户原有tools/和两个MicoVIO未跟踪测试保留，不纳入提交。

## 后续交接

见 [NEXT_STEP_HANDOFF_zh.md](NEXT_STEP_HANDOFF_zh.md)。仅完成用户第1步，按用户指示本地提交后结束，由队列提供下一步正文。

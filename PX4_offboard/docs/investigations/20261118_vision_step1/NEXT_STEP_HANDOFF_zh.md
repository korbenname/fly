# 给下一项队列的交接

本轮只完成第1步。实现提交619736483a69df691eca49a5fbdb3135082fd8ed，接口/证据在本目录。后续准确范围以用户下一条排队任务为准，不把本清单当作已获得飞行授权。

## 可直接复用的入口

- 唯一消息源：native/src/jikl_fc01_system/msg。保留原五种消息，新增TargetObservation、VelocityCommandV2。
- 统一构建：target/build.sh。显式FC01_PX4_PREFIX，本机实际使用/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs。
- 生产接口启动：target/src/fc01_bringup/launch/robot.launch.py；默认任务false，start_camera可关闭，vision_config可换现场配置。原飞控systemd不由这个launch启动。
- 接口接收：native/src/jikl_fc01_system/src/fc01_target_interface_node.cpp。只读三个端点，速度RELIABLE/volatile/KL1；不能将它误认为V2飞行执行器。
- 验证命令：[VALIDATION_COMMANDS_zh.md](VALIDATION_COMMANDS_zh.md)。已构建树$HOME/.cache/fc01-vision-step1-20261117/install。

## 下一步必须沿源码核实的未完成逻辑

| 风险/边界 | 文件和函数 | 当前证据与建议验证 |
|---|---|---|
| 重复帧会给目标续鲜、源戳年龄截断 | mission_node.py::_message_age/_on_state/_ingest，vision_node.cpp::publishOnce | 目前可接受相同stamp/seq、旧数据被截在0..0.15s，未来时刻也未明确拒绝。构造重复/乱序/未来/reset/source变化真实ROS回归，再修时效；不以发布频率代替测量进展 |
| 平台状态尚无PX4生产桥 | mission_node.py::__init__/_on_pose/_on_velocity/_on_omega | 仍订阅/sim/platform_*；pose接收时间、速度与角速度没有完整同周期/独立源时效合同，不能用于飞行。桥必须区分PX4实际状态与目标，处理ENU/NED、FLU/FRD及时间同步 |
| 相机内外参/相对速度尚未现场确认 | vision_state_adapter.cpp::fillTargetObservation，vision_image_node.cpp::onImage，camera_source.cpp::threadLoop | 采集完成时刻不是曝光时刻。相机原点不是body原点；图像节点只旋转position的俯仰补偿与velocity可能不一致。内参数值可用不等于标定有证据；速度是相对变化率非自机实际速度 |
| 未接入原生到高与控制仲裁 | SystemNode::snapshot/tick、Runtime/Core及mission::_publish_navigation/_publish_command | 当前原生视觉false，没有V2执行订阅；上游valid=false并不会自动把控制从航点切到速度。本轮接收成功不证明权限切换正确 |
| 两种高度语义不同 | native/config/flight_test.yaml；mission/config/mission.yaml | 原生rc_target_height1.5m相对本次z0，h=z0-z；导入mission还保留2.78m上游世界绝对高度。未来必须保持用户原1.5m路线，并禁止上层正常视觉跟踪改垂向目标 |
| 上游特殊模式不适于直接激活 | mission_node.py::_update_mission等 | push/impact历史逻辑、模拟接触力/impact/reset仍存在；本轮默认control_enabled=false。不允许把消息接通当作可运行这些模式的依据 |
| 可选消息导入仍有宽泛except | mission_node.py顶部、main | 消息依赖已唯一构建并真实导入；导入失败仍可能退化为无输出警告。构造实际依赖失效再决定最小显式失败方式，不能通过继续吞异常掩盖 |
| 相机资源异常/构造失败待深入 | vision_node.cpp构造/startWorker、camera_source.cpp线程/VideoCapture | 缺设备等待和正常退出已真实通过；构造中线程创建失败、cap.read阻塞等没有故障注入结论，留资源回归，不宣称均安全 |

## 既有控制与发行边界

原生控制头文件/主节点/飞行配置与本轮基线逐字节一致。原生候选adc55d0、包源码4c85a9e和维护安装修复41e1532的关系已核对。当前用户已安装20261117_stage3_fix1，维护切换已由用户反馈成功；本轮没有板端连接。11月14日runtime和11月17日VM1.23m继承测试是不同来源，不能拿VM数值覆盖用户1.5m目标。

板端尚缺：当前fc01-navigation.service完整入口/实际可执行路径、runtime.yaml完整读回、px4固件build身份及dds graph、真实相机设备和内外参/曝光时间、目标尺寸和来源标识、同次任务日志。现有PX4源码07983bfb含先前dirty DDS改动，13种SDK字段虽一致，不能证明实际飞控已刷入。无需因此阻断可离线完成的后续工作。

用户原有未跟踪tools/、test_micovio_clock_candidate.py、test_micovio_uart0_reader.py没有被纳入本轮提交。不得覆盖。未推送、未改main、未修改UART/MTF/PX4参数、未部署或运行动力程序。

## 回退本轮源码

若后续尚未依赖本轮，可用git revert 619736483a69df691eca49a5fbdb3135082fd8ed生成显式回退提交，保留全部历史；操作前核对用户工作区，勿reset/clean。文档证据独立提交，不影响飞行程序。用户板端现用包没有被本轮改动，不需因本轮接口开发切换现用服务。

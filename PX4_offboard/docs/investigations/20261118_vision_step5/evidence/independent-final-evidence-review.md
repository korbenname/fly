# Step5 最终验证证据独立复核

审查对象：当前未提交 Step5 测试入口及最终 atomic 结果。只读审查，没有改生产代码、运行真实设备、提交或部署。

## 结论

在本次测试声明的 x86 Humble / localhost 合成相机和 PX4 peer 范围内，未发现会把失败路径标成完整闭环成功的 P1。四条 pipeline 与十一条故障矩阵结果文件均为 PASS，harness、监督器夹具、mission 的记录 SHA 与当前文件一致。不能将这些结果表述为 PX4 固件采纳或真实相机/飞机表现。

## 逐项证据核对

- pipeline 实际启动安装树 C++ vision_image_node、实际 px4_state_bridge、实际 mission 和 standby.supervise。监督器夹具只替代 ARM/SDK/UART preflight、全局旧控制器探测和 AgentTransport，原锁、配置写入、spawn、日志、IPC monitor、状态写入、安全停止和清理仍走生产函数。它没有运行 systemd，不能算板端安装/服务验收。
- 合成图像经 sensor_msgs/Image typesupport 与真实 C++ 检测/估计得到 TargetObservation；任务水平封套走真实 ROS；最终证据由 /fmu/in/vehicle_attitude_setpoint、offboard_control_mode、vehicle_command 的实际 typed 接收回调记录，非本地规划输出。接收者是合成 peer，既没有 PX4 固件，也没有 Agent/UART。
- 人工 RC 意图、实际 Armed 和 Offboard 状态分开提交；程序 TAKEOFF 明确拒绝。到高在原生反馈中核对 navigation_allowed，任务输入不能提前授权。
- 从航点到目标右侧正 roll、目标左侧负 roll 的断言来自实际目标 topic。目标切换像素、mission envelope、原生 visual_reason 及输出均有原始日志可对齐；XY 模型固定，不能声称真实水平闭环收敛或漂移为零。
- 正常、无视觉、目标丢失三条 pipeline 确认收到 Disarm 400 前高度与速度已在地面门以内，ACK 单独不能结束 armed；之后才注入独立 Disarmed 状态，再确认 176 释放和新的 READY。没有提前把模型高度跳到零。十一条矩阵中十条真正水平故障同样连续积分下降到地面，关闭/撤销水平通道不重建任务；preheight_flood 是洪泛安全+正常 LAND，不应称第十一条故障下降。
- 正常/丢失流程均真实出现 DESCENT_TIMEOUT -> RECOVERY_DESCENT 和地面释放后的 OPERATOR_TAKEOVER_LATCHED，最终 READY 不代表下降没有超时或没有锁存。现有报告已指出。
- 全 DDS loss 分支停止全部反馈，原生先撤销水平再进 SAFETY，排空后检查目标与心跳停止。之后明确标记 test_only_external_ground_disarm_after_feedback_loss；该分支不具有受控下降/自动落地证据，不能计入完整故障落地。
- 故障矩阵核对 target/heartbeat 成对、时间单调、正常飞行心跳预算、控制输入 SHA 未变化及阶段顺序。重复、乱序、多 writer、命令超时确实经真实 native subscriber 后触发撤销。
- 原生 PID 来自监督器真实 IPC 通道的绑定，正常 stop 要求监督器退出0且原生 /proc 不存在；finally 的强杀清理不能作为 PASS，若出现原生残留会把结果改 FAIL。全程只复用监督器唯一 IPC，没有第二条控制器连接。

## P2 验证范围与报告限定（不要求放宽保护）

1. pipeline 的 single_native_control_publisher 检查只在任务结束、停止服务前取一次 ROS graph 快照；它证明该时刻每个 /fmu/in 控制端点仅一个 fc01_system writer。不能据此声称连续每一周期监测了无短暂第二 writer。故障矩阵另测的是水平输入 writer 冲突，二者不要混称。软件受控启动了一个 native，但这不是板端全系统重复启动证明。
2. pipeline 的 heartbeat 最大间隔是测量项，没有以预算作为此脚本 PASS 门。实际预算断言在原生矩阵。全 DDS loss 指标跨故意停发与重新准备，5.36s 不是正常飞行心跳抖动；应分段解释。
3. 有效新帧率去重键是 source/reset/sequence，时延是合成图像 host stamp 到观察回调的时间，不含 V4L2 曝光、USB传输或真实相机缓存；CPU是各进程全场景平均，非 active-only 或峰值。仓库640x480/120fps参数在合成入口复用，但不能当OrangePi实测。
4. 二次任务只到新 READY 并等待人工解锁，未跑第二次完整起飞/跟踪。完整 pipeline 的稳定观察窗口约2s；不可声称长期抗扰或水平动力学验收。
5. camera/estimator重置与历史 READY等覆盖由其他单测/ROS smoke提供，不是四条 pipeline 都逐项走相机硬件复位。最终报告应按证据层级分别列举。

## 输入绑定

- `tests/ros_step5_pipeline.py`: `08a8abb8936a13f0da137c018286b8b5d501fedb39f4f2119853e8d8045bb7cd`
- `tests/fixtures/step5_supervisor.py`: `da9e1acc6ef766515e58fed129e45c16e66eff7ca4a9d2bb1f45f575c5633771`
- `tests/ros_step5_fault_matrix.py`: `430d28c5520950001a0a0ac338adbee6856f309821846907e822836dd269ae67`
- `tests/ros_stage3_lifecycle.py`: `47e9267e27bb3a4739bf815be9faca90a161ab2842d5df91442e80c88865284f`

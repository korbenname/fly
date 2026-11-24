# 队列第1步独立审查

审查基线：`51acc74f9634e7beb631434d5035b6786abe8a2d`。只读审查，不修改共享源码。

## 可复现缺陷（应修复）

**P2：只读接收器的二次remap绕过/fmu边界。** `native/src/jikl_fc01_system/src/fc01_target_interface_node.cpp:117-120` 对参数先调用 `resolve_topic_name()`，返回解析结果给 `create_publisher/create_subscription`。后者还会解析一次。以状态输出为例：

```bash
ROS_DOMAIN_ID=168 ROS_LOCALHOST_ONLY=1 fc01_target_interface_node --ros-args \
 -r /fc01/interface/status:=/step1/status \
 -r /step1/status:=/fmu/in/vehicle_command
```

在已构建的真实x86 Humble隔离域168，使用rclpy图API观察到：

```
RETURN None
FMU publishers [('fc01_target_interface', 'std_msgs/msg/String')]
STEP publishers []
```

该类型不会匹配PX4的VehicleCommand，未获得真实控制权限；但违背该节点显式的“不在/fmu发布”边界，日志/端点审查也会误判。建议校验最终一次解析结果后返回原始topic值，让实体创建只做一次解析。增加双跳remap回归，确认只发布/step1/status且/fmu无输出。此发现已发送主代理，修后需复验。

## 可读性小问题（P3）

`target/src/fc01_vision/src/vision_node.cpp:28,64` 的depth5说明/启动日志与真正KeepLast(1)不一致；`target/src/fc01_mission/config/mission.yaml:6`仍写旧VelocityCommand类型，而实际为VelocityCommandV2。应同步说明，不涉及算法。审查期间主代理正在修正文档中其他旧VisionState表述。

## 本轮控制边界结论

- 当前tracked diff没有修改fc01_system_node、Runtime、Core、RC门、原始五种消息或原飞行YAML；只新增类型/可执行诊断器并扩展构建。
- 新接收器只有std_msgs/String输出，无Runtime或PX4发布器；单线程spin与三个缓存生命周期明确，构造异常由shared_ptr/Node成员析构清理，main捕获后shutdown。
- TargetObservation类型与原VisionState隔离；V2与V1类型/端点隔离。业务包只有一套native消息源。
- production launch默认control_enabled=false，不启动第二个fc01_system_node。速度reliable/KeepLast1/volatile在发布和新订阅端一致。
- 导入mission仍有sim平台、2.78m、push/impact、测量时效等历史逻辑；README明确后续接入且当前默认关闭。本阶段不得据接口成功宣称视觉可飞。
- 无实机PX4固件DDS或板端当前runtime连接，无法从本机验证其版本；报告应区分SDK证据与实机事实。

## 验证边界

独立执行的是上面的真实ROS双跳remap复现；其它原生控制行为结论来自源码差异检查，不宣称完成飞行验证或ARM64构建。没有启动飞行控制节点、发布飞控命令、改运行配置或操作设备。

## 修复后的独立复验（2026-11-18 00:15 CST）

主代理修复为校验resolved、实体创建传入原值，并拒绝不同参数解析到同一端点。本审查者重新运行真实Humble（ROS_DOMAIN_ID=169、ROS_LOCALHOST_ONLY=1），没有启动原生飞控节点：

| 场景 | 进程结果 | /fmu/in/vehicle_command发布者 | /step1/status发布者 |
|---|---|---|---|
| 双跳remap（同上复现命令） | 正常运行，SIGINT退出 | 0 | 1 |
| 直接将status remap到/fmu/in/vehicle_command | 退出1，`read-only interface cannot use PX4 topics` | 0 | 0 |
| status与velocity remap到/step1/same | 退出1，`remapped interface topics must be distinct` | 0 | 0 |

**P2修复的三个针对场景独立复验PASS。** 当前代码仍为只读接收，无新增飞行发布权。复验使用最新统一构建install树（binary修改时间00:14:14）。

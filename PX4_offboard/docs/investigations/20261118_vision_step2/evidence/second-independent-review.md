# 第2步修复后独立复核（第二轮）

本轮只读生产代码，无飞控节点、串口、服务操作，不占用统一构建缓存。基线仍为9e9fb3ef4c4e71d8b1b646bfe0d125a0e71c59a8。

## R1 修复结论：通过限定回归

`_clear_target`现在清除terminal_start/terminal_cmd、push_aim、last_v_cmd、前馈，并在冻结阶段锁存guidance_invalidated。`_update_mission`在该锁存时返回零+invalid；两个最终指令发布函数也拒绝锁存状态。传感器恢复和重新目标锁定不会解除。仅显式_reset_mission解除；生产未接入该重新准备流程，属于后续阶段。

实际执行：`/usr/bin/python3 -B -m unittest tests.test_mission_cached_demand tests.test_mission_sensor_contract tests.test_px4_platform_feedback -v`，29/29通过，日志`/tmp/fc01-step2-final-review-tests.log`。

另以AST提取当前原样_publish_command/_publish_navigation，不启动ROS，用可观察消息对象和发布Sink验证：即使传入valid=true且航点原点确认一致，锁存仍发零值V2+valid=false、航点valid=false。脚本`/tmp/fc01-step2-final-review-publish.py`，结果`.json`。这些是方法边界检查，不冒称真实DDS/飞行回归。

## 新发现 R3：畸形ready快照的同样本复活（P2，已通知root）

`sensor_contract.py:201`在finite/quaternion检查失败前没有消耗新测量水位。输入ready=true且新timestamp_sample的NaN位置快照，先被拒绝；随后同一source与原样sample stamp、修复为有限值的消息可重新被接受。标准桥正常会将NaN标为ready=false，所以当前正常桥流不能直接产生该路径；但mission边界已声称无效新样本不能复活，此处需要补回归。默认mission仍关闭，未获得实际飞行权限。

复现：`/usr/bin/python3 -B /tmp/fc01-step2-final-review-extra-repro.py`，结果`/tmp/fc01-step2-final-review-extra.json`。修复前same_sample_revalidated=true。建议在校验载荷前事务式消费所有已声明字段的测量水位，并保留独立旧水位判断何时入队；任何失败清历史但保留已消费水位，避免部分字段提前append导致恢复混样。本轮reviewer没有修改生产源，最终由root补丁与重跑结果关闭。

## 其他检查与边界

- R1新增成员在spin/timer开始前初始化；单线程ROS执行器下没有在构造半途回调读取这些成员。

- 平台回退会清历史并保留水位，重复快照不会更新field receive TTL；源会话更换清历史且退休旧身份。已通过已有模块测试。

- 新任务仍默认disabled，原生起飞、RC、推力、runtime配置没有进入本次修复范围。

- 未来启用mission前仍需处理上游2.78m/冲撞任务与原生1.5m、到高/RC门/水平仲裁；本次传感器测试不能代替这些后续工作。

## 审查源码身份

- target/src/fc01_mission/fc01_mission/mission_node.py SHA256 e14ada291933fdc17c5e125dd056fa84de0629f076934625bce28e2071f0e1f1
- target/src/fc01_mission/fc01_mission/sensor_contract.py SHA256 6a08ad3c5b85c655adbc01bd8c83bc4e23fd53209dcf8cd7d6ae9fb15bb1a639
- tests/test_mission_cached_demand.py SHA256 3e7df49c2a3207c6579092a4a83fc9c7c9c57b0d94afb79bb7814a402d789608

# mission/native producer 第 3 步独立复核

复核对象：fc01_system_node.cpp::publish_native_flight_state、NativeFlightGate.accept/current、MissionNode::_native_navigation_current/_on_platform_state/_publish_command/_publish_navigation。只编辑 mission guard、mission及其测试，未编辑原生发布实现。

## 已确认并修复

1. 原始底层测量的 TTL 可能在 accept 后被延长：在到达时已有 0.29 s 的 local/native/原始local接收时间，current 原先只复核消息接收/推进时间，0.49 s 时仍 true。证据 /tmp/fc01-step3-mission-current-ttl-before.json。current 每次使用重新核对原始三种年龄；对应 3 个分支回归通过。
2. NativeFlightState 与已接受 PlatformState 的坐标重置计数可能短暂不一致。mission 在每次最终授权重核 z/vz/quat 三个计数；未收到新平台参考或计数不同均撤销，直到真实桥更新。回归覆盖三个计数及尚无平台参考。
3. 第二轮复核发现：未知但过期的 changed-key 帧在校验 freshness 前退休当前会话，导致之后真实新帧永久 NATIVE_RETIRED_SESSION。证据 /tmp/fc01-step3-mission-stale-key-before.json，真实当前帧先 true，旧未知帧 false，真实新帧 false。已将源/原始单调/local样本时间核验放在 identity 变更前；旧帧仍撤销当次授权，却不退休真实会话。新回归证实真实新快照可恢复。该修复后纯测试40/40PASS。

## 原生生产者复核结论

- 快照来自当前 publication_context，消息在本周期 pair/command sink 后发布。publication_current 还要求本周期 target 和 mode 实际本地提交、无异常、未shutdown、当前controlfresh；不以 phase 字符串替代。
- 原始 local source stamp、local receive steady time、native steady time保留；mission不在接收端给旧测量补新时间。目标高度和h使用本次core reference_z - fb.z。navigation_current继续测量高度与垂速稳定证据，mission复核数值及原生布尔只收紧。
- 任务次数、源process session、同机boot、PID、reset counters改变均撤销旧授权；已退休身份不可复活；同身份序号/时间回退锁存。改变身份后首帧用于失效/切换，下一张新快照才可能授权。
- 生产任务无自动解锁/起飞/垂直命令。旧XYZ NavigationCommand生产始终invalid/全零，V2最终vz=0。视觉未开或无目标只保持native_height_hold；native状态缺失不停止原生垂直控制。
- 未发现其他可复现阻断问题。单writer图检查不等于逐包来源认证；本地ROS发布成功不等于PX4已经采纳，消息与注释未混淆这一点。

## 验证

环境 Ubuntu22.04 x86_64、ROS2 Humble，ROS_DOMAIN_ID184/ROS_LOCALHOST_ONLY1；无硬件、无/fmu/in发布。
- /tmp/fc01-step3-mission-unit.log 最新40/40PASS（mission gate10、cached2、sensor12、px4platform16）。
- 上一次构建Python SHA一致版本：/tmp/fc01-step3-mission-ros-final.json/.log，真实MissionNode+真实ROS pub/sub 11组PASS；/tmp/fc01-step3-interfaces-final/RESULT.json 与 /tmp/fc01-step3-sensors-final/RESULT.json 均PASS。
- 最后stale-key窄修之后native_flight_gate.py源码SHA ca7e302ca2b4cd4f5e5d41cfb049c715e6daa0c814377a8efe98e2a59aa5fbda，root需刷新统一install再复验最终ROS。其他生产文件未改。
- Python编译及限定 git diff --check 通过。首次测试命令误写不存在的test_platform_bridge_math得到ImportError，改用实际test_px4_platform_feedback后40/40通过；这是测试命令错误，非生产代码问题。

现场仍需：实际RC映射/Offboard确认、ARM64、真实PX4 adopted setpoint与飞行性能。第4步前V2原生接收仍是只读，本步不声称已开启视觉飞行。

## 独立复核R1后续

已接受 /tmp/fc01-step3-final-review.md 的R1，异常同seq/stamp改变armed/offboard/landed/slot/h/vz可沿用旧true。已修为必须seq与stamp同时相同且所有授权有关原始时间/安全布尔/模式/高度字段一致，否则撤销latest；完全相同DDS重复仍只保留原TTL。新增回归后41/41PASS。等待统一install刷新后最终ROS。

## 最新实际ROS结果

统一install guard已与源码34fc7bef一致。R1新增ROS夹具首轮因前一张正常帧尚在回调队列，等待任一invalid后立即比较reason失败。仅对/tmp复制夹具停发后排空50ms，并等待矛盾reason与invalid同时出现，再用真实ROS domain185运行12组PASS：/tmp/fc01-step3-mission-fixture-drained.{py,json,log}。repo保持root冻结，等待root应用最小测试修改并正式复验；没有修改生产逻辑来规避失败。

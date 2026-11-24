# 第3步独立只读复核（最终；2026-11-18）

未编辑生产源码。发现1个可重复的消息边界缺口；已由mission负责者修复并经本复核重新验证：

## R1 / P2（已修复）：重复身份消息改变实际安全字段时，gate仍沿用旧授权

`target/src/fc01_mission/fc01_mission/native_flight_gate.py:89-94`：seq相同或stamp相同走重复分支，检查feedback/publication/nav/settled及Kill/failsafe/takeover，但未核对armed/offboard/landed/RC槽位/实测h/vz等。新收到同seq、同stamp消息若改变这些字段，函数仍返回旧`latest`的true。已确认armed=false、offboard=false、landed=true、slot2、h=0、vz=1都返回`NATIVE_NAVIGATION_CURRENT`。

证据 `/tmp/fc01-step3-review-duplicate-change.json`。原生生产者自身每次发布seq递增，正常DDS重复不会改变内容，所以该问题需要同身份内容矛盾的异常消息；当前V2仍为只读意图，也未直接获得PX4发布权。不是实机正常路径已会出现的结论。建议同seq或同stamp快照须安全字段与已接受消息一致；不一致立即撤销，至少完整验证所有必需布尔/模式/数值而不更新接收时效。完全相同重复可保持剩余TTL但不能续鲜。

## 已核对的正确边界

- 原生producer在同轮真实target+mode双提交之后产生10Hz状态；重新snapshot并检查控制预算、当前health/reset、RC/Battery/Preflight，历史阶段名不能单独授权。
- NativeFlightState source绑定boot/PID/启动会话，任务epoch来自原生安全重新准备；mission检查同机时间、来源、唯一writer，并拒绝退休身份和源时间倒退。
- Core新到高证据要求至少3个实际原始sample，源域和主机域同时满足稳定窗；HeightHold越界撤销水平而保留垂向纠偏；final query再次检查当前h=z0-z和vz/模式/reset。
- mission生产px4分支从wait_native进入独立水平路径；旧sim takeoff/2.78/push高度分支被隔离。production NavigationCommand整体失效，V2最终body-FRD.vz=0。
- navigation.render_status核对boot/supervisor/controller/IPC身份，历史READY→STALE，当前准备标签额外核对本轮ground/ref/prestreamzero；显示不会发送控制请求。

## 复核后的修复验证

mission负责者已将重复分支收紧为同seq、同stamp、底层时间和全部重要安全/测量字段必须一致，否则`NATIVE_DUPLICATE_INCONSISTENT`并清除latest。对R1六种输入原样重跑全部false；证据 `/tmp/fc01-step3-review-duplicate-change-after.json`。额外复验 `tests.test_mission_native_gate` 与 `tests.test_stage3_status_readiness`：19/19通过，原始日志`/tmp/fc01-step3-independent-tests-final.log`。

只读检查此时未发现其他需阻止第3步提交的已确认缺陷。检查重点为本轮新增控制授权合同，未声称全工程/全状态空间穷尽。

复核限制：未使用香橙派/实际PX4/相机，不证明实时ARM调度、物理到高、DDS真实链路时效。`body_frd.vz=0`仅是任务接口的禁止垂直请求约定，不能独立证明倾斜时世界竖直速度为零；最终水平仲裁仍需后续阶段在原生固定高度合同下实现。当前V2没有实际PX4动力发布权。

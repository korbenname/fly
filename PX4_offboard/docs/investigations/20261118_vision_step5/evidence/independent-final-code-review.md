# Step5 最终原子快照：独立只读复核

复核时间：2026-11-18。复核范围为当前未提交差异；起点 HEAD 为 `747b9879967fb96fad5953f080b8765a7c9e874f`。未修改生产代码、配置或测试，未提交、部署、推送。

## 结论

在本次最终生产差异范围内，未发现新的、已确认的 P1/P2 逻辑缺陷。可以将它作为软件验证候选保留；不能据此宣布已经通过 ARM64、PX4 固件接收/采纳或实物视觉飞行验收。

复核的最终版本使用 `NativeFlightSnapshot` 原子消息，**不再依赖桥的降采样姿态历史恰好包含原生确认样本**。此前共同历史交集方案的可用性问题已经被替代。

## 逐项检查

1. **原生上下文一致**：`fc01_system_node.cpp:2288` 的 `publish_native_flight_state()` 在既有目标/心跳 sink 之后运行，同一执行器期间没有回调插入。`:2372` 起将同次 publication_context 的 `fb.q`、状态样本戳、当前局部原点和 `attitude_sample_.received` 一起复制。q 原值来自实际接收的 PX4 姿态；没有插值、姿态猜测或新测量戳。旧 `NativeFlightState` 消息和话题未变。新观察发布独立捕获 ROS 发布异常，没有新增控制命令。
2. **测量不续鲜**：`mission_node.py:486` 的 `_native_attitude_current()` 同时检查原始姿态源时刻和原生首次单调接收时刻；预算取 sensor/native 两者较小值。原生重复发布快照不会更新 `attitude_sample_.received`。`_on_native_snapshot()` 拒绝同状态改 payload、同原样本跨新状态改 q/origin/receipt，以及同会话 raw 戳回退。精确重复保留旧 tuple、既有 gate receipt/local_progress，不能延长 TTL。
3. **来源、任务和 reset**：binding 包含 source/boot/pid/task/z/vz/q reset、序号、状态时刻和 raw 样本戳。原 `NativeFlightGate` 继续复核唯一发布者、进程/任务身份、源/接收时间、真实到高/Offboard/RC/保护条件，并退休旧会话；原子封套不替代该门。消费时还要求平台 reset 三元组及完整 origin 一致。平台位置/速度/角速度仍由原平台历史独立检查，不能靠姿态封套补出缺测。
4. **无效与恢复**：原子封套 schema/q/origin/年龄错误会清空快照并 invalidate gate；若此前已授权则清目标并锁存跟踪故障。gate.current 或平台/原点检查失效时，定时任务和最终发布处均拒绝水平意图；已开始跟踪会走 `_clear_target()`，健康后续图像不能自动消除故障。只有既有同一原生进程新任务 epoch 规则解除任务故障，未新增放宽路径。
5. **高帧率待对齐修复**：`:718` 起先消费当前单槽 pending，未等到平台后一帧则保留该帧，仅空槽时接最新帧。仍只有一帧，无积压/外推，原 source stamp 与首次 receive 不变。新无效/重启帧仍在消费前撤销；pending 过期会走现有故障锁存；继续来的新帧不能刷新旧 pending。
6. **精确旋转与不同频率**：最终 `_publish_command()` 使用原子样本转换 ENU/FLU→FRD，并携带同一 raw 姿态戳。原生 `horizontal_snapshot()` 仍要求64帧缓存中相同 stamp、publisher、origin、reset，使用其原接收时刻；未改成最近姿态匹配，未放宽安全门。原子消息因此解决独立接收先后与200Hz raw/50Hz bridge降采样问题，同时保持最终消费者的精确复核。
7. **控制权限与兼容**：新消息是观察证据；mission仍没有解锁/起飞/垂向推力权限。此次原生 diff 仅增加观察 publisher 与 payload；Core/Runtime/RC/推力/发布边界无算法修改。launch/YAML只是让 mission 默认消费新 snapshot，默认视觉关闭与标定确认 false 没有改变，旧诊断调用方仍可订阅旧消息。新旧生产包必须成套重建，旧版本不具备新 typesupport 时不能只替换 mission 文件。

## 本复核实际执行及核验

- 实际运行 `PYTHONPATH=.:tests python3 -m unittest tests.test_mission_native_gate tests.test_mission_pending_alignment tests.test_mission_horizontal -v`：**36/36 PASS**。日志 `/tmp/fc01-step5-final-atomic-review-tests.log`。
- 读取当前四条完整 ROS 链结果：normal、target_loss、no_vision、dds_loss 均 PASS；读取 camera 链12项 PASS。这里只核对现存结果、输入身份和实现路径，没有声称由本复核重新运行这五条链。
- 当前 mission SHA256 `de7825ecdd8283d0125b59cbd6858d788d723b157f826364b9871c13ffc3a17b`，node SHA256 `19d20653aea1844e582cc01819da2d0beffe767e1c1b403281070127dee0f0b5`，与现有最终测试 source_identity 一致。
- 查阅 native 独立回归交接及实际起点差分 RESULT：相对本步起点1178条控制 sink、707状态样本一致；CTests88、两类 sanitizer、历史 sink 差分的证据由原 native 回归执行者提供，未重复计数。两条历史完整 JSON 字节差分仍失败，不应改写成 PASS；新增诊断字段解释不代替原失败证据。

## 保留的限制和现场判据

- 64 raw姿态历史、10Hz观察、0.3s TTL 的实际时效交集在高频/丢包/ARM调度下仍须拆桨观察。候选保持 fail-closed；没有通过扩大缓存或TTL来掩盖板端负载。
- 合成垂向 plant 下降出现原 DESCENT_TIMEOUT/RECOVERY_DESCENT，只能报告最终回落/地面/实际上锁/再准备通过，不能说下降全过程没有保护事件。
- 全DDS丢失测试证明锁存与发布停止，后续模拟地面/上锁用于测试清理，不能算作失去反馈仍自主降落。
- x86合成图像与DDS接收端不能证明真实相机曝光时刻、外参、识别性能、PX4采纳、水平动力学/漂移、ARM实时性和真实systemd/Agent环境。下一阶段交付必须继续标明这些边界。

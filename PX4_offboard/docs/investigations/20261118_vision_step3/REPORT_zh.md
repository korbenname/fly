# 第 3 步：原生 1.5 m 定高接续任务状态机

已完成本步代码、离线复现、真实 ROS 2 通信、回归和独立复核；未部署、未推送、未连接动力。相机仍不能直接控制飞机，水平执行仲裁留给后续队列。

## 基线与交付身份

- 分支：`readability/fc01-chinese-20261111`。
- 开始 HEAD：`827f1f4535bc55b1bfa3fc2e97439367af226831`（第 2 步报告提交）。
- 本步候选代码：`08ac6430a67fed85fba75dddb465d5598bb60462`。
- 已安装版本的发行源码 `4c85a9e7e0c66193104164f03f4bfe28de709b9d`、原生候选 `adc55d0b7c67e80bf9d77b1082208c898f95b756`、维护安装器 `41e1532e73d36005a80cd134807a70b0c0263675` 均为当前祖先。
- 本轮没有香橙派新读回。不能声称现场已经运行本候选，或当前板端 runtime 已重新核对。
- 现有现场配置快照 `tests/fixtures/stage3_runtime_20261114.yaml`：目标 1.5 m、导航槽位 1、人工解锁到导航期限 8 s、视觉关闭；hover/reserve/min/max/slew 为 `0.50/0.04/0.12/0.80/0.40`。本轮未修改原生飞行 YAML、PX4 参数、Agent、UART 或维护安装器。
- 开始/结束本地没有 `main` 分支；缓存 `origin/main` 为 `377a945d9cd1147baad8871cd5387b9e63cd774d`，未改引用、未网络推送。
- 原有未跟踪 `tools/`、两份 `tests/test_micovio_*.py` 保留，不纳入本步提交。

实际构建先于代码提交，原始结果中的 `source_head` 因而保留开始 HEAD。`FINAL_SOURCE_SHA256.json`、`PYTHON_INSTALL_IDENTITY.json`、`COMMIT_BINDING.json` 将最终文件、安装 Python 和候选提交逐字节绑定；没有把历史 ELF 重新贴上新 HEAD。新增/修改文件见 `evidence/CODE_CHANGED_FILES.txt`。

## 读过的实际路径与判断

1. `rc_navigation.hpp` / `attitude_altitude_core.hpp`：原有地面参考、人工 RC 解锁证明、8 s、导航槽位与 PX4 实际 Offboard 配对已存在，保留原顺序。READY 需要未解锁、Stabilized、非导航、低油门、接地静止、新反馈及实际零推力预发送提交，任务不得替代这些门。
2. `Runtime::step → Core::step → publication_guard/publication_boundary`：高度沿原有 `h=z0-z`，闭环反馈为 PX4 融合 z/vz，地面零点不在解锁后重新定义；测距继续仅核对上升趋势。
3. 旧 `Core::dwell()` 与 `target_height_settled()`：主机周期可用同一条缓存测量凑到高；进入 HeightHold 后越界仍保留水平许可。已独立复现，确认为本步缺陷。
4. `mission_node.py::_update_mission/_publish_command/_publish_navigation`：旧实机分支会使用上游 2.78 m 和自动 takeoff；没有原生状态仍生成向上意图。已有失败复现不是根据注释推断。
5. `navigation.py::render_status` / `navigation_status.py`：旧 READY 标签可与当前 ASCEND/armed 状态矛盾；服务重启的旧 READY 未明确显示 STALE，且缺部分进程身份仍可能提示解锁。

## 实际代码变化

### 原生到高与发布后状态

- `takeoff_evidence.hpp::TargetHeightSettling`：至少三条严格推进的原始位置样本，同时覆盖源时间和主机单调时间的稳定窗。重复、同戳换值、回退、断续和过期不能制造连续稳定。
- `Core::step/target_height_settled/navigation_current/local_requires_control_step`：真实离地、平滑轨迹到最终目标、实测高度和垂速连续稳定后才到高。已经 HeightHold 后越界会撤销水平授权，保持垂向闭环纠偏；中间越界回调不能被健康尾帧覆盖。最终查询重新核对当前健康、模式、重置、h/vz 和证据年龄。
- 新增 `NativeFlightState.msg` 和 `SystemNode::publish_native_flight_state()`：真实 target/mode sink 之后约 10 Hz 生成，使用新 snapshot 和当前发布合同；包含 boot、进程会话、PID、任务 epoch、原始测量戳、单调接收时刻、地面零点、目标与实际高度、离地/到高和发布证据。BEST_EFFORT、volatile、keep-last 1。它是观察合同，没有新的解锁入口。
- 状态 JSON 同样重核当前导航许可并补稳定样本计数；没有改变原有推力、人工 RC、接管、Kill、failsafe 或最终目标/心跳发布顺序。

### 实机 mission

- 生产 launch 固定 PX4 平台来源，启动 `wait_native`。原生唯一 writer、本机 boot、进程/任务/reset 身份、序号、源与单调年龄、原始 local 推进、当前离地/到高/发布证据及数值必须一致。
- mission 与 PlatformState 的 z/vz/quat reset 必须一致。来源或任务变化清除旧目标。检测成功和任务计时不再提供飞行授权。
- 实机绕过全部旧 simulation takeoff/strike/push 高度分支；默认 `control_enabled=false`。`takeoff_altitude_m: 2.78` 实例移除，兼容模拟值改 1.5，实机不读取它。
- V2 仅表达水平意图，最终 `body_frd.vz=0`；旧 NavigationCommand 没有独立 z 有效位，生产发送明确失效的 XYZ 撤销。倾斜时 body vz=0 本身不能证明世界垂直速度为零，后续仲裁必须按原生固定高度合同执行。
- 视觉关闭、无相机或无目标，不阻断原生 1.5 m 路径。本步 V2 原生接收仍只读，没有开启视觉动力控制。

### 状态窗口

`render_status` 严格核对 boot、监督器 PID、控制器 PID 和 IPC 状态归属；历史 READY 显示 STALE。当前 READY 标签还要与原生阶段、接地静止、RC、地面零点和零推力提交一致。保留当前显式诊断模式和真实故障锁存的解释；显示层不重建原生 8 s 窗口、不发送飞行动作。

## 缺陷与独立复核

| 级别与状态 | 已确认问题 | 修复/证据 |
|---|---|---|
| P1，已修 | 缓存单样本凑连续到高；到高后测量越界仍准许水平 | `native/cached-height-reproduction.json`，新原生 gate 回归 |
| P1，已隔离实机路径 | mission 未经原生授权自行生成 2.78 m 起飞需求 | `fc01-step3-mission-before.log`，生产 wait_native/水平分支 |
| P2，已修 | 历史或矛盾 READY 提示人工解锁 | `fc01-step3-status-before.json`，8 项状态专项与实际 render |
| P2，复核发现并修复 | accept 时已旧的底层样本在 current 再借接收 TTL 续鲜 | `fc01-step3-mission-current-ttl-before.json`，每次使用重核原始年龄 |
| P2，已修 | 过期未知身份先退休当前会话，使真实新反馈无法恢复 | `fc01-step3-mission-stale-key-before.json`，先校验时间再切身份 |
| P2，独立复核发现并修复 | 同 seq/stamp 却改变模式/高度等内容仍沿用旧授权 | `fc01-step3-review-duplicate-change*.json`，内容矛盾立即撤销 |

复核原始说明在 `evidence/fc01-step3-final-review.md`、`evidence/fc01-step3-mission-review.md`，原生专项见 `NATIVE_REVIEW_zh.md`。评级针对接入后的潜在影响，不是已经发生实机失控的结论。

## 最新源码实际验证

环境：Ubuntu 22.04.5 x86_64、ROS 2 Humble，SDK px4_msgs；隔离 ROS domain 184/191–198、localhost-only，无 PX4 固件、Agent、串口、相机或真实动力。

| 检查 | 实际结果 |
|---|---|
| 相关 Python 单元/状态/任务/静态合同/打包边界 | **124/124 PASS**，`fc01-step3-all-python-final.log` |
| 维护安装/预编译服务/配置兼容 | **63/63 PASS**，不增加硬件准入条件 |
| 原生 Debug 严格告警构建、CTest | **86/86 PASS**，最终 `fc01-step3-native-final-ctest.log` |
| 新原生 gate ASan + UBSan | PASS；仅专项，不声称全工程 sanitizer |
| Humble 统一构建 | **4 包 PASS**；最终安装 Python SHA 与源码完全一致 |
| 生成类型支持与 CDR | **9 种消息 PASS**，含 NativeFlightState |
| 真实 ROS mission 原生接续 | **12 组 PASS**，最终仓库 smoke 已应用队列排空修正并重跑 |
| 真实 ROS 旧接口/remap | PASS；保留 V1 与只读接收 |
| 真实 ROS 相机合成图像→平台桥→mission | **13 组 PASS**，无真实相机性能结论 |
| 视觉 CTest | **2/2 PASS** |
| 真实 ROS Runtime→Core→发布边界生命周期 | **10/10 场景 PASS**，详见下表 |
| 三套原生确定性轨迹 | **26,371 行逐字节相同**，原始 hexfloat |
| 当前开始基线的完整 SystemNode 控制出口差分 | **1/1 PASS**，1,178 条 target/heartbeat/command 全字段与顺序完全相同；707 份控制事实投影相同 |
| 升级 ROS 测试替身后的恢复/语法 | 恢复 **1/1 PASS**，syntax PASS |
| 更早 f067 历史完整 JSON 字节比较 | **7 项中 5 PASS、2 FAIL**；见下一节，不计为全通过 |

这些是不同层级的计数，不能相加成所谓“飞行通过次数”。

### 真实 ROS 生命周期

| 场景 | 检查结果 |
|---|---|
| normal | 人工解锁、slot1、实际 Offboard、离地、连续到高、保持、正常 LAND、地面上锁、释放与再次 READY，等待下一次人工操作 |
| range_unavailable | 贴地测距暂不可用仍可按融合 z/vz 起飞；空中恢复趋势核对，完成上锁/再次准备 |
| feedback_stale | 旧反馈触发现有保护，导航撤销、故障锁存、健康尾帧不恢复输出 |
| quat_reset / vertical_reset | 姿态/垂直估计重置保持现有保护，不重建空中 z0 |
| offboard_loss / offboard_flag | 模式退出或 PX4 失联标志触发现有接管保护 |
| rc_takeover / kill | 现有接管/急停停流与锁存成立 |
| velocity_warning | 单独 `|vz-z_deriv|` 走诊断告警，验证窗口不停止闭环；清理阶段人为瞬移地面可见原有跳变保护，不是告警触发保护 |

错误档位、8 s 超时、模式确认延迟、假到高、重复/回退样本、中间越界和 armed 后 reset 同时由原生 Runtime/Core/boundary 回归覆盖；状态测试覆盖历史 READY 和断 DDS。未把 mission 的合成 NativeFlightState 测试当成原生真实起飞证明。

正常简化模型：超调 **0.1393 m**，进入稳定范围约 **7.89 s**（从 ramp 起），原生确认 HeightHold 约 **10.18 s**；观察保持约 **2.00 s**，最大误差 **0.1393 m**，RMS **0.1279 m**；心跳含窗口边缘最大间隔 **0.05394 s**。这些由无姿态/电机/地效/延迟噪声的简化垂向模型产生。XY 输入固定，因此“水平漂移 0”没有测量能力意义。没有本次可比空中 Position ULog，不作真实性能对比。

本模型下降期间进入原有 `DESCENT_TIMEOUT_CONTROLLED_RECOVERY_ACTIVE_HANDOFF_UNVERIFIED` 后继续受保护下降并完成上锁。保留该真实路径，不写成从未超时。现有 0.2 m/s 与下降期限的实机匹配需要现场观察，本步不调参。

### 失败、修正和验证限度

- 首轮节点构建：const 状态函数调用会修改控制周期观察器的接口失败。改用 snapshot 已记录的周期资格；最终真实构建通过，原始编译错误保留。
- 状态集成初次 72 项有 3 失败：显式诊断被 STALE 覆盖及旧测试身份夹具；修正诊断优先级与真实夹具后重跑通过。
- range_unavailable 首次漏采仅一周期的 SAFE_DISARMED，生产 phase_event 已记录终态。夹具现在同时要求可靠阶段事件和 IPC 当前 ACK+新 PX4 上锁证明，不能靠 ACK 或较后阶段名冒充上锁；重跑 PASS。原失败 result/controller 压缩日志保留。
- 最后 gate 源码更新后的一次 ROS 测试发现安装树仍旧，精准原因断言失败。重新构建、比对安装 SHA，再以最终仓库 smoke 重跑 12 组 PASS。原日志保留。
- 旧 fake-ROS 首先缺新消息替身，补齐后历史完整 JSON 两例仍 FAIL：新增稳定样本字段和收紧的导航观察值不再与更早版本逐字节相同。没有删除或跳过这两例，也不声称全量历史测试全绿。独立完整节点比较保留所有动力/心跳/命令字段，并对当前开始基线核对 707 份控制事实；另提取旧两例的 558/1,178 条控制出口也完全相同。完整原始日志与明确投影规则均保留。
- 原始 ROS CLI 文本的末尾空行原样保存在 `.txt.gz`，不改写证据来满足空白检查。
- 小型脚本首次 source Humble 使用 nounset、首次新差分临时目录重名等工具/夹具错误不算生产故障；最终执行命令已改正。

## 剩余事项与后续交接

- **P1，后续水平执行仲裁**：当前 V2 只读；下一步必须在原生固定 1.5 m 高度、RC/接管/Kill 和最终发布上下文中实现水平仲裁，不能直接开视觉开关。
- **P2，板端事实**：当前服务入口/实际 runtime/进程版本、RC 映射、实际 PX4 DDS 版本与角速度话题、时钟同步和 ARM64 调度尚未本轮读取。
- **P2，真实测量与标定**：相机设备、矫正/内外参、杆臂、采集延迟及完整 Position 飞行 ULog/同期香橙派日志仍缺；相机读阻塞/退出硬截止也是第 2 步遗留限制。
- **P3，兼容结构**：旧 simulation 高度任务保留为显式离线兼容；唯一 ROS writer 图检查不等于逐包来源认证。
- 本轮没有新确认且未修的 P0；不代表工程不存在未发现的 P0/P1。

复验命令见 `VERIFY_zh.md`。本候选可单独回退：尚未部署时用 `git revert 08ac643` 生成反向提交，保留之前历史；不得覆盖用户未提交内容。实际设备仍保持原安装版本，本轮不需要也没有切换板端服务。

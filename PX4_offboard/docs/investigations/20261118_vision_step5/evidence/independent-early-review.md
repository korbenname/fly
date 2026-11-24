# 第5步独立边界审查与负面验证

基线：`747b9879967fb96fad5953f080b8765a7c9e874f`。审查只修改新增 `tests/test_vision_step5_safety_edges.py`，未修改生产源码、提交、部署、systemd、PX4或设备。原有UART未跟踪文件保留。

## 真实入口与证据层级

1. `vision_replay.py::driver()` 编译 `native/tools/vision_replay.cpp`，用真实Runtime/RC/Core和本进程双sink；没有ROS、Supervisor、systemd或PX4。`encode_records()`仍编码旧VisionState/NavigationCommand合同，未编码新HorizontalVelocityCommand/PX4实际水平快照。它可作旧视觉兼容与控制安全回归，不能证明第4步新水平生产通道闭环。`tests/test_vision_replay.py::test_target_height_changes_original_vertical_controller_thrust`刻意验证旧合同能改高度；这与新选择器固定1.5m是隔离合同，不能合并解读。
2. `tests/ros_vision_smoke.py`实际跑Python Odometry/Health→VisionState适配器，检测ENU/FLU变换、只读参数、发布者冲突撤销、零fmu发布者及SIGINT退出。没有相机目标检测、mission或原生飞控输出。
3. `tests/ros_stage3_lifecycle.py`第364行直接Popen传入的原生ELF，并用IPC调STATUS/LAND，实际DDS订阅姿态/心跳/命令；未调用Supervisor/systemd。第4步全部12场景的实际接收者是合成PX4 peer，不是PX4固件，更不是PX4已采纳。必须在第5步额外贯通服务入口或者如实记录覆盖缺口。
4. 既有ARM64 `docs/investigations/20261116_arm64_release/FINAL_REPORT_zh.md`绑定旧候选adc55d0/56424d7；真systemd证据在QEMU-system-aarch64 VM。其node源码SHA `309a8008...de4122` 与当前 `761f5297...4a5d5`不同，没有新水平头文件/消息。不能作为当前第5步ARM构建或设备通过证据。
5. 安装器和服务仍强制视觉关闭：`navigation_prebuilt_install.py:416` / `navigation_prebuilt_service.py:150`、`navigation_service.py::navigation_profile()`；默认真实服务不会运行视觉路径，纯给服务加visual=true会被软件合同拒绝。这是待显式候选发行适配的限制，不应为测试通过偷偷取消。

## 已执行的独立新回归

命令：

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_vision_step5_safety_edges -v
PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_mission_native_gate tests.test_stage3_status_readiness tests.test_vision_replay -v
```

最新结果：2个新增unittest通过，内部5项具名C++检查通过；第二组30项通过（与父级总数计数时按方法去重）。实际编译 `SystemNode`回调和生产外环头文件，没有复制算法。

- NAV撤销序号11→乱序旧有效10→新有效12，撤销代数保留。
- 速度完全重复不更新原received_at；invalid→乱序old good→new good不能抹掉撤销。
- 同sequence改载荷先撤销且保留首份原载荷，不能用后续变值冒充重复。
- `Controller::current()`对于任一通道新增第二writer、原TTL过期、撤销代数变化均拒绝原计划；重复调用不积分yaw。
- PX4实际快照持续新鲜但任务指令不推进时，原命令TTL到期请求下降；新健康帧不能自清故障。

日志：`/tmp/fc01-step5-safety-edges-unittest.log`；原始编译/执行/源哈希JSON位于`/tmp/fc01-step5-safety-edges/`。
旧兼容回归日志：`/tmp/fc01-step5-boundary-existing.log`。

新增测试首跑有2个测试夹具错误，不是生产缺陷：断言假设VelocityCommand浮点是float而实为double；将main重命名引入非main函数缺return的严格告警。已保存`/tmp/fc01-step5-safety-edges-test-fixture-first-failure.log`并修正夹具。未放宽生产校验。

## 当前只读审查结论

没有新复现的生产控制安全缺陷。现有处理并非应被取消的错误：

- 命令任一通道发生来源/序号/原点异常时，全局撤销代数阻止已选中计划，避免A→B→A被末帧覆盖。
- 重复图像/重复原生状态不能续鲜；历史READY不能替代当前离地/连续到高/实际Offboard。
- 原生最终策略仍要求HeightHold、当前visual plan、有限倾角、当前armed/offboard/RC/反馈；偏航/倾角不能绕过其原生门。
- 不给新水平通道带垂向权限；旧回放测得可改高度不应用于当前生产新选择器。

## 第5步仍应补齐的关键证据

- 同一隔离ROS运行相机原始输入→真实目标检测→PlatformState桥/mission→真实原生订阅→姿态/心跳实际接收，而非拼接各自PASS。
- 服务/监督器入口的实际运行范围；若不能在现环境完整跑，说明直接node测试不覆盖它。
- 洪泛、跨话题顺序、重复/乱序/多writer、相机和估计器重置的真实DDS运行，须确保故障注入实际发生在跟踪已启动后。
- 水平故障完整下降/上锁/再准备。第4步7个水平故障只到DESCENT；夹具清理强制位置回地面后出现估计跳变锁存不能当作受控落地。
- 合成相机源的CPU/RSS/延迟/新帧率、实际控制周期与心跳间隔，注明x86与生成输入边界；真实相机采集延时/标定与PX4采纳仍待现场。

第4步normal模型从DESCENT进入过RecoveryDescent，再完成接地/上锁/再准备，不得写成无超时普通下降。XY固定输入的0漂移不是稳定性结论。

## 续审补充

读了新 tests/fixtures/step5_supervisor.py：真实standby监督器锁、配置、spawn、LogPump、IPC、StatusWriter和退出资源清理；ARM/SDK/UART预检、全局硬件进程发现和AgentTransport替代项单独写boundary.json。该证据比直接ELF完整，但仍不是systemd/PX4硬件验收。

新最终边界负面C++夹具另以ASan+UBSan编译执行通过，日志 /tmp/fc01-step5-boundary-asan.log。

已建议父代理统一raw图像array('B')赋值、有限共享段和按实际收包决定结果证据布尔值，避免夹具性能/失败路径统计误导。第二次READY未覆盖第二次真实到高跟踪，若只做READY必须报告此覆盖边界。

## 最终故障矩阵独立复核（2026-11-18）

- 核对 `/tmp/fc01-step5-fault-final/final-summary.json` 及全部 11 个逐场景 result.json；摘要 SHA 与原件一致，11/11 PASS、全部 gate true、native_exit=0、同 HEAD `747b9879967fb96fad5953f080b8765a7c9e874f`，52 个原生生产输入与实际 ELF 构建证据绑定。
- 对照 `tests/ros_step5_fault_matrix.py` 的生成锚点与原驱动，原7个水平故障 + 重复/乱序/第二发布者共10项，故障回落段没有 cleaning/h=0 的收尾捷径；原单轴动力模型持续积分，直到真实原生 Disarm 请求，另验证 ACK 单独不能代替新 Disarmed 状态。
- 全10项包含：故障观察 → 连续模型下降 → Disarm请求 → ACK-only反证 → 独立新Disarmed → SAFE_DISARMED → 地面重准备 → 第二次 READY。最后第二次 READY 仅是重新准备，未覆盖第二次完整飞行。
- 实际阶段均会经过原控制器 `RECOVERY_DESCENT`（下降10秒后原超时保护）；释放Offboard后还有 `OPERATOR_TAKEOVER_LATCHED`，之后以新接地/上锁/复位控制证据恢复 READY。不能称下降无超时或完全无锁存阶段。
- `preheight_flood` 通过真实DDS注入有效运动请求突发，实际姿态发布在起飞到高前保持无水平/偏航，仍完成 1.5m 到高和原生落地/上锁/重准备。矩阵最长心跳接收间隔 0.058559641998726875s，仍为本机合成性能。
- 这些验证是直接 native ELF + Humble DDS + 本机合成PX4，未通过 systemd/真实Supervisor、未运行PX4固件、未使用ARM64/实体相机/实物飞机；不能把 DDS 接收端模型称为飞控实际采纳。

未确认新的生产缺陷；新增安全边缘回归 2/2 PASS，严格编译及控制器负面场景 ASan/UBSan/leak check PASS。父代理正在完成真实监督器入口的相机→任务→原生贯通测试；本次故障矩阵独立审核不代替该项。

## 新确认的相机/平台跨速率时间对齐缺陷

父代理贯通第7/8次运行中，原始新图像持续检测有效，mission曾进入实际TRACK，但数十毫秒后撤销，原生按保护下降。第8次真实mission状态明确为 `TARGET_SOURCE_OR_RECEIVE_STALE` 与 `POSITION_IMAGE_TIME_NOT_BRACKETED`。

独立离线重现命令：`python3 /tmp/fc01-step5-pending-alignment-repro.py`。脚本用生产 `_consume_pending()` 的真实AST方法和真实 `PlatformHistory`，120Hz图像、50Hz PX4源、2ms raw样本年龄及10ms平台桥接调度。平台和图像均有效；最新单槽不断覆盖尚待未来平台端点的图像，2秒消费 **0帧**；对照仅保留最老待对齐帧，其他完全相同时消费 **79帧**。该对照仅用于确认根因，尚非候选生产补丁。

真实缺陷范围：`mission_node.py::_on_state()` 无条件覆盖 `pending_target`，配合 `_consume_pending()` 严格内插时间条件，图像比平台更快时可能永远看不到可对齐的待帧。不应通过放宽TTL、将最新平台姿态冒充图像时刻或忽略失效来修复。可采用有界等待/队列，保留原始戳、接收时间、顺序和双时效，平台更新后消费，再继续最新新帧。生产修复由父代理协调；本独立审查未修改生产文件。

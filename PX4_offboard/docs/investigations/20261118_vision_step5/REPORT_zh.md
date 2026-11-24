# 第5步：相机输入到原生飞控发布边界回归

日期：2026-11-18。分支 `readability/fc01-chinese-20261111`。本步完成候选修复、真实Humble构建、软件闭环/故障回归、独立代码及证据复核和本地提交；尚未通过实机视觉飞行验收。

## 1. 身份与实际范围

- 开始提交：`747b9879967fb96fad5953f080b8765a7c9e874f`。
- 代码/测试候选：`5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`，24文件；本报告与证据另作后续本地记录提交。
- x86原生ELF SHA256：`1e9a0162eb2b4200b8d913257139765e936efd637b5e5e93c685436e93967d45`。
- mission SHA256：`de7825ecdd8283d0125b59cbd6858d788d723b157f826364b9871c13ffc3a17b`。
- 构建/测试发生在代码提交之前，原始结果保留当时HEAD及工作区SHA；`SOURCE_BINDING.json`确认53个原生生产输入、52个target生产文件、已安装mission和候选提交为相同字节，没有重贴历史构建身份。

没有推送、部署、修改main、PX4参数、现场runtime、Agent/UART或维护安装器，没有解锁或使用真实动力。视觉及水平控制默认仍关闭，标定确认仍为false。未纳入原有未跟踪 `tools/` 和两份MicoVIO测试。本步不生成ARM64安装包，不提前执行下一步发行。

已重读现有 `vision_replay.py`、`ros_vision_smoke.py`、起飞/发布边界、Stage3 ROS场景及历史ARM64发行记录。历史ARM64/VM验证属于旧候选，不含本步快照类型/任务修复，不能作为本候选板端证据。

## 2. 入口及证据层级

| 入口 | 实际执行内容 | 不覆盖内容 |
|---|---|---|
| `vision_replay.py` | 实际Runtime/Core与内存发布sink，旧自机视觉合同 | 新目标相机水平通道、DDS、固件 |
| `tests/ros_vision_smoke.py` | 真实Odometry→VisionState适配、参数/资源/清理 | 目标识别和控制器 |
| `tests/ros_camera_pipeline_smoke.py` | 实际C++图像检测/估计→TargetObservation DDS→实际mission→水平封套DDS | 原生节点、PX4；Native/Platform为合成消息 |
| `tests/ros_step5_pipeline.py` | 同域实际图像节点、PX4桥、mission、监督器、原生ELF与typed `/fmu/in` 收包 | 实际systemd、Agent、UART、PX4固件 |
| `tests/fixtures/step5_supervisor.py` | 真正 `standby.supervise` 的锁/配置/启动/日志/IPC/状态/停止；唯一IPC连接复用 | 替代ARM/UART预检、全局硬件进程发现、AgentTransport |
| `tests/ros_step5_fault_matrix.py` | 直接调用Stage3驱动，真实原生DDS收包/发布、连续垂向模型 | 相机/mission生产链，真实PX4气动 |

最终环境：**Ubuntu22.04 x86_64、ROS2 Humble、FastDDS localhost隔离域**。本地真实ROS提交及typed接收已证明；PX4端是合成peer，没有SITL/真实固件接收、采纳或物理飞行证据。

`CameraSource`仍是V4L2实际设备入口，开发机无相机设备。本步从生产 `vision_image_node` 的Image入口送合成图像。没有编造曝光时刻、USB采集延迟、真实识别或香橙派性能。

## 3. 两项已复现的P1集成缺陷

### 高帧率图像饿死待对齐帧

`mission_node.py::_on_state()`旧逻辑每次覆盖唯一 `pending_target`。相机快于平台反馈时，新帧总在最新平台采样之后；严格插值等不到后一平台样本，持续有图仍会 `POSITION_IMAGE_TIME_NOT_BRACKETED`，旧目标过期并触发下降。

用实际生产方法和PlatformHistory确定性复现，完整ROS链也出现同根因。修复先消费并保留待帧，仅空槽时接新帧。仍只有一帧、不外推、不续TTL、不改原戳/首次接收。无效帧/来源重置立即撤销；过期仍锁存故障。新增4项专项，均通过。原失败日志保留。

### 独立订阅者姿态先后及降采样不一致

桥与原生各自订阅BEST_EFFORT姿态。旧任务用桥最新姿态编码FRD速度，原生可能尚未收到该样本，精确匹配正确拒绝后转故障下降。完整ROS运行真实复现；末状态里样本后来已到，不能仅靠末快照判断。

首次“查共同历史样本”方案只解决等频先后；独立审查发现raw200Hz/桥50Hz会跳过所需样本，已放弃该方案作为最终实现。

最终新增只读 `NativeFlightSnapshot.msg`：原授权状态、原生已消费的精确NED/FRD `wxyz`、局部原点和首次单调接收时间同包。原生 `publish_native_flight_state()`在既有控制发布边界之后复制/发布；旧NativeFlightState类型/话题保留。mission `_on_native_snapshot/_native_attitude_current/_publish_command`使用这份精确样本编码；同时复核源/接收年龄、进程/任务、reset、原点、重复内容。平台桥仍独立负责图像对齐及实际位置/速度/角速度。

原生64帧精确匹配、原始接收时效及最终边界不放宽。没有拿最近姿态替代、补造戳或给旧数据续鲜。真实raw200Hz请求（实际约177～188Hz）与桥50Hz的四条完整链全部通过。新类型/原生/任务必须成套构建，不能只换Python文件。

## 4. 原有控制行为与差分

本步没有修改高度环、Runtime/Core、人工RC流程、推力限制、安全发布算法或PX4参数。高度仍用本次 `h=z0-z`、融合z/vz闭环到1.5m；目标测量不是自机速度，任务只有水平意图。MTF‑02P仍直连PX4融合。

无视觉实际SystemNode相对本步起点：**1178条控制sink完整字段及顺序逐字节相同，707个控制状态样本一致**。Step3/Step4基线sink差分亦通过。hover/reserve/min/max/slew保持 `0.50/0.04/0.12/0.80/0.40`；`native/config/`与起点无差异。

真实故障仍按原保护：跟踪失效/异常撤销走保护下降，健康新图不自清故障；RC接管/Kill/Offboard退出/反馈无效优先于任务；全DDS丢失锁存并停发。正常通道撤销与故障退出保留区分。

## 5. 最新源码的实际结果

以下计数属于不同层级且相互覆盖，不相加成一个总数。结果及源码摘要见 `evidence/VALIDATION.json`、`SOURCE_BINDING.json`；命令见 [VERIFY_zh.md](VERIFY_zh.md)。

| 检查 | 最终结果 |
|---|---|
| Python相关独立方法 | **216/216 PASS**，0失败/错误/跳过；冻结后重新执行 |
| C++ Debug严格构建/CTest | **88/88 PASS** |
| 真实Humble统一构建 | **4包PASS**，含C++/Python消息typesupport |
| 生成消息布局/嵌套/类型/CDR | **11种PASS** |
| 原生实际tick原子快照 | **1方法PASS**，6个命名检查；新观察不刷新旧姿态接收年龄 |
| ASan+UBSan+泄漏检查 | **2个插桩程序PASS**：实际节点/快照、最终水平边界；非全工程sanitizer |
| 四条服务/原生全流程（其中3条含相机） | **4/4 PASS**：normal、target_loss、no_vision、dds_loss |
| 真实原生水平矩阵 | **11/11 PASS**：10条完整故障下降，1条洪泛后正常LAND |
| 真实原生安全矩阵 | **9/9 PASS** |
| 轻量ROS任务专项 | 水平8、不等频9、原生授权12个命名检查PASS |
| C++相机→mission专项 | **12命名检查PASS** |
| 原有Odometry ROS适配 | **7命名检查PASS** |
| 语法/配置解析、diff空白检查 | PASS |

独立审查另跑36项任务单测通过，是216项的子集。37项专项包含该36项及原生tick1项，也不能再加一次。

四条完整链中，正常/失图/无任务均经历人工RC意图→独立Armed事实→导航槽1→独立Offboard事实→推力爬坡→真实离地/到高门→连续下降→地面→Disarm ACK反例→独立Disarmed事实→176释放→新READY。未自动解锁；再次准备没有继续第二次起飞。

11矩阵覆盖到高前洪泛、重复、乱序、双writer、丢目标/指令超时/来源改变/估计重置/原点变化、水平反馈失效/重置。10条故障不跳模型到地面，持续积分下降；个别故障注入条件恢复仅为保留可用下降反馈，跟踪故障不解除。九项安全覆盖位置过期、姿态/垂向重置、实际nav_state离开Offboard、Offboard控制标志、RC接管、Kill、贴地测距暂不可用及垂速不一致诊断路径。

历史READY、8秒窗口、档位/模式确认、假到高、原始样本不推进，由原生单测/状态回归覆盖；重复帧、旧图、空帧、断图、时钟回退、相机重启及目标多发布者由真实图像专项覆盖。并非每个条件都跑一遍相机全链或真实设备复位。

发布者唯一性：pipeline结束停服前真实graph确认每个控制端点仅一个原生writer；**不是全程逐周期连续监测**。水平输入writer冲突另在最终边界和矩阵覆盖。停服务必须监督器退出0且实际原生PID消失，finally强杀不计PASS；最终无测试控制进程残留。

## 6. 模型与主机实测指标

简化垂向模型为 `a=9.80665*(thrust/0.5-1)-1.4*v`，固定XY，无真实气动、电机滞后、风/地效/噪声；只能检验软件路径。正常全链观测：

| 指标 | 本机合成结果 |
|---|---:|
| 目标高度 | 1.5m |
| 正超调 | 0.13570m |
| 从THRUST_RAMP到原生HeightHold/授权 | 10.026s |
| 到高后约2.6s观察最大/平均绝对误差 | 0.13570 / 0.12549m |
| 正常链最长心跳接收间隔 | 55.385ms |
| 正常链最长控制周期 | 50.474ms |
| 合成640×480 BGR、请求120fps，有效新观测 | 110.90fps |
| 图像host源戳→TargetObservation回调，均值/P95/最大 | 3.869 / 5.037 / 24.727ms |
| raw姿态请求/实际有效率 | 200 / 187.16Hz |
| 水平漂移 | **未测**，XY固定，不把0写成性能 |

正常链各进程场景平均CPU（单核百分比）及最大采样RSS：相机52.15%/93.13MiB，桥22.19%/83.19MiB，mission18.43%/71.36MiB，原生2.63%/43.13MiB，监督器0.72%/23.68MiB。不是active-only或CPU峰值，不能换算OrangePi实时能力。

独立4s相机专项有效新帧115.73fps，P95 4.378ms；不同负载和计数窗口，不能与上表混称同一测量。实际相机曝光/USB延迟、ARM帧率、外参、真实识别率均未测。

原生11矩阵正常发布窗口（Offboard确认到Disarm或故障注入，含边缘）最长心跳54.285ms，9安全矩阵53.315ms；pipeline心跳仅测量，预算断言在原生矩阵。全DDS场景统计5.362s跨越主动停发和重新准备，**不是正常飞行抖动**，不拿它掩盖失联。不能据此声称PX4最后心跳或 `No offboard signal` 已获现场对齐。

完整短链见 [SHORT_TRACE_zh.md](evidence/SHORT_TRACE_zh.md)，带实际输出的103条事件见 `CAUSAL_TRACE.jsonl`。正常链右目标后正roll、左目标后负roll由真实typed输出记录证明；原生诊断源为航点或PX4速度闭环，可与任务封套对应。

## 7. 保护事件、失败证据及独立复核

正常模型下降真实经过原有10s `DESCENT_TIMEOUT → RECOVERY_DESCENT`，最终接地上锁；地面释放出现 `OPERATOR_TAKEOVER_LATCHED` 后重新预流。不能宣称全程普通下降、无超时/无锁存。未改下降参数去美化结果。这是板端下降时间核对事项，尚不能仅凭模型认定真实飞机有缺陷。

全DDS场景证明锁存停发，之后 `test_only_external_ground_disarm_after_feedback_loss` 是夹具清理，不算缺反馈下自主落地证据。

两条历史Stage2完整JSON字节差分仍FAIL：新增27个诊断字段导致。原始失败不改写为PASS；独立结构比较显示全部共有状态字段及全部控制sink相同，见 `NATIVE_FINAL.tar.gz` 内的 `atomic-native-regression/historical-full-json/EXPLAINED_DIFFERENCES.json`。这两项未计入216项或最终通过数量。

早期失败及过渡方案分开保存：`PRE_FIX_FAILURES.tar.gz`、`PRE_ATOMIC_PASS_AND_LIMITS.tar.gz`。包括原生姿态竞态、图像/平台待帧饥饿、默认共享段不足以容纳BGR图像、慢Python像素转换、错误打开第二IPC连接等。后几项是夹具问题：仅测试采用有界16MiB SHM、array('B')、复用监督器唯一连接；生产DDS全局设置和原生单客户端限制不改。快照测试捕获消息初版破坏旧sink插桩，改为显式 `capture_messages=True`；sanitizer夹具目录权限不合格时正确拒绝，修正测试目录0700后通过，初次日志保留。

最终独立代码审查未发现本次差异中新的已确认P1/P2；独立证据审查未发现所声明软件范围内P1假通过，同时明确了唯一writer快照、短定高窗口、合成指标和外部地面清理等限制。两份审查在evidence，不把审查结论扩大为全工程绝对安全。

报告另经独立核对，修正了归档内路径和无视觉场景组名。复验文档七段Shell语法通过；按文档将两套归档sanitizer夹具解到新目录、重新编译并运行，均PASS。这是复验而非增加两项独立测试，日志见 `sanitizer-document-repeat.log` 及两个repeat日志。四个归档均逐成员读回，33份证据文件的SHA256全部通过。

## 8. 待现场验证及下一步交接

- **P1现场启用门**：本候选ARM64编译/运行、真实PX4接收/采纳、链路时钟与融合/重置时序、实际视觉安装与水平动力学、空中失锁退出。缺少真实相机与PX4/SITL证据，不能宣布可直接视觉实飞。
- **P2时效与负载**：原生64帧raw历史、10Hz快照、0.3s时效在板端高频/丢包/负载下需拆桨核实；不得靠放宽门槛处理。
- **P2下降与任务周期**：核实真实下降时间、接地/Disarm/176释放及多轮恢复；本步仅二次READY，未测二次完整任务。
- **P2发行**：维护安装器仍保留视觉false限制；下一步需成套构建消息/控制器/视觉/mission并验证配置继承，历史ARM包不能替代。
- 需要现场相机设备/内外参/目标尺寸/安全间距及采集延时证据、真实运行参数/话题版本、同次PX4 ULog和OrangePi日志。没有可比的空中Position ULog，不编造与用户稳定Position实飞的精度对比。

没有确认遗留的本步源码提交阻断问题；以上风险和证据缺口不因本机PASS而消失。TSan、真实ARM64、实物相机、systemd/Agent/UART、PX4固件采纳和动力测试均未执行。

可复验命令见 [VERIFY_zh.md](VERIFY_zh.md)，文件校验见 `evidence/SHA256SUMS`。本步候选可独立 `git revert 5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`，保留报告证据和用户改动；没有部署，无需动现场旧服务。现场核对仍限拆桨、未解锁；带桨试飞由现场人员单独执行。

# 第6步：ARM64离线发行与维护安装验证

日期：2026-11-18。分支`readability/fc01-chinese-20261111`。最终fix1包已在Ubuntu22.04 ARM64/Humble/systemd虚拟机完成安装、重复安装、无飞机反馈待机、11种统一消息、pending/track/定高切换、启动失败自动恢复，以及恢复原活动/停止状态的手动回退。虚拟机不是实际Orange Pi Zero 3W，结果不代表相机、PX4接收或实飞通过。

## 源码与交付身份

- 本步起点：`19a5aa8c191feccc6cab7a7d274b8d1900a5443f`。
- 本步发行代码：`c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac`。
- 原生及target业务候选：阶段5的`5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`。
- 维护行为沿用已在实机跑通的`41e1532`安装流程。此前`4c85a9e`发行、`adc55d0`原生候选是历史身份，不能替代本包的原生/target新消息构建。
- 本步未修改`native/src/`、`native/config/`、`target/src/`、`standby.py`或`prebuilt_boot.sh`。已编译的原生与target输入逐文件绑定当前候选；`ARM64_BUILD.json`覆盖59项原生输入、52项target输入及全部安装树文件。
- 实际ARM构建时的发行脚本身份为`6a9d0b3`；其后`05aff4a`、`c6fe7ed`只修正发行/维护软件。111项实际编译输入与当前候选逐字节一致，保留`ACTUAL_BUILD_SOURCE_BINDING.json`的真实历史构建身份，不冒称用c6fe7ed重新编译了未变动的控制器。最终包的发行源码身份仍为完整`c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac`。

实际文件位于`/home/jikl62/FC01_OrangePi_Navigation_20261028/dist/arm64/20261118_vision_step6_fix1/`：

| 文件 | 已生成的身份 |
|---|---|
| `FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz` | 196394133字节，SHA256 `06e596b44b5aad5b8dab79a3eafe44147c0abd6f8d94495dcf25e2cebcebb830` |
| `install_fc01.sh` | 配套独立维护安装器，SHA256 `076a1431fe4a5dbbc8498e80e1ac3e44598df29272757ae0430e0f7f8de7c75f` |
| 两份同名`.sha256` | 固定对应包和安装器，不能与旧包混用 |
| `DELIVERY.json` | 源码、控制候选、大小、SHA、文件数的机器可读记录 |

操作说明为[FC01_ARM64_INSTALL_zh.md](../../FC01_ARM64_INSTALL_zh.md)，现场来源说明为[FC01_VISION_SITE_CONFIG_zh.md](../../FC01_VISION_SITE_CONFIG_zh.md)，模板为[FC01_VISION_SITE_TEMPLATE.json](../../FC01_VISION_SITE_TEMPLATE.json)。包内同时保留对应源码与这些说明。历史测试/审查归档和未跟踪UART工具没有塞入运行载荷，也没有从仓库删除。

## 本轮实现

`build_arm64_bundle.py`收录原生、统一消息、视觉、mission、PX4桥、生产launch和必需脚本。源码来自Git已提交字节，原生/target与候选逐字节校验；实际安装的Python、配置、launch也必须匹配源码，不能仅给旧产物换摘要。`__pycache__`/pyc不进入包，重复typesupport被拒绝。保留原有路径、ELF、文件数、单文件及总容量校验，没有关闭安全检查。

新增`build_arm64_runtime.py`在Ubuntu22.04 ARM64/Humble中执行真实ldd，记录依赖包版本、来源、版权和SHA，复制到独立可迁移前缀。包内含OpenCV/cv_bridge C++运行库、NumPy、ROS launch/ros2cli及所需Python间接依赖；不替换目标Ubuntu的glibc、加载器、libstdc++或基础Humble。目标机仍需原有Humble、Python3/YAML、systemd、sudo、psmisc和已有Agent/UART环境，不需要为本包重新编译或联网安装上述视觉依赖。任意运行时dlopen插件无法只靠ldd完全枚举，仍须实际搬迁启动验证和现场相机验证。

`navigation_prebuilt_install.py`保持软件验收与飞行就绪分离：校验包/本机依赖/配置/unit，保留旧runtime和DDS，停止旧服务并确认实际旧控制器消失，再启动同一服务的新入口。验收核对唯一原生进程的路径、摘要、参数和DDS环境。明确启用track时还核对本服务树内的相机、桥、mission、只读接口四个辅助进程，**不要求任何图像、PX4、RC或DDS反馈**。失败按既有事务恢复旧服务配置和运行状态。`FC01_PREBUILT_INSTALL_OK`是软件切换成功；`FC01_FLIGHT_READINESS`独立报告运行准入缺项。

`navigation_prebuilt_service.py`由原`standby`独占原生控制器，同一服务额外拥有视觉launch进程组与只读状态线程。无相机时相机节点保留重试/失效状态，mission等待可靠输入；辅助进程故障不会从服务管理侧强杀正在运行的原生控制器。服务退出先执行原生安全停止，再有界清理辅助进程组。日志进入systemd journal，避免新增无界文件日志。定高模式不启动视觉节点，只刷新当前监督器身份。

新增`navigation_vision_mode.py`只在服务停止、实际无原生进程的维护状态切换altitude/track，与服务共用配置锁。配置备份、原子写入、本机软件检查、失败恢复和安装runtime摘要同步齐全。track要求现用1.5m/槽1、显式配对开关及真实现场来源记录；仅修改视觉开关与收紧水平限制，不修改RC/推力/DDS/高度。版本迁移只搬迁已校验的现场记录，发生在停旧之前。未测标定模板保持null/false，不能当作现场校准。

`navigation_status.py`与`navigation.py`增加只读视觉状态。监督器身份绑定boot/PID/单调时钟，父状态不替子观测续鲜，重复发布者或过期状态不显示当前授权。原生真实到高授权与目标锁定分别呈现，不用任务计时、历史READY或旧JSON推定飞行许可。

## 已复现并修复的问题

| 问题与级别 | 根因、修复与证据 |
|---|---|
| P2：依赖扫描重复库误报变化 | 首次记录已加入package，后续比较时未加，导致相同字节字典不等。实际ARM扫描及确定性单测复现；改为完整来源记录后比较，增加重复库回归。 |
| P2：track安装可能漏过辅助程序崩溃 | 原验收只检查原生。现额外查实际launch树及四个辅助节点，拒绝死leader、缺桥、错身份和多实例；允许空传感器观测，补失败恢复回归。 |
| P2：新只读观察线程使用错误ROS Context | 全局spin_once隐式创建默认executor，与独立Context不匹配而启动即异常。改用同Context的SingleThreadedExecutor；真实x86 Humble无相机/无DDS启动、观察和退出通过。 |
| 兼容性：合法NumPy扩展无DT_NEEDED | ldd把无依赖共享对象输出为statically linked。仅在ELF独立证明ET_DYN、无PT_INTERP、有界PT_DYNAMIC且无DT_NEEDED时接受空依赖；静态程序、静态PIE、缺库和损坏ELF仍拒绝，新增回归。 |
| 依赖缺口：最小Humble无ros2 launch/OpenCV | 实际构建机/虚拟机核查发现，改用随包CLI及私有依赖闭包，不能假设目标机已装桌面ROS全集。搬迁导入已通过，最终软件事务结果见下节。 |

早期独立审查所指的track辅助程序验收及只读ROS Context问题已经修复；最终独立代码审查执行106项相关测试通过，模式审查的19项属于其中及下列回归的重叠子集，不重复相加。仍有一项安全间距配置P2未关闭，单列于末节；不把“未发现本轮P0/P1”写成全部问题已解决。

## 已执行验证

不同层级及相互覆盖的计数分开列出，不相加为一个虚构总数。命令与环境见[VERIFY_zh.md](VERIFY_zh.md)。

| 检查 | 当前实际结果 |
|---|---|
| 最终安装、状态、模式、包、资源及载荷合同Python回归 | **180/180 PASS**，14.450s，最新已提交源码；`python-final-fix1.log` |
| 原有监督器/传输/有界日志专项 | **26/26 PASS**，9.000s，真实子进程/IPC回归；Agent恢复使用替身、不是实际XRCE会话 |
| ARM64真实用户态Humble构建 | **4包PASS**，11min34s，GCC11/Release：原生、mission、vision、bringup |
| 构建输出 | vision有`PYTHON_EXECUTABLE`未使用的CMake警告；非构建失败 |
| ARM64实际依赖扫描 | **89个输入ELF**，278个来源包、1700项外部运行文件，成功 |
| 私有可迁移安装树 | **7582文件**，2187项复制记录，59项基础目标依赖记录 |
| 发行独立解包与摘要/路径/清单核验 | **PASS**，7748项payload加1份发行manifest，共7749文件 |
| x86 Humble实际辅助链 | **PASS**，无相机/无DDS时四节点待机、无目标锁定、无原生到高授权，完整清理 |
| ARM64虚拟机不同目录Python导入 | **早期包PASS**：NumPy、launch、launch_ros、ros2cli、ros2launch、mission和PX4桥；最终fix1另由完整预检/实际辅助启动验证 |
| 最终fix1包ARM64 systemd安装 | **PASS**，输出`FC01_PREBUILT_INSTALL_OK`，Supervisor25096/native28390 |
| 同一包重复安装 | **PASS**，Supervisor25096/native28390不变，保留原回退点 |
| 无DDS飞机数据待机 | **PASS**，六路回调均0，原生IDLE、无运动授权；独立30秒ROS订阅观察输出均0 |
| 手动回退 | **PASS**，输出`FC01_PREBUILT_ROLLBACK_OK`；原版active、唯一原生进程、runtime字节恢复 |
| 最终包11种统一消息 | **PASS**，生成类/字段/源码身份、typesupport与本机CDR往返；不创建ROS节点或发布者 |
| diff/AST/语法 | PASS；未改原生/target控制输入 |

原统一消息检查脚本第一次因测试辅助模块`rosidl_runtime_py`缺失而失败，原日志`vm-final-11-messages.log`保留。生产入口及消息typesupport不依赖该辅助模块；第二次仅将测试脚本的`get_message`改为直接加载包内实际生成类，其余来源、字段、默认值、typesupport及CDR检查保持执行，11种全部通过，见`vm-final-11-messages-direct.log`与`fix1-unified-11-types.json`。未向目标VM临时安装该模块掩盖缺项，也未把修改后的脚本当作原脚本原样通过。

阶段5的216项Python、88项CTest、四条全链与故障矩阵是原候选先前证据，**不是本步新执行数量**。本步没有改控制器/target业务算法，没有把先前测试写成本次重跑，也未重新声称ASan/SITL/物理飞行通过。

## 原有控制边界与现场待测

本轮发行不改变人工RC解锁、8s槽1、实际Offboard确认、`h=z0-z`融合z/vz高度环、1.5m目标合同、推力包络、RC接管、Kill、失联保护或安全锁存。安装继承实际生效参数；若旧现场高度并非1.5m，不会为了通过而静默覆盖。虚拟机旧1.23m是刻意的继承测试输入，后续track已在明确标记的软件测试1.5m配置上验证。

视觉默认关闭。现场若只有串口MicoVIO自机里程计，没有V4L2目标图像源，不能宣称目标跟踪已可飞。MTF-02P继续直连PX4融合，本包不改EKF2或PX4参数。真实设备权限/帧率/延迟/标定、安全间距、Zero3W负载与控制周期、UART/DDS质量、实际PX4消息版本、真实到高与视觉飞行表现均待现场确认；无虚拟机结果可替代这些项目。

未推送、未改main、未操作实际香橙派/PX4、未解锁或启动真实动力。原有未跟踪`tools/`及两份MicoVIO测试保持原样，未纳入提交。

## 最终ARM64虚拟机维护事务

环境为QEMU system-aarch64、Cortex-A72/2核/2GB、Ubuntu22.04.5 ARM64、Humble、systemd249及5.15通用内核。无真实PX4、相机或UART；测试Agent进程/PTY无实际XRCE会话。此项只覆盖软件安装和待机，不作为真实MicroXRCEAgent连接或Zero3W负载测试。

| 同一最终摘要包的事务 | 已核对结果/证据 |
|---|---|
| 初装，保留旧服务/参数 | PASS；`vm-install-fix1.log`、`vm-runtime-inheritance.log` |
| 旧控制器退出后启新 | PASS；旧Supervisor21502/native22053正常退出后，新Supervisor25096/native28390才启动；安装器逐身份等待退出 |
| 重复安装不另起实例 | PASS；`vm-repeat-fix1.log`，进程与原回退点不变 |
| 无飞机反馈成功待机 | PASS；`fix1-no-feedback.json`、`vm-fix1-status.log`、`no-feedback-ros-final.json` |
| 统一消息及运行依赖 | PASS；安装器完整预检、11消息直接生成类/CDR检查日志；原脚本缺测试辅助模块的失败证据保留 |
| 手动回退并恢复原活动状态 | PASS；`vm-manual-rollback-fix1.log`，旧版唯一进程及原runtime恢复 |
| 软件启动故障自动恢复 | PASS；`vm-failure-recovery-fix1.log`，候选ExecStartPre退出23、自动重试受600秒窗口约束，随后恢复原活动服务/唯一旧native/runtime逐字一致 |
| 未标定pending无相机待机 | PASS；最终包实际唯一原生及4辅助、IPC推进，camera/mission观测当前；标定5缺项、mission控制false、无锁定/授权 |
| 有来源track无相机待机 | PASS；VM-only synthetic现场记录、mission true，实际4辅助与唯一native、无锁定/到高/飞行授权；不代表现场标定 |
| 模式切换后重复安装 | PASS；track保留Supervisor52149/native55665与原回退点，实际4辅助唯一 |
| 原服务inactive安装 | PASS；无旧native，安装235.5秒软件验收、目标1.5m，六路反馈回调0 |
| track切回altitude并重新待机 | PASS；300.8秒软件验收，原视觉4辅助全部退出，两个原生视觉开关false |
| 回退恢复原先inactive | PASS；`FC01_PREBUILT_ROLLBACK_OK`，旧目录/MainPID0/无native/runtime逐字一致，完整suite退出0 |

进程监视器以1秒采样记录17次转换的辅助证据，采样中均未重叠且换PID间存在空窗；监视器已停止。不能据此声称获得内核逐事件轨迹；严格的停旧在先顺序由安装器PID/start_ticks归属检查与`stop_and_wait`事务保证。原辅助脚本注释中的0.25秒不是当前实际采样频率，最终引用以实测1秒为准。

## 耗时复现与修正

首个6a9d0b3发行在ARM64 VM真实复现180秒启动预算不足，安装返回失败并自动恢复旧版；原始`vm-install-1.log`保留。05aff4a将软件预算改为600秒并消除相同目录库别名重复ldd，c6fe7ed继续补进度、实际进程归属清理、推进IPC验收和未标定诊断待机。同一安装事务的完整ldd集中到必经的配置检查，不从硬件反馈取安装许可。

最终修正版`20261118_vision_step6_fix1`、SHA06e596...实际安装已打印INSTALL_OK：Supervisor25096/native28390；旧native22053及Supervisor21502先正常退出，等待60.9秒；新版完整检查170.9秒，含监督器再次核对清单的总初始化/IPC验收244.4秒，仍在600秒有限预算内。停旧前检查145.1秒。拆包清单103.7秒。这些是TCG虚拟机实测耗时，不能代替Zero3W性能。

旧VM故意设置目标1.23m，安装逐字保留目标/RC/DDS/推力；不是本包把目标改为1.23m。后续视觉测试已明确单独建立软件测试专用1.5m基线，现场安装仍保留实际值。没有飞机数据的六类回调均0，原生IDLE、navigation_allowed不成立。真实ROS只读30秒观察无设定值/命令/心跳（均0），无/fmu/out发布者，证明该断反馈场景未产生飞行动作；不证明PX4实际接收。

## 未解决项与最终资料身份

已确认未修P2：site `standoff_m`可大于`horizontal_max_radius_m`，配置工具目前接受、运行任务明确拒绝`STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE`。保留运行保护，不为首测放宽。现场必须填写`standoff_m <= horizontal_max_radius_m <= 2m`。任意现场相机参数的C++初始化约束并未全部前置到配置工具，必须检查实际节点启动状态；`FC01_VISION_MODE_CONFIG_OK`不代表标定真实性或任意相机配置都能初始化成功。

`track --pending-calibration`会启用配对原生视觉/水平开关，但mission的`control_enabled=false`，只作诊断待机。普通定高首测应明确选择`altitude`；有真实来源记录的`track --site`才将mission控制开关置为true，运动仍受原生真实到高和新鲜反馈门约束。

本报告与证据作单独本地提交；其完整SHA见交付回复或`git log -1 --format=%H -- docs/investigations/20261118_vision_step6`。资料提交与包内源码`c6fe7ed`分开；仅补报告不重贴发行SHA，不重打包已验证字节。工作区保留用户原有未跟踪UART工具及测试，不宣称整个仓库无未提交文件。

故障注入时继承单元的Restart=on-failure/30s让状态持续auto-restart，未呈终态failed；安装器按600秒有限预算判失败并自动恢复。此场景未提前终止重试，完整原始日志保留。原版启动恢复另用约90秒软件初始化，不要求飞机数据。

pending模式完整软件启动/验收实测376.8秒，在600秒窗口内；定高从停止状态启动235.5秒。TCG仿真中辅助ROS进程与监督器共享CPU会延长清单检查和预检；没有调低进程身份或数据时效门槛追求通过。

有来源track在VM软件测试配置下完整启动408.8秒，通过当前服务树的4辅助/1原生与IPC推进验收。camera为CONFIGURATION_CONFIRMED但CAMERA_FRAME_STALE_OR_DISCONNECTED，mission为WAIT_NATIVE、target_locked=false、control_authority=false。confirmed仅描述VM测试文件确认字段，不是真实相机校准。

track重复安装完整预检325.2秒，已校验文件/消息/565 ELF/338独立ldd，软件活性再验4.7秒。随后真实ROS只读30秒观察setpoint/command/heartbeat均0，见track-no-feedback-ros-final.json；与定高无反馈结果一致。

辅助链停止观测：原生先正常退出，随后进程组SIGINT清理视觉/桥/mission；两份Python节点在destroy_node时记录KeyboardInterrupt、launch显示exit -2，最终unit正常deactivated且进程退出。此为维护关停日志噪声P3，未伪称所有子节点均退出0；不影响事务进程退出检查。复用的VM Agent按既有所有权保留，不能把systemd的left-over Agent提示解释成残留FC01控制节点。

## 最终结论

同一SHA最终包的软件交付条件已完成。`vm-mode-lifecycle-fix1.log`末尾为`FINAL_FIX1_ARM64_MODE_LIFECYCLE_SUITE_PASS`；最后VM恢复旧`20261117_stage3_fix1`目录、inactive/MainPID0、零原生进程和原始运行参数。最终交付目录及命令见[操作说明](DELIVERY_OPERATIONS_zh.md)，不需要香橙派重编译。

本轮没有新增已确认的P0/P1安装缺陷；上述P2间距关系检查和P3关停日志提示保留。阶段5的现场启用门、64帧历史/时效负载、真实下降时长及多轮恢复仍需现场；本轮ARM64软件构建/待机已覆盖阶段5发行缺口，未覆盖真实PX4接收/采纳或飞行。此前模型曾进入DESCENT_TIMEOUT/RECOVERY_DESCENT，不能把它写成已证明实机正常下降。

第一至第五步测试继续引用原报告，不累计为本轮新通过数量。本轮实际为180项发行相关Python、另26项监督器/日志Python、4包ARM64构建、11消息类型/CDR、同一最终包的上述VM事务。未再次运行全工程ASan/CTest/SITL，未测真实相机恢复、实际XRCE/UART重连、Zero3W性能或带桨飞行。

安装只验证软件条件；默认定高视觉关闭，飞控断电/无相机/无RC/DDS可以待机。接入硬件后运行程序持续独立判就绪，历史READY不能作许可。现场操作者按新鲜READY→人工RC解锁→8秒内槽1→PX4实际Offboard→1.5m真实稳定到高的原流程验证，水平track还要求实际标定、新图与可信反馈。

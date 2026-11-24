# 第4步独立审查记录（截至2026-11-18 03:21，尚待最终ROS结果）

审查者只读检查实际生产代码，没有提交、部署或操作设备。开发基线2948bfe4e6a26b5ee58bf02e4e9988e484fad81e。工作区存在其他人的未跟踪tools/与MicoVIO测试，不属于本轮修改。

## 已核对的实现

- 原生新选择器明确使用PX4实际x/y/vx/vy和姿态；任务目标和实际自机状态分别校验。
- HorizontalVelocityCommand为独立生产封套；原V2布局、生成时间语义和诊断端点保留。封套独立携带原姿态采样戳和完整PX4原点。
- 完整FRD三维旋转后取NED XY，保留合法水平请求在倾斜机体中的bodyZ补偿；纯bodyZ请求为明确的零水平策略例外。
- 单一确定性仲裁，当前有效跟踪优先；航点正常撤销为有限刹停，跟踪曾开始后真实撤销/过期/缺匹配姿态不能回退到旧航点。
- 任务只产生水平与期望航向速率；目标z、航点z、视觉vz和模拟任务高度没有原生垂向权限。原生target=1.5m保持。
- 最终边界重验实际快照、选中指令、撤销代数、原始源与接收TTL；不在边界检查中再次积分偏航。
- wz正向为NED顺时针期望航向变化率，区别于PX4实测机体系角速度；零值保持已提交航向。
- 参数与入口默认track，默认任务/视觉/新水平选择器关闭；首测launch要求现场配置，没有冒充已标定的安装旋转/间距。

## 本次审查确认并促成修复的缺陷

1. 跟踪撤销后旧航点重新接管：独立纯C++反例，已修复且反例PASS。
2. 精确姿态匹配的原始接收TTL/原点不完整：结构与最终检查已补齐。
3. 命令先到、匹配姿态后到导致同sequence派生字段被误认篡改：线上载荷与派生匹配证据分开比较，测试PASS。
4. 未选通道变化使正常等待/跟踪最终计划失效：选中依赖检查，保留实际状态/撤销严格性。
5. 相同原姿态样本重复外壳可覆盖历史q；ref原点A→B→A；指令实际DDS writer更换；进程期seen_valid令后续任务invalid不断撤权；递增sequence下frame/origin A→B→A；相同raw local样本XY/VXY改值A→B→A：已逐项在当前node源码核对修复。
6. 高度暂离稳定窗时停止调用水平状态机，导致同周期跟踪故障不走下降：实际Runtime独立复现前exit134，修复后同一反例exit0进入Descent，且没有放宽水平授权。
7. Python min/max可将NaN偏航内部量裁为有限满速：任务生产路径先拒绝NaN/Inf并撤销，新增回归。
8. 初始invalid空原点/producer reset锁定会话：Core已改为首个valid前不锁无效身份；node已增加每通道初始化策略。尚需最终真实ROS晚启任务回归；不得清掉实际PX4撤销代数。

## 独立执行/检查的证据

- 独立执行13项Mission实际方法回归PASS；检查38项完整Mission测试与8项真实Humble任务收发证据。最近真实ROS日志打印了当前源码绝对路径，避免旧安装缓存冒充。
- 独立执行新水平纯算法与完整Runtime测试PASS；最新88/88 CTest证据PASS。
- 独立编译并执行跟踪撤销/旧航点反例，以及高度门关闭+跟踪撤销实际Runtime反例，确认修复。
- 检查ASan+UBSan纯算法/Runtime通过日志。
- 检查无视觉26371行确定性输出完全一致的三组SHA和字节比较证据；原生子代理已继续对最终修复后源码重跑。
- git diff --check通过。

## 交付前仍须汇总

最终Humble构建、最新源/ELF身份、真实原生ROS路径正常与故障场景结果、初始任务启动/多轮重准备结果、最终本地提交。NavigationCommand.msg仍需补充新选择器下frame/reset/XY语义的中文注释。

本轮没有真实ARM64、PX4、相机、Position ULog或飞行性能证据。软件方向/权限测试不等同实机水平稳定能力；高度性能、漂移、延时、相机内外参和安全间距仍须现场验证。不得宣称可以直接带桨启用候选。

03:24追加：独立从当前fc01_system_node.cpp抽取四个真实方法，编译为仅内存夹具（没有ROS/硬件），7项回调检查PASS：NAV初始空原点→正确原点不撤权、首valid建立身份、重复NAV不续鲜、NAV frame A→B→A保留两次撤销、速度封套初始空原点→真实原点不撤权、已tracking后连续invalid只在true→false撤权一次、同raw姿态不追加/续TTL且改q撤权保留原q。抽取生成脚本/源码/输入SHA证据位于/tmp/fc01-step4-node-callback-review.{py,cpp}及-source.json，执行文件输出ACTUAL_NODE_CALLBACK_REVIEW_PASS 7 named checks。这验证已修bootstrap回调策略；不等同完整ROS晚启mission场景。Core对任意实际PX4代数改变保留拒绝是预期保护，不再把inactive会话初始化计为该类事件。

03:26追加：实际原生Humble入口最新normal场景已PASS（/tmp/fc01-step4-complete-normal/result.json），全部source/ELF/input hash门通过。8项水平wire检查包括无相机等待、航点XY、跨周期撤销刹停、左右横移、4度俯仰完整三维精确投影、纯bodyZ零水平、±wz与0保持；走过正常降落/接地/Disarm/释放Offboard/新任务准备。该简化垂向模型超调0.1394m、最大心跳接收间隔0.0530s；XY固定输入，漂移0不能作为性能结论。完整故障矩阵仍在运行，尚不将它提前标为通过。生产源码已冻结，余下只做证据审查；NavigationCommand消息旧注释P3记录，避免为非行为注释造成已验证构建身份不一致。

03:28故障矩阵审查：已有track_timeout/source/reset/origin、horizontal_feedback_invalid通过。track_lost及horizontal_reset首次运行在故障注入前的航点阶段失败，未覆盖所请求的故障。独立检查states发现NAV sequence固定落后速度封套19帧并以20Hz单步前进，约0.95s缓存时延；fixture writer为reliable depth20，生产命令应depth1。已建议按生产QoS修测试发布者重跑，不放宽源TTL。旧失败证据必须保留，用作积压命令被拒的事实；根因在没有收发时间和matched transport确认前不可绝对断言。

03:33追加：NavigationCommand.msg已补充双合同中文说明，旧布局未动。最新4包Humble构建通过；10种生成接口本地CDR、12项真实原生授权任务通信和8项真实水平任务通信均通过。测试fixture指令writer已修为depth1，新的verified-matrix正在从normal开始全部重跑。旧矩阵offboard_loss只有inputs_unchanged_during_run=false（期间构建身份文件更新），实际故障动作门全通过；仍必须以冻结后重跑结果为准。

03:34追加：修正fixture指令writer depth1后，verified-matrix normal及track_lost均通过；track_lost确实完成8项运动wire检查后注入invalid，原生同次场景进入DESCENT、VISUAL_RECOVERY_HORIZONTAL_INPUT_REVOKED。所有时序/source identity gate均true。最新88/88 CTest完整结果已查。

03:38最终独立复核：`/tmp/fc01-step4-verified-matrix` 的12/12场景均PASS：normal、track_lost、track_timeout、track_source、track_reset、track_origin、horizontal_feedback_invalid、horizontal_reset、rc_takeover、kill、offboard_loss、feedback_stale。逐一读取实际result.json，所有9项gate均为true，包含冻结源码/ELF/夹具身份、单调wire、target/heartbeat配对和心跳窗口检查。每场景8项水平实际wire检查均完成；7项水平故障附加撤销检查通过并真实进入DESCENT。normal经历下降、接地、Disarm、释放Offboard、第二次新READY；Kill/模式丢失/反馈过期保持安全锁存，RC接管先既有受控交接再锁存。

重要覆盖边界：7项水平故障矩阵验证至进入DESCENT及XY/yaw撤销；随后夹具清理将模型位置立刻归地面，记录有UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP→SAFETY。因此不得声称这7项场景已经完整受控下降到接地。完整正常降落由normal独立覆盖。

再次独立按字节重算ROS_BUILD.json绑定的52个生产输入SHA256，全部匹配；实际ELF SHA256也匹配。normal模型指标为超调0.139893m、斜坡开始至稳定窗入口7.91222s、至HeightHold10.1814s、后续3.35906s观察窗最大误差0.139893m/RMS0.132082m、最长心跳接收间隔0.0539543s。此时观察窗口尚短，XY固定零，无横向动力学/扰动/真实相机，不作为硬件性能或水平稳定证明。

独立审查结论：当前第4步软件候选在上述验证边界内通过，没有发现需再阻断本地候选提交的问题；维持默认关闭，缺现场标定/安全间距/航点原点时拒绝开启是预期行为。尚缺真实ARM64/PX4/MTF融合/相机时间延迟/安装外参/真实航点原点与飞行日志，不得据此直接宣称实机可飞或交付已完成的ARM64安装包。

# 第5步最终独立只读复核

复核当前工作区，不修改仓库文件、配置、设备或服务。实际执行 `PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_mission_horizontal tests.test_mission_pending_alignment tests.test_mission_native_gate -v` 全部通过（原始日志 `/tmp/fc01-step5-final-review2/targeted.log`）。

## 当前证据已确认

- 四条最终贯通测试 normal / target_loss / no_vision / dds_loss 均PASS，分别8/9/6/8具名检查，源码mission SHA831656efa8518603ebccebfd93d3139ee6a1fe0dadef98b43dd021a3d8563b29与四项一致，测试driver SHA0c7833d7407c1c3484bb217c7fedb17cda4380623476da4ed90443d10bb2ffd5与当前文件一致。
- 固定单个等待时间对齐目标的修复保持原始源戳/接收戳与TTL，不外推、不积压；无效、来源变化、过期依旧先撤销。安全回归可验证。
- 使用NativeFlightState已收到姿态的精确戳再从平台历史找同一样本，解决了已测同频50Hz下桥先到、原生后到的race，没有放宽原生最终匹配或TTL。
- 单IPC所有权正确保留，测试只给监督器原连接添加观察tee与有界请求中继；真实监督器spawn/monitor/stop执行，但ARM/UART预检、Agent和全局进程发现替换，不是systemd或设备验收。
- Stage3测试扩展通过可选flag直改既有驱动，旧默认分支没有暗中启用完整故障下降；wrapper不再生成/exec源文本。完整故障下降观测和最后失败清理跳地面已区分。
- dds_loss只验证真实原生锁存及停止实际DDS输出。随后external ground/disarm是测试收尾，不能计入自动落地或受控下降证明。

## 仍有需要处理或显式列出的可用性缺口

**P1候选可用性问题：PX4 raw姿态频率高于桥快照时，当前精确查找可能没有共同样本。** PX4StateBridge只每50Hz发布最新姿态，任务PlatformHistory只保存这些快照；native10Hz状态回报的已接收raw样本可能是200Hz原始流中被桥快照跳过的一帧。原生与桥都收到并验证过该姿态，任务却没有它的精确q，代码会发valid=false并触发跟踪保护下降。

已用实际当前Mission `_publish_command`确定性执行复现，脚本 `/tmp/fc01-step5-final-review2/unequal_attitude_repro.py`，结果 `/tmp/fc01-step5-final-review2/unequal_attitude_result.json`：当前原生样本15ms、平台最新10ms，所有门有效，但精确样本缺失→validfalse。等频合成50Hz四项贯通PASS不能覆盖此条件。保护动作本身正确；问题是跨频率协调尚不完整，不能宣称相机可直接稳定辅助实飞。

建议完成一项可验证设计：发布原生确认的精确姿态q与其来源/reset/stamp（版本化消息或独立只读契约），或让任务取得桥接器所有已验证原始姿态的有界历史。不能取最近样本/插值冒充原生q，也不能放宽TTL。若本步不继续修改，报告必须列出该未解决问题和200Hz raw / 50Hz桥 / 20Hz native / 10Hz状态的ROS复现待项。

缺共同样本当前只会validfalse，状态页仍可能显示TRACKING_CURRENT，建议明确 `NATIVE_CONFIRMED_ATTITUDE_NOT_SHARED_OR_STALE` 作为诊断。此项不涉及放宽授权；可与上面的修复一起验证。

## 其他报告边界

正常下降仍有DESCENT_TIMEOUT→RECOVERY_DESCENT，地面释放有OPERATOR_TAKEOVER_LATCHED后恢复；二次任务只到READY。水平XY固定无真实动力学，不能称0漂移。没有当前ARM64、V4L2采集延时、真实PX4采纳、真实飞行/外参结论。默认visual=false和安装器未启用视觉仍应保留，后续发行不能掩盖缺口。

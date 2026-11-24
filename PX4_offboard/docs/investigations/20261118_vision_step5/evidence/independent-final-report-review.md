# Step5 最终报告与复验命令独立核对

只读核对时间：2026-11-18。当前代码提交 `5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`。未改生产代码、测试、配置或报告，未提交、部署。

## 结论

报告中的主要计数、正常链指标、证据层级与已存最终证据一致。未发现需要阻断本步候选交付的P1/P2报告错误。`VERIFY_zh.md`七个bash代码块均通过 `bash -n`；本次没有再执行长场景或sanitizer，因此不宣称重新证明运行结果。

核对内容：

- Python216独立方法、CTest88、四个Humble包、11消息合同，与最终结果一致；重复专项/独立审查未加到总数。
- 四链均PASS；11矩阵恰为10条连续故障下降与1条洪泛后正常LAND。报告没有再把11条都叫完整故障降落。
- 正常/失图/无任务连续模型下降、ACK不能替代Disarmed事实，以及新READY而非第二次起飞，报告均明确。
- 全DDS场景的外部地面/上锁是清理，不被计为自主落地；5.362秒心跳统计解释正确。
- 唯一writer是停服前graph快照，不冒称全程逐周期监控；报告已写。
- 正常链高度超调0.13570m、HeightHold10.026s、观察窗约2.603s、误差0.13570/0.12549m、心跳55.385ms、周期50.474ms、有效帧110.90fps、图像时延3.869/5.037/24.727ms、姿态187.16Hz均与result一致。
- 固定XY不被称真实漂移0；CPU/内存、合成源戳时延不被扩大成ARM/相机性能。
- 既有DESCENT_TIMEOUT/RECOVERY_DESCENT、OPERATOR_TAKEOVER锁存、两条历史JSON失败均保留，不改写成无保护正常落地或测试全PASS。
- 当前无ARM64/实物相机/PX4采纳/systemd/Agent/UART证据，以及旧ARM64验证不能背书本候选，报告明确。
- 103条因果事件、78成员NATIVE_FINAL归档与294成员ROS_FINAL归档存在；代码SHA和日志层级说明一致。

## P3 文档小修正

1. REPORT第7节的归档内路径写成 `NATIVE_FINAL.tar.gz/historical-full-json/EXPLAINED_DIFFERENCES.json`，实际成员是 `atomic-native-regression/historical-full-json/EXPLAINED_DIFFERENCES.json`。建议补完整内部前缀，方便直接查找。
2. 结果表“**四条完整相机/服务/原生链**”作为组名略宽：`no_vision`刻意不启动相机/mission/bridge。正文和VERIFY已准确说明，可将表行改成“四条完整链/无视觉基线场景”，避免只读表格时误解。非证据失败。

复验命令的独立归档sanitizer socket路径约100字节，小于Unix108字节限制；没有发现明显路径长度阻断。未执行命令中未包含的板端流程。

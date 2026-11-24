# 第6步最终证据独立审查

2026-11-18，只读核对最终包、报告和真实VM事务证据。未操作VM、服务或设备，未修改源码/包，未提交。

## 核对通过

- 安装包实算SHA256为06e596b44b5aad5b8dab79a3eafe44147c0abd6f8d94495dcf25e2cebcebb830；安装器实算SHA256为076a1431fe4a5dbbc8498e80e1ac3e44598df29272757ae0430e0f7f8de7c75f，与FINAL_DELIVERY.json及报告一致。
- 发行源码c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac与原生/target候选5c4dc14分开报告；历史构建6a9身份未改贴。
- 180 tests/14.450s及另26 tests/9.000s对应实际日志；阶段5证据不重复计入本轮。
- vm-mode-lifecycle-fix1.log包含inactive安装、pending、synthetic-source track、track repeat、return altitude、辅助退出、inactive rollback与最终suite PASS。结尾实际成功回退，控制器退出等待60.8秒。
- 最后快照为旧20261117_stage3_fix1目录、inactive/MainPID0/controllers=[]；回退前后runtime字节一致。
- fix1-transition-assertions.json为17次1秒采样转换，unique=true且switch之间观察到空窗；报告清楚说明采样不能替代逐事件证明，严格排序靠安装器stop_and_wait。
- unified-11-types报告passed=true、11消息。保留原rosidl_runtime_py测试辅助缺项失败，与直接生成类/CDR通过分开，没有补装伪装。
- pending为mission false，confirmed测试明确synthetic VM来源，现场不能复制；真实相机、PX4接收、飞行和Zero3W性能仍待测。
- 已知P2间距/半径关系、P3SIGINT关停日志均保留。默认视觉关闭与runtime就绪门不被安装成功代替。
- 操作命令与安装/模式CLI一致，回退从真实INSTALL_STATE读取backup；文档不硬编码VM备份给现场。

## 本次仅文案澄清

REPORT_zh.md的不同目录导入行明确标为早期包PASS，最终fix1另有完整预检和实际启动；把后续track的计划式措辞改为已在标明VMfixture的1.5m配置验证。其余未改。

## 结论

未发现本报告声明范围内的P0/P1假通过或交付身份冲突。可以提交证据并交付当前冻结的最终包。实际硬件条件仍按报告待测，不得宣称实飞通过。

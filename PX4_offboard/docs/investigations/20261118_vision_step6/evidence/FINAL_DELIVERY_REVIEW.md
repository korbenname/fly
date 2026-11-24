# 第6步最终交付物独立复核

2026-11-18；只读源码、最终包与文档，未编辑生产文件/包/VM，未提交。

## 实核通过

- HEAD 为 c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac，开发分支 readability/fc01-chinese-20261111。
- 最终包 dist/arm64/20261118_vision_step6_fix1/FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz 实际 196394133 字节，重新计算 SHA256=06e596b44b5aad5b8dab79a3eafe44147c0abd6f8d94495dcf25e2cebcebb830，与 DELIVERY.json/sha256 文件一致。
- install_fc01.sh 实际 75639 字节，SHA256=076a1431fe4a5dbbc8498e80e1ac3e44598df29272757ae0430e0f7f8de7c75f，与摘要一致。
- 实际遍历 tar 7749 个常规文件；native 中 11 份 .msg，含 NativeFlightSnapshot/TargetObservation/PlatformState/VelocityCommandV2。没有 __pycache__/pyc 文件。
- 发行 ARM64_RELEASE.json 绑定源码 c6fe7ed、原生/target候选5c4dc14ddd84371d60387e7ff5227c94b7b6ee47，非旧6a9包。
- 包内控制器 ELF magic 合法、e_machine=183(AArch64)，mission/px4_state_bridge/vision/只读target接口实际安装入口均存在。
- ACTUAL_BUILD_SOURCE_BINDING.json 记录的构建时 source_commit=6a9d0b3（历史构建身份）；111份实际编译输入与当前文件逐字SHA一致。原生/target未因后续发行修复发生变化。不要把该文件的 source_commit 改成c6fe冒称重新编译。
- git diff 候选5c4dc14..HEAD -- native/src native/config target/src standby.py prebuilt_boot.sh 为空。
- dist INSTALL_zh.md 与 docs/FC01_ARM64_INSTALL_zh.md 完全一致。它明确写出VM非真机、自动停旧、安装与就绪分离、预编译不用重编、未标定pending与真实site分开，以及回退实际备份路径。

## 最终报告仍需修正

1. REPORT_zh.md 开头仍称VM事务执行中；151/151 + 11/11旧计数应替换为最新最终单组180 tests，14.450秒，日志python-final-fix1.log。不要将这些重叠数量相加。
2. VERIFY_zh.md 最终构建命令仍使用 --version 20261118_vision_step6、旧output目录；最终sha检查cd也错指旧目录，应改fix1。build-bundle-full.log/DELIVERY.json是早期包证据；最终用build-bundle-fix1.log/FINAL_DELIVERY.json。
3. VM初装、repeat、回退/故障注入/track、类型支持等需以最后真实退出码和标记更新；草稿待补不代表失败，不得提前当通过。
4. REPORT中“两项独立审查P2已关闭”是此前问题表的历史描述，但末节另有真实尚未修复P2；最终应具体列名，避免给人全部P2已关闭的印象。
5. 初装VM保留1.23m目标是继承测试，不能写成最新包默认高度或现场目标；后续1.5m需明确独立VM fixture，现场继续继承。
6. 记录最终资料提交与包内源码commit分别是什么，报告提交不需要伪装成重新打包身份。工作区原有tools/和两份MicoVIO测试未跟踪，不能纳入或声称全库干净。

## 已知P2/现场限制

- standoff_m 可大于 horizontal_max_radius_m：配置工具接受，运行任务持续安全拒绝 STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE。不建议为已冻结包放宽保护。最终说明必须明确实填 standoff_m <= horizontal_max_radius_m <=2m。随包现场来源说明目前未明确写这个关系，只写上界；可在外部最终说明/报告补充，勿悄改包后沿用旧SHA。
- FC01_VISION_MODE_CONFIG_OK仅证明模式配置及原生check通过；不是任意相机site的C++初始化或标定真实性证明。实际辅助节点仍需真实启动验收。
- pending会将visual_navigation_enabled/px4_horizontal_enabled设true，但mission control_enabled=false；它是视觉诊断待机，不等同altitude。正常首测定高应选择altitude。
- 真正track必须填写可查的现场校准来源；串口MicoVIO若仅给里程计不具V4L2图像，不能作此目标视觉源。
- Zero3W性能、现场相机、PX4接收和真实飞行完全待测。ARM64 VM无反馈观察不能代替实机。

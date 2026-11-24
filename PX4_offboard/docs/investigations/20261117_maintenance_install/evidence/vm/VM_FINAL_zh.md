# ARM64 维护安装集成验证

使用隔离完整 QEMU-system-aarch64 / Cortex-A72 / Ubuntu22.04 ARM64 / Linux5.15 / 真实systemd249 / ROS2 Humble。没有连接用户OrangePi、PX4或RC。独立PTY提供本机串口设备，dummyAgent只是可执行进程及串口所有者，没有XRCE会话；六路飞机输入publisher为0。只读输出observer不发布任何消息。

包为用户现有stage3_fix1，SHA5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373，完全不变。最终安装器源码41e1532e73d36005a80cd134807a70b0c0263675，SHA96d6b6f9b0005a859d856152631821cf0d4677bc1826fc2356a6318f45e75a5c。中间版SHA归属见SCRIPT_VERSIONS_zh.md；文件名含final的早期日志不能据名称认定为最终摘要。

## 实际完成的事务

- active旧版 -> 软件安装成功：旧native428、服务Main407，观察旧native集合为空后才出现候选native2157/Main1590。日志actual-install-active.log，进程原始序列active-install-transition-proof.log。输入为0、setpoint/command/heartbeat输出为0，候选IDLE/WAITING，ready_for_manual_rc=false。
- 活动候选重复安装：原backup、Main1590/native2157不变，未重启，actual-repeat-active.log。
- active回退：恢复原navigation_boot入口，native3156，完整runtime原字节恢复，actual-rollback-active.log、rolled-back-active.json。
- 正常停止旧版后再装：没有旧native或MainPID，仍能完成安装，原target1.23/RCslot1保持；final-install-stopped.log / installed-from-stopped-old-script.json。
- 中间89d4版再次inactive安装、回退成功，日志final-install-stopped.log/final-rollback-stopped.log及intermediate-installed-stopped.json；已有unit生命周期与最终96d6一致，最终版新增首装unit.d父路径链接拒绝和类说明串整理。
- 最终96d6独立脚本重复安装与status：release-final-repeat.log、release-final-status.log。samebackup/Main5586/native6224，INSTALL_OK，缺DDS/RC/姿态/位置/电池/PX4。最终回退release-final-rollback.log恢复原inactive/MainPID0/无native。
- 最终96d6软件启动失败自动恢复：ExecStartPre仅在candidate cwd退出23，启动确实失败；安装器返回1并提示已恢复，原WorkingDirectory和inactive/MainPID0恢复，完整runtime SHA与最初基线一致。SOFTWARE_FAILURE_RESULT.json及release-final-failure-*日志。没有把预期失败写成安装成功。

ARM64运行库及消息typesupport检查随每次安装真实运行，ldd-sourced-runtime.log为正确ROS/包环境的动态依赖；未source环境的ldd结果同样保留，不混淆为真实服务依赖失败。没有新字节码污染不可变payload。

进程监视器从50ms因TCG负载调整到1秒，提供独立采样证据，不声称内核逐事件追踪。最强顺序保证来自安装器实际检查服务inactive/MainPID0/native0才调用start，与源码/注入测试/采样共同验证。所有source callback为0和输出为0的记录说明缺反馈不会获得动力发布权，不能据此推断真实飞行性能。

进度文件一处22:02为预计/误写时间，事务先后以实际日志、monotonic和文件时间为准。第一次后续状态取证引用了guest不存在的/tmp脚本而报错，随后以stdin传入相同只读采集源码成功；原错误日志保留。该取证错误不影响前面已完成的安装失败自动恢复。

最终96d6版全新正常安装实际完成：`release-final-full-install.log` 返回0，`FC01_PREBUILT_INSTALL_OK`，MainPID8467、唯一native9016，实际exe在20261117_stage3_fix1版本目录；`release-final-installed.json` 的六路source callback仍为0，setpoint/heartbeat/VehicleCommand输出仍为0，状态IDLE/WAITING，未就绪。最终完整进程序列及guest安装器摘要在release-final-process-transitions.jsonl。故障注入配置已移除，再次成功安装证明没有遗留阻断。

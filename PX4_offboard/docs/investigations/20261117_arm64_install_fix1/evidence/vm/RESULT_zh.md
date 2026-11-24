# 20261117 fix1 ARM64 实际安装回归

完整 QEMU-system-aarch64 6.2 Cortex-A72 2核/2GiB、Ubuntu22.04 ARM64 generic5.15、真实 systemd249 PID1、ROS2 Humble。使用实际生产安装器、ARM控制器、Supervisor、/proc/systemctl/fuser；PX4为始终未解锁/接地/静止的直接DDS合成源，Agent仅为持有PTY的软件替身。没有访问真实香橙派、UART、XRCE、PX4或电机。

最终实际包 SHA256 `5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373`，源码提交 `4c85a9e7e0c66193104164f03f4bfe28de709b9d`。

- 旧20261116失败目录直接执行原navigation.py --help，产生 `navigation_status.cpython-310.pyc`，SHA `00da579a6844553ecbdcf0bf5641c5b099a1a4150428a214b3eea9243f7fc3cb`；整个本次安装期间保持不变。
- 新独立版本真实安装成功（actual-install-retry.log）；唯一新控制器PID2370，systemd MainPID1755，Agent保持PID588。首次安装曾因0.25s快照新鲜度门拒绝、未停止旧版，保留actual-install.log；第二次有界重试成功，没有修改TTL。
- 不设置PYTHONDONTWRITEBYTECODE/PYTHONPYCACHEPREFIX，直接执行新版navigation.py status，退出0；新版本无任何__pycache__或pyc；随后严格verify_tree成功（direct-status-and-strict-tree.log）。状态文字本次显示STALE（历史快照约1.2s），不能当作当前飞机反馈有效的证明。缓存阻断修复与反馈新鲜度分别报告。
- 真实重复安装返回already_active=true，控制器仍PID2370、同backup、runtime完全不变（actual-repeat.log、repeated-state.json）。
- 目标高度1.23m、RC档位1和所有推力/权限/DDS参数保持，仅版本号及配置来源路径两个身份字段改变。旧服务配置/回退备份保留。
- 本次fix1手动回退三次有界尝试均在停服前被原始快照新鲜度门拒绝（actual-rollback*.log），没有覆盖旧文件、改参数或停止正在运行的新版；不能报告本次新版回退通过。20261116前一轮真实回退成功记录仅作历史证据，不能代替本次结果。
- 取证后在仅地面夹具中正常systemctl stop，节点已退出；再正常关闭隔离VM。这是验证结束清理，不是回退成功。

初次启动夹具时同时存在原Supervisor残留dummyAgent和测试服务dummyAgent，只在隔离VM、正式baseline之前正常清除了重复替身；安装器从未绕过唯一Agent检查。QEMU启动两次参数失误保留qemu.log，最终精确参数在boot-command.json。无生产代码修改。

详见RESULT.json、baseline-final.json、installed-state.json、repeated-state.json、final-state.json和guest-transaction-final.tar.gz。动态写入归档controller.log引发tar警告，保留guest-transaction-evidence.tar.gz，最终停止后稳定归档另存guest-transaction-final.tar.gz；不将有警告的第一份作为唯一证据。

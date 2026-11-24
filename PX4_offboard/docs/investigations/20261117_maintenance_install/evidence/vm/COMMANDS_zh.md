# 隔离 ARM64 VM 维护安装验证命令

环境为本地 QEMU system-aarch64 完整 Ubuntu22.04 ARM64/Humble/systemd249。所有 systemctl 命令只在 guest 执行。无真实香橙派访问。

准备：正常停止/禁用前轮合成反馈和测试Agent服务。保留PTY作为串口设备；原Supervisor按既有逻辑创建dummyAgent本地进程，无XRCE/PX4会话。启动旧productionservice，确认六类源callback为0。

```bash
systemctl stop fc01-navigation.service fc01-vm-ground.service fc01-vm-agent.service
systemctl disable fc01-vm-ground.service fc01-vm-agent.service
systemctl start fc01-navigation.service
```

只读观察器只有subscriptions，订阅原生输出并记录输入publisher计数；不发布遥测、控制命令或设定值。独立proc监视器每50ms记录nativePID集合变化，验证旧native完全退出后再启动新native。

安装器与包就绪后将实际执行：
```bash
runuser -u orangepi -- bash /home/orangepi/install_fc01_maintenance.sh /home/orangepi/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz
```

后续按每个实际事务结果记录backup路径并执行重复安装/回退。所有测试继承原目标高度1.23m与RC档位1，不能把默认值替换当作成功。

实际执行的独立安装器文件名为 `install_fc01_20261117_maintenance.sh`；逐次SHA记录在 `install-1-script.sha256`。进程监视器先用50ms，TCG负荷过高后改为0.2s，最终1s；它不是逐事件内核追踪，最强保证来自安装器停机验证（inactive/MainPID0/原生进程不存在后才启动）与独立进程采样共同提供。

已实际执行并成功：
```bash
runuser -u orangepi -- bash /home/orangepi/install_fc01_20261117_maintenance.sh /home/orangepi/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz
runuser -u orangepi -- bash /home/orangepi/install_fc01_20261117_maintenance.sh /home/orangepi/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz
runuser -u orangepi -- bash /home/orangepi/install_fc01_20261117_maintenance.sh --rollback /home/orangepi/FC01_navigation/prebuilt-backups/20261117T134041_006203Z-766
```

运行库使用实际服务同等前缀环境核对：
```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/FC01_versions/20261117_stage3_fix1/install/setup.bash
ldd /home/orangepi/FC01_versions/20261117_stage3_fix1/install/lib/jikl_fc01_system/fc01_system_node
```
`ldd-sourced-runtime.log` 为实际启动环境的解析结果；`environment-and-dependencies.log` 同时保留未source ROS环境时的原始shell结果，该原始结果中的 `not found` 不代表服务环境缺库。

最终维护脚本已替换后执行：
```bash
bash /home/orangepi/install_fc01_20261117_maintenance.sh --rollback /home/orangepi/FC01_navigation/prebuilt-backups/20261117T135535_488198Z-3302
```
该备份原本 inactive，结果仍 inactive、MainPID0、native0，运行YAML逐字节恢复。随后从此inactive基线再执行最终脚本安装、重复安装、状态与回退。

最终交付SHA96d6版实际执行repeat与status，通过sameSupervisor5586/native6224，不重启。安装器输出的完整绝对路径回退命令也实际调用：
```bash
bash /home/orangepi/install_fc01_20261117_maintenance.sh --rollback /home/orangepi/FC01_navigation/prebuilt-backups/20261117T140234_274294Z-5047
```

# FC01 维护安装：直接替换安装器，复用现有 ARM64 包

旧错误表示安装器把“监督器的飞机状态快照必须新鲜”当成了停旧前提。它不能证明包损坏；飞控断电或没有 DDS/RC 时，这个部署前提本来就无法满足。此次已从安装、重复安装、停止、验收和回退的成功条件中移除该前提。运行时飞行准入保护保留。

## 复制到香橙派

通过 U 盘把新 `install_fc01_20261117_maintenance.sh` 放到 `/home/orangepi/Desktop/`。原先已经校验通过的 `FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz` 继续使用，无需重新下载或重新编译。不再运行旧 `install_fc01_20261117.sh`。

现场按约定落地、拆桨、飞控断电执行。以普通 `orangepi` 用户运行，安装器自行请求 sudo：

```bash
bash "$HOME/Desktop/install_fc01_20261117_maintenance.sh" "$HOME/Desktop/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz"
```

安装器会校验包和本机软件，保留实际生效参数及服务备份，停止原 `fc01-navigation.service`，等待旧控制节点和旧监督器退出，再启动同一服务的新入口。无反馈时原控制器正常退出可能需要约 60 秒；安装器留出 120 秒并禁止强杀。新软件预检与验收另有 180 秒上限，期间不要重复启动安装命令。

成功标记：

```text
FC01_PREBUILT_INSTALL_OK entry=/home/orangepi/FC01_versions/20261117_stage3_fix1 service_pid=<PID> controller_pid=<PID>
```

随后单独输出 `FC01_FLIGHT_READINESS=...`。飞控断电时 `ready_for_manual_rc=false`，DDS、RC、姿态、位置、电池和 PX4 缺失属于正常待机，**不会把已经完成的软件安装改判为失败**。安装不会自动解锁、发送起飞请求或提升权限。

## 查看状态

```bash
bash "$HOME/Desktop/install_fc01_20261117_maintenance.sh" --status
python3 "$HOME/FC01_versions/20261117_stage3_fix1/navigation.py" status
```

安装器状态查看 systemd、真实控制器进程及独立就绪摘要。原导航状态入口继续显示具体等待原因。飞控上电后，现场按原流程确认真实反馈和就绪，再使用原有人工 RC 解锁、起飞、接管和急停操作。`INSTALL_OK` 仅表示软件版本已切换成功。

## 回退

安装完成会打印 `FC01_ROLLBACK_COMMAND=`，其右侧就是当前备份的可执行回退命令。也可读取同一个备份记录：

```bash
FC01_BACKUP=$(python3 -c 'import json,pathlib; print(json.loads((pathlib.Path.home()/"FC01_versions/20261117_stage3_fix1/deployment/INSTALL_STATE.json").read_text())["backup"])')
bash "$HOME/Desktop/install_fc01_20261117_maintenance.sh" --rollback "$FC01_BACKUP"
```

成功标记为 `FC01_PREBUILT_ROLLBACK_OK`。原先运行的服务恢复运行；原先已停止的服务恢复为停止。初次创建服务的回退恢复到原来无该单元的状态。回退本身也不要求飞机反馈。新版软件启动失败会尝试自动恢复；若旧进程无法退出或配置被外部修改，将明确报错并保留备份，不强杀、不覆盖外部编辑。

## 文件身份与验证边界

- 安装器源码提交：`41e1532e73d36005a80cd134807a70b0c0263675`。
- 安装器 SHA-256：`96d6b6f9b0005a859d856152631821cf0d4677bc1826fc2356a6318f45e75a5c`。
- 原包 SHA-256：`5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373`。
- 原包源码提交：`4c85a9e7e0c66193104164f03f4bfe28de709b9d`；原生候选提交：`adc55d0b7c67e80bf9d77b1082208c898f95b756`。本次未重编或更改该包。
- 开发机相关 89 项测试通过。ARM64 集成验证环境为 QEMU 完整 Ubuntu 22.04 / ROS 2 Humble / systemd；具体结果和脚本摘要见本目录证据。尚未在用户的真实 Orange Pi 或 PX4 上验证。

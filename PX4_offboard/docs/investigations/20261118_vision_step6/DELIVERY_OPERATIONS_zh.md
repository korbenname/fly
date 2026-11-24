# 最终fix1：香橙派维护切换操作

交付目录：`/home/jikl62/FC01_OrangePi_Navigation_20261028/dist/arm64/20261118_vision_step6_fix1/`。

复制目录内的安装包、安装器及两份.sha256到香橙派桌面。全部命令仅在落地、拆桨、飞控断电的维护状态执行；无需连接相机、RC或取得DDS数据。不要sudo整个安装器。

## 自动停止旧任务并安装启动

```bash
cd "$HOME/Desktop" &&
sha256sum -c FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz.sha256 &&
sha256sum -c install_fc01.sh.sha256 &&
bash ./install_fc01.sh ./FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz
```

安装器自动备份、按服务归属停旧、核对实际退出后启动新版。成功标记`FC01_PREBUILT_INSTALL_OK`，包含entry/service_pid/controller_pid。软件校验和启动可能用数分钟，每段显示进度，启动验收上限600秒。保留终端直到结果返回。无飞机数据时`FC01_FLIGHT_READINESS.ready_for_manual_rc=false`及缺项是预期待机，不是安装失败。若想单独先停止旧任务，可执行`sudo systemctl stop fc01-navigation.service`；随后的同一安装命令仍能完成切换。

## 查询

```bash
bash "$HOME/Desktop/install_fc01.sh" --status
python3 "$HOME/FC01_versions/20261118_vision_step6_fix1/navigation.py" status
journalctl -u fc01-navigation.service -n 80 --no-pager
```

## 选择定高测试

```bash
bash -c 'set -e; entry="$HOME/FC01_versions/20261118_vision_step6_fix1"; sudo systemctl stop fc01-navigation.service; python3 -B "$entry/navigation_vision_mode.py" altitude --entry "$entry" --maintenance; sudo systemctl start fc01-navigation.service; bash "$HOME/Desktop/install_fc01.sh" --status'
```

安装保留当前目标高度；必须核对显示1.5m。维护模式工具要求现用目标为1.5m、导航槽为1；条件不符时定高/视觉配置都拒绝，不会擅改现场参数。定高模式visual_navigation_enabled=false、px4_horizontal_enabled=false，不启动视觉任务。

## 缺现场标定：仅视觉诊断待机

```bash
bash -c 'set -e; entry="$HOME/FC01_versions/20261118_vision_step6_fix1"; sudo systemctl stop fc01-navigation.service; python3 -B "$entry/navigation_vision_mode.py" track --pending-calibration --entry "$entry" --maintenance; sudo systemctl start fc01-navigation.service; bash "$HOME/Desktop/install_fc01.sh" --status'
```

应见track/pending、mission_control_enabled=false、明确标定缺项。两个原生视觉开关启用，但任务控制关闭；这不是普通定高模式，也不代表可以跟踪。软件启动尚未完成时，立即查询可能只显示等待，稍后再查。

## 真实标定后选择首测track

```bash
entry="$HOME/FC01_versions/20261118_vision_step6_fix1"
cp -n "$entry/docs/FC01_VISION_SITE_TEMPLATE.json" "$HOME/fc01_site.json"
```

先按`$entry/docs/FC01_VISION_SITE_CONFIG_zh.md`填写实际设备、内外参、目标模型、采集延迟、安全间距及来源。不能把模板null/false批量改为confirmed，不能复制VM的synthetic数据。需要真实V4L2图像源；串口MicoVIO里程计字节不等于目标图像。

必须满足`standoff_m <= horizontal_max_radius_m <= 2.0`（米）。现有配置工具未前置拒绝间距大于半径，运行程序会安全拒绝STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE；此问题保留记录，未降低运行保护。

```bash
bash -c 'set -e; entry="$HOME/FC01_versions/20261118_vision_step6_fix1"; sudo systemctl stop fc01-navigation.service; python3 -B "$entry/navigation_vision_mode.py" track --entry "$entry" --site "$HOME/fc01_site.json" --maintenance; sudo systemctl start fc01-navigation.service; bash "$HOME/Desktop/install_fc01.sh" --status'
```

配置成功是`FC01_VISION_MODE_CONFIG_OK`；生产launch固定track。此时mission_control_enabled=true，仍必须等待真实相机、目标与PX4反馈。每次配置检查失败会恢复原配置并停止命令，服务保持停止，需先读错误再启动。

如需在切模式后等待完整软件验收，可再次执行上面的安装命令：重复安装保留当前模式和回退点，不另起控制器。

## 回退

使用本次成功安装输出的`FC01_ROLLBACK_COMMAND`原样执行。也可自动读取当前版本记录：

```bash
FC01_BACKUP=$(python3 -c 'import json,pathlib; p=pathlib.Path.home()/"FC01_versions/20261118_vision_step6_fix1/deployment/INSTALL_STATE.json"; print(json.loads(p.read_text())["backup"])') &&
bash "$HOME/Desktop/install_fc01.sh" --rollback "$FC01_BACKUP"
```

成功标记`FC01_PREBUILT_ROLLBACK_OK`。恢复原服务和runtime，原先停止则恢复停止，原先活动则重新启动验收。旧目录保留。

## 现场状态判据

当前反馈连续有效，Stabilized、未解锁、非导航档、最低油门、接地静止，完成本轮地面零点和零推力预发送后，才可显示新鲜READY_WAIT_ARM。人工RC解锁后8秒内选导航槽1，PX4实际确认Offboard才允许爬升；高度使用z0-z、PX4融合z/vz，1.5m连续稳定且navigation_allowed=true后才授权水平。track还需连续新图锁定、标定和自机反馈有效。接管、Kill和失效保护始终优先。

本轮安装验证环境是Ubuntu22.04.5 ARM64/Humble/systemd249的QEMU虚拟机；没有实际Zero3W、PX4或相机。实物帧率、通信质量、PX4实际接收和飞行性能待现场验证。阶段5简化模型存在DESCENT_TIMEOUT保护过程，不能把软件验证描述为已经实飞通过。

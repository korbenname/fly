# FC01 Orange Pi ARM64 离线安装

版本 `20261118_position_vision_step3`，适用 Orange Pi Zero 3W、Ubuntu 22.04 ARM64、ROS 2 Humble。包内含预编译控制器、ROS 消息和运行库、视觉任务及对应源码。发行身份见 `ARM64_RELEASE.json`。

## 1. 安装切换

落地、拆桨并断开飞控电源后，把本版的包、安装器和两份校验文件一起复制到香橙派桌面。执行：

```bash
cd "$HOME/Desktop" &&
sha256sum -c FC01_OrangePi_20261118_position_vision_step3_arm64.tar.gz.sha256 &&
sha256sum -c install_fc01.sh.sha256 &&
bash ./install_fc01.sh ./FC01_OrangePi_20261118_position_vision_step3_arm64.tar.gz
```

安装器继承当前运行配置，在旧进程退出后用同一服务启动新版。看到 `FC01_PREBUILT_INSTALL_OK` 即完成软件切换。保存本次输出的 `FC01_ROLLBACK_COMMAND`。飞控、USB 相机、遥控器或 DDS 离线时，运行状态可显示等待。安装会保留原有模式，选择下列模式需单独操作。

## 2. 选择基础定点或主相机导航

在相同维护状态下执行其中一项。两种模式都使用 PX4 的位置控制器和融合反馈。MTF‑02P 继续直连 PX4，USB 主相机可从地面观察。模式工具保留现有 1.5 m 目标与 RC 导航槽位 1，配置成功打印 `FC01_VISION_MODE_CONFIG_OK`。

**基础定点 `position`** 可拔掉 USB 主相机。真实离地后保持本轮地面起飞点 XY，目标相对高度 1.5 m。

```bash
entry="$HOME/FC01_versions/20261118_position_vision_step3"
sudo systemctl stop fc01-navigation.service &&
python3 -B "$entry/navigation_vision_mode.py" position --entry "$entry" --maintenance &&
sudo systemctl start fc01-navigation.service
```

**主相机导航 `position-track`** 同样先守起飞点。真实离地、1.5 m 高度和垂速连续稳定后才授予水平任务权限。默认在起飞点等待有效目标。跟踪需要新鲜连续观测。只有显式配置、原点对齐且通过范围检查的航点才可在无锁定时前移。

按包内 [现场配置说明](FC01_VISION_SITE_CONFIG_zh.md) 和 [模板](FC01_VISION_SITE_TEMPLATE.json) 准备 `$HOME/fc01_site.json`，使用该相机、目标和安装位置的实际标定与实测数据。

```bash
entry="$HOME/FC01_versions/20261118_position_vision_step3"
test -s "$HOME/fc01_site.json" &&
sudo systemctl stop fc01-navigation.service &&
python3 -B "$entry/navigation_vision_mode.py" position-track --site "$HOME/fc01_site.json" --entry "$entry" --maintenance &&
sudo systemctl start fc01-navigation.service
```

配置校验失败时，原配置保留，以上命令停止在报错处。查看错误后可手动重启原配置。

## 3. 实时状态和回退

```bash
bash "$HOME/Desktop/install_fc01.sh" --status
python3 -B "$HOME/FC01_versions/20261118_position_vision_step3/navigation.py" monitor
```

监视窗口显示新鲜度、起飞锚点、离地、到高授权和指令来源。Ctrl+C 退出窗口。拆桨地面核对 PX4 固件、位置有效性、RC 与 DDS 状态。人工 RC 流程沿用已有配置。以当前 `READY_WAIT_ARM` 和 PX4 的实际 Offboard 状态为准，历史状态标为 `STALE`。

回退也在维护状态执行。使用安装输出中 `FC01_ROLLBACK_COMMAND=` 后的完整命令。成功标记为 `FC01_PREBUILT_ROLLBACK_OK`。旧 fix1 版本保留在原目录。

板端相机帧率与时延、实际 PX4 目标采纳、自动爬升横漂和悬停误差需现场核验。

# ARM64 安装修正版 fix1

2026-11-17。对应源码4c85a9e7e0c66193104164f03f4bfe28de709b9d，仍使用阶段3同一ARM64控制器，不修改控制算法/参数/权限。

本次安装失败已定位：旧包里的 navigation.py 直接运行时会自动写入 __pycache__/navigation_status.cpython-310.pyc，后续精确发行清单校验拒绝这个额外文件。新入口在导入项目模块前禁写缓存，仍严格拒绝任何额外缓存；不是跳过完整性检查。修正版安装到独立的20261117_stage3_fix1目录，不删除或覆盖旧20261116失败目录、旧源码、服务、原参数和回退证据。

把这两个新文件从U盘复制到香橙派桌面：

- FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz
- 本次相配的 install_fc01.sh（替换桌面的旧同名安装器；不要继续用install_fc01_diagnose.sh）

以普通orangepi用户，拆桨、未解锁、接地静止；原服务和通信/PX4/RC保持有效安全地面反馈。解锁和Kill开关OFF、导航开关离开导航档。不要sudo整个安装器，写服务时它自行请求sudo。

先校验复制文件：

```bash
cd "$HOME/Desktop"
printf '%s\n' '5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373  FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz' | sha256sum -c -
printf '%s\n' 'c52f285e32241cf3a12a8daf67e60cc4ae8ff61a2afe91eb94648dd2626eecff  install_fc01.sh' | sha256sum -c -
```

两项均OK后安装：

```bash
bash "$HOME/Desktop/install_fc01.sh" "$HOME/Desktop/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz"
```

必须看到安装成功，再核对真实服务和进程路径；仅运行新目录中的status不能证明新控制器已启动：

```bash
systemctl --no-pager show fc01-navigation.service -p ActiveState -p MainPID -p WorkingDirectory -p ExecStart -p DropInPaths
pgrep -af '[f]c01_system_node'
bash "$HOME/Desktop/install_fc01.sh" --status
python3 "$HOME/FC01_versions/20261117_stage3_fix1/navigation.py" status
```

成功后WorkingDirectory、ExecStart及唯一fc01_system_node路径应指向/home/orangepi/FC01_versions/20261117_stage3_fix1。旧20261112服务保持到安全切换，生效目标高度继承原runtime；不会改为测试用1.23m或覆盖现场1.5m。重复执行同一安装命令会核对已运行版本而不重启/覆盖回退点。

手动回退：

```bash
FC01_BACKUP=$(/usr/bin/python3 -c 'import json; from pathlib import Path; print(json.loads((Path.home()/"FC01_versions/20261117_stage3_fix1/deployment/INSTALL_STATE.json").read_text())["backup"])') &&
bash "$HOME/Desktop/install_fc01.sh" --rollback "$FC01_BACKUP"
```

回退也需要有效安全地面条件。已完成回退后二次回退会拒绝覆盖，不是安装半成功。

若提示反馈STALE或“未取得新鲜且来源一致的监督器状态”，这与缓存错误分开处理。状态窗口会把快照存放时间加入各反馈年龄；显示0.9秒并不等于0.5秒有效窗口仍满足。不要跳过TTL或改保护参数。保留原服务，记录下面只读输出供核查：

```bash
bash "$HOME/Desktop/install_fc01.sh" --status
journalctl -u fc01-navigation.service -n 60 --no-pager
```

支持合同仍为既有人工RC flight-test、Humble、DDS domain0/network、/dev/ttyS2 115200 Agent。其他配置明确拒绝而不改成默认值。目标板安装由用户执行；本轮实际测试环境、完整回归和局限见FINAL_REPORT_zh.md。

本次fix1完整ARM64 VM验收：安装、无环境开关直接status后精确清单、重复安装同PID通过；手动回退三次被新鲜度保护拒绝，未完成本轮回退验收。未更改回退代码或任何保护。若现场也出现此拒绝，请保留当前运行版本并提供上述状态/日志，不要反复盲试、删除事务记录或强制替换service。

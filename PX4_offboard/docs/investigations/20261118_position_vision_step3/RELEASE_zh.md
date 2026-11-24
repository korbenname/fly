# FC01 定点与视觉第 3 步发行

分支 `readability/fc01-chinese-20261111`。归档源码为 `c7de1387c635cf1733e1d1e1a20a8c24d7ad7752`，原生及任务候选为 `b2ee5215e03a7c375714efb5f562f788473369a9`，包含第 1 步 `2a1348990976999a0549c8f00c85a2a6e6993b6b`。对照的上游 `korbenname/fly` 的 `target/` 为 `c4b32b92455a9c25fb1bce1262e8426f249f8521`。第 3 步修改离线说明、集成夹具和证据，控制器、任务、安装器和服务的生产源码沿用前两步。x86 和 ARM64 均从 `53a37dd` 的这些生产文件构建，经逐文件比较与归档提交一致。[第 1 步](../20261118_position_takeoff_step1/HANDOFF_zh.md) · [第 2 步](../20261118_position_vision_step2/HANDOFF_zh.md) · [集成复验](ROS_zh.md)

## 交付身份

目录 `/home/jikl62/FC01_OrangePi_Navigation_20261028/dist/arm64/20261118_position_vision_step3`。归档包含 7,751 个文件，196,421,358 字节。[交付清单](evidence/DELIVERY.json)

| 文件 | SHA-256 |
|---|---|
| `FC01_OrangePi_20261118_position_vision_step3_arm64.tar.gz` | `fdc2d179cfdd4dabdbd73b8d6a5ff89da3855938f1717c7726e0b41f82823e8c` |
| `install_fc01.sh` | `7f5a3ebc8f7455b6adabee65d614fba4f9aa3b4aece87ef5e25db0025f2950d9` |

两份同名 `.sha256` 与 [安装说明](../../FC01_ARM64_INSTALL_zh.md) 随包交付。安装器使用独立版本目录，继承当前模式和配置。在香橙派按说明可选 `position` 或 `position-track`，打开实时监视窗口，或执行安装回执中的回退命令。已安装的 `20261118_vision_step6_fix1` 是回退基线。

## 构建和归档复验

Ubuntu 22.04 ARM64 QEMU 用户态、ROS 2 Humble 和 GCC 11 的四包构建用时 14 分 40 秒。原生 ELF 为 AArch64，SHA-256 为 `2664fd6af966c16e1f5e351d4d6b8374be1bb0ad07be61f3157dc2a4c870ce8a`。首次扫描发现安装树少了六个 SDK 环境钩子，恢复同一已核验 SDK 的钩子后重跑扫描。最终动态依赖扫描覆盖 89 个构建输入 ELF。原生及工具源码 60 项、视觉任务源码 52 项和安装树 7,583 项逐文件绑定至包内构建回执。开发机发行相关测试 112 项通过。[ARM64 构建](evidence/arm64-build.log) · [依赖扫描](evidence/arm64-runtime-scan.log) · [依赖收集](evidence/arm64-runtime-collect.log) · [宿主测试](evidence/host-release-summary.log)

在仓库根目录核对同一份生成文件：

```bash
(
set -e
cd dist/arm64/20261118_position_vision_step3
sha256sum -c FC01_OrangePi_20261118_position_vision_step3_arm64.tar.gz.sha256
sha256sum -c install_fc01.sh.sha256
bash -n install_fc01.sh
)
```

另从空目录执行 `navigation_prebuilt_install.extract_checked`，逐文件核对包内清单和 ELF，按 `build_arm64_bundle.validate_evidence` 复核安装树和真实编译输入，并用 `git show` 核对选择的发行源码。[独立解包结果](evidence/arm64-independent-extract.json)

## 同一份包的 ARM64 systemd 验收

Ubuntu 22.04.5 ARM64、ROS 2 Humble、systemd 249 的 QEMU 虚拟机以运行中的 fix1 开始。UART 由本地空 PTY 和替身 Agent 模拟。以 `orangepi` 账户执行交付安装命令，并核对两份摘要。旧原生 PID 70547 完全退出后启动新版 PID 76539。新版保留 1.5 m、原模式、RC 与 DDS 参数，打印 `FC01_PREBUILT_INSTALL_OK` 并待机。ARM64 上 11 个生成消息完成 CDR 往返。

通过交付中的 `navigation_vision_mode.py` 配置 `position` 和 `position-track`，后者使用 VM 专用合成标定。两个模式各只有一个原生控制器。分别监听 20 秒，收到的轨迹、姿态目标、控制命令与心跳均为 0。退出 `monitor` 后服务 PID 不变。重复安装返回 `already_active: true`，保留同一进程和原备份。回退等待新版所有服务所属进程退出，恢复 fix1 并打印 `FC01_PREBUILT_ROLLBACK_OK`，两份服务文件和旧 runtime SHA 逐项一致。0.25 秒一采样的进程见证在每次切换间观察到空档且未观察到控制器重叠。

验收脚本首次在回退成功后误用带尾空格的模式匹配成功标记。已用独立的只读复验补齐回退后的快照和逐字节比较。证据包含实际执行的脚本、补查脚本、日志与进程见证。纯软件检查在这一 QEMU 虚拟机中可能耗时数分钟。[验收摘要](evidence/ARM64_SYSTEMD_ACCEPTANCE.json) · [完整复验记录](evidence/ARM64_SYSTEMD_EVIDENCE.tar.gz)

## 现场待验证

实际 Zero 3W 的 CPU、内存及载荷下心跳、USB 主相机帧率/采集时延与标定、UART 和 DDS、当前刷入固件的位置目标采纳、MTF‑02P 的现场融合质量、飞行横漂和悬停性能仍待测。x86 模型使用 PX4 位置控制源码、三轴点质量及模拟飞控反馈，尚未证明达到用户手动 Position 的实飞表现。ARM64 虚拟机提供软件部署证据。

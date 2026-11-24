# 2026-11-17 FC01 维护安装修复记录

## 源码与范围

本轮基线 `80b32f1ed7737ecbccc021085f5a45647bb376c1`，分支 `readability/fc01-chinese-20261111`。
安装代码与测试提交 `41e1532e73d36005a80cd134807a70b0c0263675`。发行身份详见 `DELIVERY.json`。

本次仅修改安装事务和发行入口，不更改飞控源码、控制权限、目标高度、推力参数、RC、Kill、PX4 失联保护或视觉开关。`native/src`、`native/config`、`native/tools`、`launcher.py`、`standby.py`、`navigation_service.py`、`navigation_prebuilt_service.py` 相对本轮基线均无差异。未操作用户实机、推送远程或修改 main。用户原有未跟踪的 MicoVIO 测试及 tools 文件保留，不纳入本轮提交。

## 原因与实际修改

旧安装器要求地面反馈、实时 IPC 来源和 250 ms 快照等满足飞行准入的条件，导致现场断电维护无法停旧。部署软件是否完整、服务是否正确启动，与飞机是否就绪被混为一谈。

| 文件 | 修改目的 |
|---|---|
| navigation_prebuilt_install.py | 所有部署路径只验本机软件/进程；移除 require_ground/live_state/Agent会话依赖；支持旧服务已停和无单元但已有明确runtime的初装；完整参数备份继承；先退旧再启新；软件故障恢复；独立显示飞行就绪 |
| build_arm64_bundle.py | 从已提交源码生成固定包 SHA 的可执行独立安装器，支持 --reuse-archive，不重打原 ARM64 包 |
| tests/test_navigation_prebuilt_install.py | 54 项维护安装、进程顺序、重复安装、恢复/回退、并发编辑、原参数保护、包安全与显示回归 |
| tests/test_arm64_bundle.py | 独立脚本生成、无工程目录运行、SHA固定、帮助/语法和摘要验证 |

安装仍校验压缩包摘要、完整文件集合和每文件 SHA、路径/容量/链接安全、AArch64 ELF、Ubuntu22.04/Humble、动态库、自定义消息类型支持、原生参数检查器、systemd 单元和实际服务/进程路径。Agent 可执行文件和本机串口资源仍属本机依赖；Agent 无会话、飞控断电、无 RC/DDS 不再是安装失败原因。

停止必须观察 MainPID=0、服务 inactive/failed 且全机旧控制器为空。原生控制器无反馈可能等待 60 秒退出，安装器预算调整到 120 秒；没有 SIGKILL。软件启动验收允许原有完整预检，在 180 秒内核对唯一控制器真实路径、ELF摘要、归属、cwd、完整参数和 DDS 设置，再观察一秒进程稳定性；不消费飞机状态。

独立复核期间发现并修复：可选状态 JSON 异常/查询失败反向影响结果、状态显示对旧源反馈续鲜、sudo 等待时覆盖外部参数修改、回退专用 drop-in 编辑竞态、初装 drop-in 父路径符号链接、错误回退脚本名和停旧超时误称恢复。针对回归均保留测试。历史失败证据没有被删除。

## 可复核验证

开发机 x86 Python 测试命令：

```bash
PYTHONPATH=tests:. python3 -m unittest test_navigation_prebuilt_install test_navigation_prebuilt_service test_arm64_bundle test_navigation_service -v
```

实际 89 项通过，日志 `evidence/host-89-tests.log`。其中安装器 54 项、预编译服务 6 项、打包 3 项、原服务 26 项。独立审查安装器与打包器 57 项通过，日志 `evidence/independent-57-tests.log`。各组合存在重叠，不累加成测试总数。

安装替身的 live_state/Agent会话方法直接抛 AssertionError，任何部署路径误调就失败；每次 start 断言旧控制器及旧 MainPID 都不存在。状态缺失、旧状态、无 Agent 进程/会话均覆盖。首次创建单元仍要求已有明确 runtime，不凭空生成飞行权限或参数。

严格重读原包通过：4974 份清单及文件；原 tar 6498104 字节，SHA 未变。最终独立 shell 语法、内嵌 Python 语法、帮助入口、固定唯一 SHA 和 SHA 侧文件验证通过。原包清单里的旧安装器是不可变历史内容；现场使用外部新安装器，不能误用包内旧版安装入口。

ARM64 集成结果见 `evidence/vm/` 与该目录的最终报告；只在隔离 QEMU 完整 Ubuntu22.04 ARM64/Humble/systemd249 内运行，不是用户香橙派实测。PTY/dummyAgent仅模拟本机串口与进程，无 PX4/XRCE 会话，无合成遥测 publisher；只读观察器检查控制输出。测试原目标设为1.23m、RCslot1，用于证明继承实际配置而非覆盖默认1.5m。

## 交付与边界

实际交付 `dist/arm64_maintenance/install_fc01_20261117_maintenance.sh`，固定复用用户已有 `dist/arm64_fix1/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz`。一条命令、状态、回退方式见 `INSTALL_zh.md`。

三项验收分别是：无飞机输入仍可安装；旧进程消失后才出现新进程；零反馈运行时仍不能获得飞行动作权限。证据支持软件层验收，不构成真实飞行性能或实机安全证明。真实 UART、电气连接、PX4 DDS、RC配置、定位/电池、起飞落地表现仍由现场按原流程确认。本轮没有执行解锁或动力程序。

最终交付安装器 SHA96d6 已实际执行重复安装、状态、回退和软件启动故障注入恢复。故障通过 VM 专用 ExecStartPre 仅在候选目录退出23实现，安装器正确返回失败1并恢复原 inactive / MainPID0 / old入口，完整 runtime SHA 与基线相同（`evidence/vm/SOFTWARE_FAILURE_RESULT.json`）。第一次取回状态用了不存在的guest临时脚本路径，仅取证命令报错，原记录保留；随后通过标准输入传入同一只读采集源码成功取证，恢复断言均通过。

最终交付96d6版又完成一次实际全新安装，返回0 / INSTALL_OK，服务8467、唯一控制器9016，六路输入和三类输出均为0；证据release-final-full-install.log / release-final-installed.json。重复、回退、故障恢复和最终正常安装均实际执行最终交付文件。真实OrangePi未连接。

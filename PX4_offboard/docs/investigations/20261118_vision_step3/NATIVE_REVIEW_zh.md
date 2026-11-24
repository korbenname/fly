# 第 3 步原生到高授权审查与回归

审查基线：`827f1f4535bc55b1bfa3fc2e97439367af226831`。本节只描述离线原生控制证据，不代表实机飞行性能或 PX4 已接收发布。

## 已确认的缺陷

旧 `Core::step()` 的到高 `dwell()` 只累计主机控制周期时间。在位置消息仍处于 `local_timeout` 内时，控制循环重复读取同一个 `timestamp_sample`，也能把单个符合高度/速度容差的缓存样本累计为连续稳定。独立复现用本轮基线头文件运行退出 1，输出 `REPRODUCED: one unchanged PX4 local sample manufactured target settling`；同一复现改用当前头文件运行退出 0。见 `evidence/native/cached-height-reproduction.json` 和配套原始日志。

此外，旧 `HeightHold` 仅凭阶段和历史离地事实就保持 `navigation_allowed()`，实测高度后来越出容差也没有撤销水平导航权限。本轮新增的回归明确要求：撤销水平权限，但保持原生垂向闭环纠偏，不能因普通高度偏差切断推力。

## 修改位置

- `native/src/jikl_fc01_system/include/jikl_fc01_system/takeoff_evidence.hpp`：新增 `TargetHeightSettling`，至少三条严格新原始位置/速度测量，同时满足 PX4 源时间与本机单调时间稳定窗；重复、同戳换值、回退和断续输入不能造新证据。已确认缓存过期时同时撤销内部 `confirmed`，不仅改变函数返回值。
- `native/src/jikl_fc01_system/include/jikl_fc01_system/attitude_altitude_core.hpp`：用上述真实多样本稳定证据进入 `HeightHold`；已经定高后若越界，重新累计稳定窗。`navigation_current()` 在当前快照上复核健康、解锁/Offboard、未着地、Kill/接管、估计器重置、实测高度/垂速与证据时效。现有视觉最终发布查询改用该当前判定。
- 同一文件的 `local_requires_control_step()`：到高后，中间越界的 local 回调也必须立刻交给控制步骤清窗，不能让随后正常尾帧覆盖。新显示阶段 `TARGET_HEIGHT_SETTLING` 表示已离地但目标高度重新稳定中，不再错误显示“离地未确认”。
- `native/tests/test_stage3_native_navigation_gate.cpp`：真实 `Runtime → Core → publication boundary` 合成回归，以及源时间/主机时间证据专项反例。

没有修改垂向 PID、推力包络、推力斜率、人工 RC 解锁/导航配对、Kill、接管或既有故障保护。本轮没有修改生产飞行配置。有效配置快照 `tests/fixtures/stage3_runtime_20261114.yaml` 保留目标 1.5 m、导航槽位 1、人工解锁到导航期限 8 s、视觉关闭。该快照只是已有现场证据，不能替代再次核对设备当前 `runtime.yaml`。

## 实际验证

环境：当前 x86_64 开发机；GCC 主机纯 C++，无 ROS/DDS/飞控/动力连接。

1. Debug 严格告警构建及 CTest：86/86 通过。日志 `evidence/native/configure.log`、`build-final.log`、`ctest-final.log`。
2. 新 gate 回归覆盖：一条缓存高度不能到高；离开容差撤销水平权限而垂向发布继续；返回一个样本不能立即恢复权限；短暂高度/垂速越界不能被回调尾帧覆盖；当前反馈失效、Offboard 撤销、z/vz/姿态重置、高度或垂速越界均不授权；armed 后的 z/vz 重置不能重新定义 z0；迟到短源序列和同戳换值不能凑稳定时间。
3. 现有 `test_rc_navigation`、`test_rc_takeoff_stage`、`test_stage3_state_invariants`、`test_takeoff_evidence` 等均包含在 86 项中，继续覆盖严格地面参考、真实零预流提交、RC 解锁到导航期限、错误槽位/模式确认延迟、离地/1.5 m 到高/降落/再次准备和安全锁存。
4. 新 gate 使用 ASan+UBSan 运行通过，无报告；见 `sanitizers-build.log`、`sanitizers.log`。这不是对所有代码的 sanitizer 覆盖声明。
5. 基线与当前的三套既有确定性轨迹共 26,371 行逐字节相同：控制轨迹 14,626 行、事务/IPC 966 行、最终发布 10,779 行。浮点输出采用原有 hexfloat 精确比较。见 `evidence/native/differential/summary.json`；完整日志以 `.log.gz` 保留。该结果证明这些已覆盖输入上的无视觉推力/状态/发布顺序不变，不宣称全状态空间等价。本轮故意修复的异常重复样本与越界路径由独立反例验证。

## 可复现命令

在工程根目录：

```bash
cmake -S native/tests -B /tmp/fc01-vision-step3-native-build -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01-vision-step3-native-build -j 2
ctest --test-dir /tmp/fc01-vision-step3-native-build --output-on-failure

g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I native/src/jikl_fc01_system/include -I native/tests \
  docs/investigations/20261118_vision_step3/evidence/native/reproduce_cached_height.cpp \
  -o /tmp/fc01-cached-height-repro
/tmp/fc01-cached-height-repro

g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -fno-omit-frame-pointer -g \
  -I native/src/jikl_fc01_system/include \
  native/tests/test_stage3_native_navigation_gate.cpp -o /tmp/fc01-native-gate-san
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 /tmp/fc01-native-gate-san
```

## 仍待现场验证

本回归只喂合成的 PX4 融合 z/vz，不模拟机体动力学。尚无本轮实机 ROS/DDS 延迟、真实离地、到高超调、稳定时间、定高误差、水平漂移或最长实测心跳间隔结论。后续现场先按拆桨、未解锁范围核对当前运行版本、有效配置、RC 档位/开关以及反馈时效。带桨飞行和性能结论仍由现场人员执行并保留同次试飞日志。

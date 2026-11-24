# 第 3 步可复验命令

以下在开发机 Ubuntu 22.04.5 x86_64 / ROS 2 Humble 实际验证。仅用于离线与隔离 DDS 测试；不是设备安装或带桨飞行命令。必须使用空闲、与实机不通的 ROS domain。`ros_stage3_lifecycle.py` 会启动真实原生节点，但仅在 localhost 隔离域内消费合成 PX4 数据。

## 本机单元和构建

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  tests.test_mission_native_gate tests.test_mission_cached_demand \
  tests.test_mission_sensor_contract tests.test_px4_platform_feedback \
  tests.test_overnight_stage5_ros_contract tests.test_release_payload_policy \
  tests.test_navigation_feedback_freshness tests.test_navigation_diagnostics \
  tests.test_navigation_service tests.test_stage3_status_readiness -v
PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
  tests.test_navigation_prebuilt_install tests.test_navigation_prebuilt_service \
  tests.test_native_profile tests.test_vision_step3_node_sinks -v
cmake -S native/tests -B /tmp/fc01-vision-step3-native-build -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01-vision-step3-native-build -j 2
ctest --test-dir /tmp/fc01-vision-step3-native-build --output-on-failure
FC01_PX4_PREFIX=/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs \
FC01_TARGET_BUILD_DIR=/home/jikl62/.cache/fc01-vision-step3-20261118 bash target/build.sh
```

原生新 gate 的 ASan/UBSan 与漏洞复现命令见 `NATIVE_REVIEW_zh.md`。新节点差分证据默认写 `/tmp/fc01-vision-step3-node-sinks`。

## 真实 ROS 通信与当前消息生成

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 bash <<'CHECK'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step3-20261118/install/local_setup.bash
FC01_STEP3_INSTALL=/home/jikl62/.cache/fc01-vision-step3-20261118/install
FC01_STEP3_OUTPUT=$(mktemp -d /tmp/fc01-step3-verify.XXXXXX)
/usr/bin/python3 tests/ros_generated_interface_check.py --prefix "$FC01_STEP3_INSTALL/jikl_fc01_system" --report "$FC01_STEP3_OUTPUT/generated.json"
/usr/bin/python3 tests/ros_native_mission_smoke.py --output "$FC01_STEP3_OUTPUT/mission.json"
/usr/bin/python3 tests/ros_target_interface_smoke.py --prefix "$FC01_STEP3_INSTALL" --output "$FC01_STEP3_OUTPUT/interfaces"
/usr/bin/python3 tests/ros_mission_sensor_smoke.py --prefix "$FC01_STEP3_INSTALL" --output "$FC01_STEP3_OUTPUT/sensors"
ctest --test-dir /home/jikl62/.cache/fc01-vision-step3-20261118/build/fc01_vision --output-on-failure
printf '证据：%s\n' "$FC01_STEP3_OUTPUT"
CHECK
```

成功标记：9 种消息、`FC01_NATIVE_MISSION_GATE_PASS 12 named checks`、`FC01_STEP1_ROS_INTERFACE_PASS`、`FC01_STEP2_MISSION_SENSOR_PASS 13 named checks`、CTest 2/2。任务 fixture 临时启用意图只供本域订阅者读取，没有 `/fmu/in` 动力发布权。

## 原生全流程合成 DDS

本轮构建身份文件是 `evidence/fc01-step3-ROS_BUILD.json`，核对 50 个原生输入和二进制摘要（准确数量以文件为准）。它只适用于本次 ELF。重新编译后须生成对应的新构建身份，不能删去检查或仅替换源码 HEAD。

在前述相同的干净 ROS 环境中，执行：

```bash
FC01_STEP3_NODE=/home/jikl62/.cache/fc01-vision-step3-20261118/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node
FC01_STEP3_PROOF=docs/investigations/20261118_vision_step3/evidence/fc01-step3-ROS_BUILD.json
for FC01_STEP3_SCENARIO in normal range_unavailable feedback_stale quat_reset vertical_reset offboard_loss offboard_flag rc_takeover kill velocity_warning; do
  FC01_STEP3_CAPTURE=$(mktemp -d /tmp/fc01-step3-lifecycle.XXXXXX)
  /usr/bin/python3 tests/ros_stage3_lifecycle.py \
    --node "$FC01_STEP3_NODE" --build-evidence "$FC01_STEP3_PROOF" \
    --domain 195 --scenario "$FC01_STEP3_SCENARIO" \
    --output "$FC01_STEP3_CAPTURE/result"
done
```

每个 `result.json` 应为 `PASS`、`error:null`，所有 gate true，包括 `inputs_unchanged_during_run`。简化 plant 的性能不能当作实机结果。首次 range 场景漏采终态已修夹具，原失败保留 `evidence/first-failures/`。

## 仍会失败的历史严格比较

```bash
PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_stage3_control_equivalence -v
```

当前真实结果 5 PASS / 2 FAIL：历史 f067 的完整 JSON 字节与新增观察字段不一致。该命令没有被跳过或改成宽容比较。本次控制出口差分使用前述 `test_vision_step3_node_sinks`，明确保留所有 target/heartbeat/command 字段与顺序，独立比较本次基线的控制状态字段，完整日志留档。不要把历史测试写成全绿。

## 现场后续只读范围

本轮没有板端执行。后续拆桨、未解锁先核对维护安装器既有 `--status`、实际服务/进程路径和生效 `runtime.yaml`；用现有 `navigation.py status` 观察当前反馈，读取 `/fc01/native/flight_state`、`/fc01/platform/status`、`/fc01/mission/sensors/status`。

原有人工流程仍由原生控制器决定：地面 READY、操作者按既有 RC 映射解锁、8 s 内导航槽位 1、PX4 实际确认 Offboard、原生爬升与定高。这里不改变硬件操作条件，也不把单个状态字符串当作现场起飞许可。本候选尚未安装到设备，安装交付由后续队列完成。

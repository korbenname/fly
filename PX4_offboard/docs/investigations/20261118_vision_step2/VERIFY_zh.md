# 第 2 步开发机复验命令

以下已在 Ubuntu 22.04.5 x86_64、ROS 2 Humble 实际执行。只使用合成 DDS/图像；不启动原生飞控节点。不是香橙派部署或飞行命令。

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
/usr/bin/python3 -m unittest tests.test_px4_platform_feedback tests.test_mission_sensor_contract tests.test_mission_cached_demand tests.test_overnight_stage5_ros_contract tests.test_release_payload_policy -v
/usr/bin/python3 tests/run_native_cpp.py
FC01_PX4_PREFIX=/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs \
FC01_TARGET_BUILD_DIR=/home/jikl62/.cache/fc01-vision-step2-20261118 bash target/build.sh
```

在新子 shell 中选用空闲隔离域（开发机实际使用过 175–181），不要接入生产 ROS 域：

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH ROS_DOMAIN_ID=180 ROS_LOCALHOST_ONLY=1 bash <<'CHECK'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step2-20261118/install/local_setup.bash
STEP2_INSTALL=/home/jikl62/.cache/fc01-vision-step2-20261118/install
STEP2_OUTPUT=$(mktemp -d /tmp/fc01-step2-verify.XXXXXX)
/usr/bin/python3 tests/ros_mission_sensor_smoke.py --prefix "$STEP2_INSTALL" --output "$STEP2_OUTPUT/mission"
/usr/bin/python3 tests/ros_camera_calibration_smoke.py --binary "$STEP2_INSTALL/fc01_vision/lib/fc01_vision/vision_image_node" --output "$STEP2_OUTPUT/calibration"
/usr/bin/python3 tests/ros_px4_platform_bridge_smoke.py --report "$STEP2_OUTPUT/bridge.json"
/usr/bin/python3 tests/ros_target_interface_smoke.py --prefix "$STEP2_INSTALL" --output "$STEP2_OUTPUT/interfaces"
/usr/bin/python3 tests/ros_generated_interface_check.py --prefix "$STEP2_INSTALL/jikl_fc01_system" --report "$STEP2_OUTPUT/generated.json"
/usr/bin/python3 tests/ros_velocity_command_smoke.py --report "$STEP2_OUTPUT/v1.json"
/usr/bin/python3 tests/ros_vision_smoke.py --report "$STEP2_OUTPUT/self-odometry.json"
ctest --test-dir /home/jikl62/.cache/fc01-vision-step2-20261118/build/fc01_vision --output-on-failure
printf '证据目录：%s\n' "$STEP2_OUTPUT"
CHECK
```

可观察成功标记：`FC01_STEP2_MISSION_SENSOR_PASS 13 named checks`、标定 JSON `passed:true`、`FC01_PX4_PLATFORM_BRIDGE_PASS`、`FC01_STEP1_ROS_INTERFACE_PASS`、消息 `messages:8`、两个旧入口 PASS、CTest 2/2。

`evidence/` 的 before/失败日志用于历史复现；`--expect-unfixed` 仅给保存的旧节点复现，不是当前验收选项。独立方法复现脚本引用原工作区/临时路径，报告保留当时执行身份；当前正式回归使用上面的仓库测试入口。

现场后续只读观察入口（本轮未在香橙派验证）：`/fc01/platform/status`、`/fc01/vision/camera_status`、`/fc01/mission/sensors/status`。传感器 ready 不代表原生 RC/起飞准入，也不代表视觉控制已启用。

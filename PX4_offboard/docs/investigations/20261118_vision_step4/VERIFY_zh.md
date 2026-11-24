# 第4步可复验命令

以下已在 Ubuntu 22.04 x86_64 / ROS 2 Humble 运行。不是板端安装或飞行命令。ROS测试仅在localhost独立域消费合成数据；绝不能连接实机DDS域运行这些夹具。

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
PYTHONPATH=.:tests /usr/bin/python3 -m unittest discover -s tests -p 'test_mission_*.py' -v
PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_vision_step4_node_inputs tests.test_vision_step4_node_sinks tests.test_overnight_stage5_ros_contract tests.test_release_payload_policy -v
PYTHONPATH=.:tests /usr/bin/python3 -m unittest tests.test_mission_sensor_contract tests.test_px4_platform_feedback tests.test_navigation_feedback_freshness tests.test_stage3_status_readiness tests.test_overnight_stage5_ros_contract -v
cmake -S native/tests -B /tmp/fc01-vision-step4-native-build -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01-vision-step4-native-build -j 2
ctest --test-dir /tmp/fc01-vision-step4-native-build --output-on-failure
FC01_PX4_PREFIX=/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs FC01_TARGET_BUILD_DIR=/home/jikl62/.cache/fc01-vision-step4-20261118 bash target/build.sh
```

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 bash <<'CHECK'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash
/usr/bin/python3 tests/ros_generated_interface_check.py --prefix /home/jikl62/.cache/fc01-vision-step4-20261118/install/jikl_fc01_system --report /tmp/fc01-step4-generated-repeat.json
/usr/bin/python3 tests/ros_native_mission_smoke.py --output /tmp/fc01-step4-native-mission-repeat.json
/usr/bin/python3 tests/ros_mission_horizontal_smoke.py --output /tmp/fc01-step4-horizontal-repeat.json
CHECK
```

真实原生收包/输出全流程：在上述干净ROS环境中运行。`ROS_BUILD.json`严格绑定本轮ELF和生产源码；重新构建后应生成新的真实构建证据，不能拿历史ELF重贴HEAD。

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH bash <<'MATRIX'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash
FC01_STEP4_NODE=/home/jikl62/.cache/fc01-vision-step4-20261118/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node
for FC01_STEP4_SCENARIO in normal track_lost track_timeout track_source track_reset track_origin horizontal_feedback_invalid horizontal_reset rc_takeover kill offboard_loss feedback_stale; do
  FC01_STEP4_OUTPUT=$(mktemp -d /tmp/fc01-step4-replay.XXXXXX)
  /usr/bin/python3 tests/ros_stage3_lifecycle.py --node "$FC01_STEP4_NODE" --build-evidence docs/investigations/20261118_vision_step4/evidence/ROS_BUILD.json --horizontal --scenario "$FC01_STEP4_SCENARIO" --domain 172 --output "$FC01_STEP4_OUTPUT/result"
done
MATRIX
```

新外环 ASan/UBSan 检查：

```bash
for FC01_STEP4_TEST in horizontal_navigation horizontal_runtime; do
  g++ -std=c++17 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie -no-pie -g -I native/src/jikl_fc01_system/include "native/tests/test_${FC01_STEP4_TEST}.cpp" -o "/tmp/fc01-step4-${FC01_STEP4_TEST}-san"
  ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 "/tmp/fc01-step4-${FC01_STEP4_TEST}-san"
done
```

无视觉精确差分（脚本会只在指定的新临时目录编译内存夹具）：

```bash
python3 docs/investigations/20261118_vision_step4/evidence/reproduce/no_vision_differential.py --output /tmp/fc01-step4-no-vision-repeat
# 实际节点的所有控制发布字段及顺序差分已包含在上方 test_vision_step4_node_sinks 中。
```

成功证据及限制见 `REPORT_zh.md` 和 `evidence/VALIDATION.json`。无视觉控制输出差分与所有失败原始证据保留；不把固定XY模型或合成图像当成真实跟踪性能。

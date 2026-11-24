# 第5步可直接复验命令

以下命令用于开发机 Ubuntu 22.04 x86_64 / ROS 2 Humble。所有相机、RC、PX4反馈均由合成夹具生成，最终 `/fmu/in` 只有本机typed接收端消费。不要把这些测试加入实机DDS域。它们不是安装、解锁或实机飞行命令。

## 1. 原生与Python回归

```bash
cd /home/jikl62/FC01_OrangePi_Navigation_20261028
PYTHONPATH=.:tests /usr/bin/python3 -m unittest \
 tests.test_mission_cached_demand tests.test_mission_horizontal \
 tests.test_mission_native_gate tests.test_mission_sensor_contract \
 tests.test_px4_platform_feedback tests.test_navigation_feedback_freshness \
 tests.test_stage3_status_readiness tests.test_vision_step4_node_inputs \
 tests.test_vision_step4_node_sinks tests.test_vision_step5_safety_edges \
 tests.test_vision_step3_node_sinks tests.test_overnight_stage5_ros_contract \
 tests.test_release_payload_policy tests.test_vision_replay \
 tests.test_vision_adapter tests.test_vision_state_validation tests.test_vision_settings \
 tests.test_stage3_vision_lifecycle tests.test_stage3_vision_resource_budget \
 tests.test_stage3_adapter_budget tests.test_mission_pending_alignment -v
PYTHONPATH=native/tests /usr/bin/python3 -m unittest native.tests.test_rc_recovery_adapter -v
/usr/bin/python3 native/tests/adapter_compile_check.py
cmake -S native/tests -B /tmp/fc01-step5-repeat-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/fc01-step5-repeat-native -j 4
ctest --test-dir /tmp/fc01-step5-repeat-native --output-on-failure -j 4
```

前一组216个独立Python方法，CTest88项。Python内会编译实际原生方法/头文件与内存ROS替身；不把它冒称DDS或Humble链接。原生adapter另外1方法。重复的37项专项/36项独立审查是上述方法的子集，不叠加数量。

## 2. 当前统一Humble构建

本轮实际使用的缓存目录如下；全新机器应替换SDK前缀。没有使用ARM64旧包内的ELF冒充本候选。

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH bash <<'BUILD'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
colcon --log-base /home/jikl62/.cache/fc01-vision-step4-20261118/log build \
 --base-paths native/src/jikl_fc01_system target/src/fc01_vision target/src/fc01_mission target/src/fc01_bringup \
 --build-base /home/jikl62/.cache/fc01-vision-step4-20261118/build \
 --install-base /home/jikl62/.cache/fc01-vision-step4-20261118/install \
 --packages-select jikl_fc01_system fc01_vision fc01_mission fc01_bringup --executor sequential
BUILD
```

工程现有全新构建入口仍为 `FC01_PX4_PREFIX=... FC01_TARGET_BUILD_DIR=... bash target/build.sh`。本轮实际四包构建日志为 `evidence/ros-build-native.log` 和 `ros-build-final.log`。新快照需要原生/消息/任务成套构建，不能只复制Python文件。

本轮 `ROS_BUILD.json` 绑定53个原生输入和实际x86 ELF。`SOURCE_BINDING.json`另绑定52个target生产文件、已安装mission各Python模块及代码提交。后续重建若字节不同，场景会拒绝旧证据；只能在成功构建后记录新的实际ELF/全部输入摘要作为另一个文件传入，不能只把HEAD改成新版本标签。

## 3. 真实ROS接口、相机与任务专项

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH bash <<'SMOKE'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=190
FC01_STEP5_CHECK_DIR=$(mktemp -d /tmp/fc01-step5-smoke.XXXXXX)
FC01_STEP5_PREFIX=/home/jikl62/.cache/fc01-vision-step4-20261118/install
/usr/bin/python3 tests/ros_generated_interface_check.py --prefix "$FC01_STEP5_PREFIX/jikl_fc01_system" --report "$FC01_STEP5_CHECK_DIR/interfaces.json"
/usr/bin/python3 tests/ros_vision_smoke.py --check-resource-budget --report "$FC01_STEP5_CHECK_DIR/adapter.json"
/usr/bin/python3 tests/ros_native_mission_smoke.py --output "$FC01_STEP5_CHECK_DIR/native-gate.json"
/usr/bin/python3 tests/ros_mission_horizontal_smoke.py --output "$FC01_STEP5_CHECK_DIR/horizontal.json"
/usr/bin/python3 tests/ros_mission_horizontal_smoke.py --decimated-attitude --output "$FC01_STEP5_CHECK_DIR/decimated.json"
/usr/bin/python3 tests/ros_camera_pipeline_smoke.py --prefix "$FC01_STEP5_PREFIX" --output "$FC01_STEP5_CHECK_DIR/camera"
printf '证据目录：%s\n' "$FC01_STEP5_CHECK_DIR"
SMOKE
```

11消息type/CDR合同、7适配检查、12原生授权、8水平、9不等频姿态、12相机专项分别通过。后四个轻量smoke不启动原生控制器；完整原生链见下方。

## 4. 真实监督器、相机到原生与完整故障矩阵

这些脚本必须使用空的localhost隔离域。每个输出目录必须尚不存在；命令顺序执行，避免同域混入别的回归源。

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH bash <<'MATRIX'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash
FC01_STEP5_PREFIX=/home/jikl62/.cache/fc01-vision-step4-20261118/install
FC01_STEP5_NODE="$FC01_STEP5_PREFIX/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
FC01_STEP5_PROOF=docs/investigations/20261118_vision_step5/evidence/ROS_BUILD.json
FC01_STEP5_OUT=$(mktemp -d /tmp/fc01-step5-matrix.XXXXXX)
for FC01_STEP5_SCENE in normal target_loss no_vision dds_loss; do
 /usr/bin/python3 tests/ros_step5_pipeline.py --prefix "$FC01_STEP5_PREFIX" \
  --build-evidence "$FC01_STEP5_PROOF" --domain 181 --attitude-hz 200 \
  --scenario "$FC01_STEP5_SCENE" --output "$FC01_STEP5_OUT/full-$FC01_STEP5_SCENE"
done
for FC01_STEP5_SCENE in track_duplicate track_reordered track_multisource preheight_flood track_lost track_timeout track_source track_reset track_origin horizontal_feedback_invalid horizontal_reset; do
 /usr/bin/python3 tests/ros_step5_fault_matrix.py --node "$FC01_STEP5_NODE" \
  --build-evidence "$FC01_STEP5_PROOF" --domain 182 \
  --scenario "$FC01_STEP5_SCENE" --output "$FC01_STEP5_OUT/fault-$FC01_STEP5_SCENE"
done
for FC01_STEP5_SCENE in feedback_stale quat_reset vertical_reset offboard_loss offboard_flag rc_takeover kill range_unavailable velocity_warning; do
 /usr/bin/python3 tests/ros_stage3_lifecycle.py --node "$FC01_STEP5_NODE" \
  --build-evidence "$FC01_STEP5_PROOF" --domain 185 \
  --scenario "$FC01_STEP5_SCENE" --output "$FC01_STEP5_OUT/safety-$FC01_STEP5_SCENE"
done
printf '证据目录：%s\n' "$FC01_STEP5_OUT"
MATRIX
```

四链：正常、失图、无任务、全DDS断开。原始姿态独立200Hz请求、平台桥50Hz，实际频率记录在result中。正常/失图/无任务完成连续下降→地面→ACK反例→实际Disarmed→释放→新READY。全DDS场景只证明锁存停发，其地面重置是显式清理。11矩阵中10项是完整故障下降，洪泛项是安全等待后正常LAND。九项安全矩阵是原生注入测试，不全都包含相机/mission或完整下降。

## 5. 无视觉精确差分与sanitizer

216项内包括旧Step3/Step4控制sink差分；针对本步实际起点额外运行如下，不改历史测试基准文件：

```bash
PYTHONPATH=.:tests FC01_VISION_STEP4_NODE_EVIDENCE=/tmp/fc01-step5-baseline-repeat python3 - <<'PY'
import unittest
import tests.test_vision_step4_node_sinks as check
check.BASELINE='747b9879967fb96fad5953f080b8765a7c9e874f'
r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(check))
raise SystemExit(not r.wasSuccessful())
PY
```

本轮1178条控制sink记录、707状态样本严格相同。完整ASan/UBSan实际节点与最终边界夹具、源码和编译参数保存在 `NATIVE_FINAL.tar.gz`，如下可重编译运行已归档夹具（不要运行旧ELF）：

```bash
FC01_STEP5_SAN=$(mktemp -d /tmp/fc01-step5-san.XXXXXX)
tar -xzf docs/investigations/20261118_vision_step5/evidence/NATIVE_FINAL.tar.gz -C "$FC01_STEP5_SAN"
for FC01_STEP5_LABEL in actual-node-and-snapshot final-boundary-negative; do
 FC01_STEP5_CASE="$FC01_STEP5_SAN/atomic-native-regression/sanitizers/$FC01_STEP5_LABEL"
 chmod 700 "$FC01_STEP5_CASE"
 g++ -std=c++17 -g -O1 -fno-omit-frame-pointer -fsanitize=address,undefined \
  -Wall -Wextra -Werror -pedantic -pthread \
  -I "$FC01_STEP5_CASE" -I "$PWD/native/src/jikl_fc01_system/include" \
  -I "$PWD/native/tests" "$FC01_STEP5_CASE/driver.cpp" -o "$FC01_STEP5_CASE/rebuilt"
 ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  FC01_SOCKET="$FC01_STEP5_CASE/control.sock" "$FC01_STEP5_CASE/rebuilt" > "$FC01_STEP5_CASE/repeat.log" 2>&1
done
```

观察夹具是最小API替身，不能称为ROS生产ELF全量sanitizer。归档中初次目录权限拒绝、旧JSON差分失败和后续PASS分开保留。

## 6. 证据校验与回退

```bash
(cd docs/investigations/20261118_vision_step5/evidence && sha256sum -c SHA256SUMS)
git show --stat 5c4dc14ddd84371d60387e7ff5227c94b7b6ee47
# 需要撤销本步候选时，在审查并保存自身改动后执行；本轮没有部署，无需改现场服务。
# git revert 5c4dc14ddd84371d60387e7ff5227c94b7b6ee47
```

旧ARM64发行记录已经阅读；它验证的是历史原生版本和维护安装，不含本步新消息/任务，不能替本候选背书。现场仍仅安排拆桨、未解锁的版本/话题/参数/时钟核对。真实相机帧率、外参、PX4采纳与飞行表现必须另有同次现场记录。

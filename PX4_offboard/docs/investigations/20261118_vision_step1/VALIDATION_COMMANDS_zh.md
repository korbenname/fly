# 第1步复验命令

实际环境：开发机Ubuntu 22.04.5 x86_64、Python /usr/bin/python3 3.10、ROS 2 Humble、默认rmw_fastrtps_cpp。不是Orange Pi实测。代码提交619736483a69df691eca49a5fbdb3135082fd8ed。所有ROS测试限定localhost和非零隔离域，不启动fc01_system_node，不运行systemd或连接串口。

## 统一构建

在工程根目录执行；SDK使用本机已匹配的安装前缀，其他主机需要换成自己的现有SDK。四包共用一套消息，不要source旧FC01 install再构建。

```bash
export FC01_PX4_PREFIX=/home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs
export FC01_TARGET_BUILD_DIR="$HOME/.cache/fc01-vision-step1-20261117"
bash target/build.sh
```

本轮从独立cache首次构建原生、随后统一加入三个target包，最终四包均通过。继续运行相同入口会增量构建，不会改用户板端版本。

## 消息、实际ROS与兼容回归

以下代码在实际开发机执行通过。需要选用未占用测试域；本轮154/155/156没有生产飞控节点。

```bash
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH bash <<'CHECK'
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step1-20261117/install/local_setup.bash
FC01_CHECK_DIR="$HOME/.cache/fc01-vision-step1-20261117"
/usr/bin/python3 tests/ros_generated_interface_check.py \
  --prefix "$FC01_CHECK_DIR/install/jikl_fc01_system" \
  --report "$FC01_CHECK_DIR/generated-interface.json"
ROS_DOMAIN_ID=154 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_target_interface_smoke.py \
  --prefix "$FC01_CHECK_DIR/install" --output "$FC01_CHECK_DIR/ros-smoke"
ROS_DOMAIN_ID=155 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_velocity_command_smoke.py \
  --report "$FC01_CHECK_DIR/legacy-velocity.json"
ROS_DOMAIN_ID=156 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_vision_smoke.py \
  --check-resource-budget --report "$FC01_CHECK_DIR/self-vision.json"
CHECK
```

关键成功标记：`messages: 7 / passed:true`、`FC01_STEP1_ROS_INTERFACE_PASS`、`REAL_ROS_VELOCITY_COMMAND_PASS`、`REAL_ROS_VISION_ADAPTER_PASS`。

新smoke使用实际Python消息发布、C++接收、C++图像节点、实际mission发布函数及生产ros2 launch。生产相机配置用不存在的video99999，验证缺相机也不崩溃、端点remap正确、正常SIGINT退出，未打开主机相机。合成黑图/红圆独立走vision_image_node，不能当作相机标定或性能测试。mission发布函数测试给显式旋转矩阵夹具，不模拟飞机动力学。

smoke会保存`ros2 interface show`和`ros2 topic info -v`原始输出。Humble/FastDDS远端history/depth显示UNKNOWN/0，是观察限制；接收器自身get_actual_qos核对KL1/volatile，不伪造远端深度。

## 单元/构建与语法

```bash
/usr/bin/python3 -m unittest tests.test_overnight_stage5_ros_contract tests.test_release_payload_policy -v
ctest --test-dir "$HOME/.cache/fc01-vision-step1-20261117/build/fc01_vision" --output-on-failure
/usr/bin/python3 -m compileall -q target/src tests/ros_target_interface_smoke.py
bash -n target/build.sh
git diff --check
```

本轮单元11项PASS，CTest 1个目标PASS（内部矩形/PnP、圆/测距、仅方位、stale四组检查）。不是把内部断言和CTest目标重复累计。

## 独立复现：二次remap

修复前`topic()`先解析再返回解析名，create_publisher又解析一次；双跳可落入/fmu。当前校验最终名但传原始名创建实体，同时检查解析后冲突。

```bash
ROS_DOMAIN_ID=169 ROS_LOCALHOST_ONLY=1 ros2 run jikl_fc01_system fc01_target_interface_node \
  --ros-args -r /fc01/interface/status:=/step1/status \
  -r /step1/status:=/fmu/in/vehicle_command
```

另开同域终端查看：`/step1/status`有1发布者；`/fmu/in/vehicle_command`无发布者。Ctrl+C退出。直接把status映射到/fmu会返回1；把status与velocity映射到同一端点也返回1。新smoke已自动覆盖。独立审查者真实运行3场景，原记录见INDEPENDENT_REVIEW。

## 当前不执行的现场核验

板端需要先在维护/拆桨/未解锁条件核对实际runtime.yaml、服务入口、固件build身份和ros2 topic info；本轮未连接设备，不能替用户报告成功。既有维护安装器及ARM64包保持原SHA，本轮未重新发行。最终板端验证和打包由后续排队任务安排。

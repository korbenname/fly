# 第 1 步交接：从离地开始保持同一个起飞点

分支 `readability/fc01-chinese-20261111`。开始时 HEAD `c93819330363229372fa43ba31f609b70b46a9ff`。已装 fix1 的发行源码是 `c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac`，其原生和 target 候选是 `5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`。开始时的原生、target 和状态查看源码与发行源码一致；本步最终代码为 `2a1348990976999a0549c8f00c85a2a6e6993b6b`，由 `4c99ec7405f3b3f929a63c9af8b8f16c9123e077` 和后续下降边界修复构成。原有未跟踪 MicoVIO 工具和测试未纳入。

- 重新读取 `korbenname/fly` 的 `target/`，提交 `c4b32b92455a9c25fb1bce1262e8426f249f8521`。上游 `mission_node.py:350` 保存第一帧 ENU 自机位置，`:1061` 使用起飞点的 XY 构造上方航点，`:1083` 转 NED。核对的文件 blob 为 `6b7962920ea177c8fe964079532463de712c9d7a`。上游使用 2.78 m，首帧捕获没有本机的地面证明。 [上游 mission_node.py](https://github.com/korbenname/fly/blob/c4b32b92455a9c25fb1bce1262e8426f249f8521/target/src/fc01_mission/fc01_mission/mission_node.py#L350)
- fix1 的 `attitude_altitude_core.hpp` 在 `navigation_allowed()` 要求离地且稳定到高，其 `HeightHold` 视觉分支才可接纳任务水平输出。`ros_step5_pipeline.py` 的固定 XY 模型不能用于爬升期纠偏验收。本步保留这一外部运动授权逻辑，另加独占的起飞点保持路线。
- 定制 FC01 PX4 `07983bfb894b9372d7520d43ac10dc792d0b4b9f` 的本地构建含两个订阅。`dds_topics.yaml:115,133` 声明 `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`，生成的 `dds_topics.h:831,861` 创建无版本后缀的 reader，`:595,655` 解码后发布 uORB。`ModeUtil/control_mode.cpp:123` 用 `position=true` 开启 PX4 位置、速度、姿态及低层控制，`MulticopterPositionControl.cpp:435–544` 消费目标并处理起飞。板级 ELF 中查到 `mc_pos_control_main` 与 `PositionControl::update(float)`。实际实机刷入镜像与此构建的关系仍需核对。 [固件证据](evidence/PX4_INTERFACE.json)

`px4_takeoff_position_hold` 缺省 `false`。候选模式保持 `attitude_altitude` 的人工 RC 生命周期和部署入口，要求 `rc_navigation_enabled=true`、程序解锁关闭、1.5 m、8 s、槽位 1、两个视觉开关关闭。从未解锁、接地静止且 PX4 局部位置和速度有效的同一观测捕获 `(x0,y0,z0)`，绑定本地位置 DDS 写者世代、原点和 reset。进入解锁配对后冻结，离地、到高、航向变化都不会改写该点。只有位于已确认地面且进入下一轮准备时才可建立新锚。[RC 地面锚](../../../native/src/jikl_fc01_system/include/jikl_fc01_system/rc_navigation.hpp)

FC01 管理人工授权、地面锚、离地与到高证据、任务期限及发布边界。准备期只发 `position=true` 心跳，等当前 PX4 状态与 RC 边沿配对后才送轨迹。爬升前段只发垂直目标，XY 位置和速度均为 NaN，水平加速度为零。离地证据成立后，目标变为 `[x0,y0,z0-h_ref]`，`h_ref` 按已有爬升速度推进至 1.5 m。PX4 使用自己的位置、速度、姿态及推力控制和现有 `MPC_*` 参数，原 FC01 推力 PI 在这一路不构造。下降经同一轨迹出口用有限的正 NED `vz`，因为定制 PX4 着陆检测要识别明确的下行速度目标。默认 0.30 m/s 依据历史 `LNDMC_Z_VEL_MAX=0.25 m/s`，需复核现场实际值及限制。正常下降使用新接地证据触发上锁，等独立 PX4 上锁状态后释放模式。修复了超过 1.5 m 后旧高度字段可能误拦速度下降计划的问题，专门测试也覆盖目标已发、心跳前坐标 reset 的部分提交。[最终发布边界](../../../native/src/jikl_fc01_system/include/jikl_fc01_system/publication_boundary.hpp)

位置模式只创建轨迹与心跳发布器，拒绝竞争控制发布者，保持目标先提交、心跳后提交，并在出口复核反馈、权限和锚点。USB 主相机和外部航点没有这个模式的运动权限，包括到高以后。MTF‑02P 仍走原串口直连 PX4，香橙派只接融合反馈。接管、Kill、Offboard 退出和失联沿原策略撤销控制或转入受监督下降。回退时将该参数保持或恢复为 `false`，旧姿态＋推力路线也可通过保留的 fix1 安装目录恢复。本步没有修改实机配置、PX4 参数或固件。

## 最终验证

Ubuntu x86_64，ROS 2 Humble，Fast DDS 本机隔离域。C++ CTest **89/89**，Python 状态显示 **9/9**，固定 build 输入的原生 Humble 构建通过。隔离 ROS **9/9**，包括无 USB 相机的全流程、DDS 断流、Kill、Offboard 退出、RC 接管、XY 失效、XY reset、模式未确认和超过 8 秒。另跑旧姿态模式的正常升降 **1/1**。对 fix1 的五项严格差分通过，其中真实适配器配内存 ROS 替身的飞行测试比较了 707 帧状态和 588 轮有序发布。使用更早期 `f06715a` 的两个全 JSON 字节检查仍因后续已有视觉诊断字段而失败，保留原日志并以实际安装基线作为本步判据。[场景结果](evidence/ROS_SCENARIOS.json) · [fix1 差分](evidence/FIX1_DIFFERENTIAL.json)

在 NED 起飞点 `(2.25, -1.75, 2.0) m`，高度约 0.025 m 时注入横向速度扰动。高度 **0.212 m** 已收到同一 XY 锚点的实际 ROS 轨迹目标。此时模型实际位置 `(2.474, -1.878, 1.788) m`，速度 `(+0.309, -0.177, -0.378) m/s`，定制 PX4 位置控制源码计算的归一化 NED 推力 `(-0.0461, +0.0264, -0.4871)` 指向水平误差反向。注入 90° 航向反馈变化后，横向速度转向起飞点，最终保持目标为 `(2.25, -1.75, 0.5) m`。本次峰值横漂 **0.333 m**，悬停末 2 秒最大横向误差 **0.0108 m**、RMS **0.0087 m**，最大垂向误差 **0.0303 m**，高度超调 **0 m**，仿真接收端最长心跳间隔 **55.2 ms**。降落、独立上锁确认和重新准备通过。轨迹、心跳、状态及模型反馈保存在归档内。[扰动结果](evidence/DISTURBANCE.json) · [原始记录](evidence/ROS_TRACES.tar.gz)

## 复验

以下命令从 FC01 仓库根目录执行。需要本机 Humble、固定 v1.16 消息 SDK、定制 PX4 源码和已有的板构建消息头。用新临时目录构建，保留原安装树。

```bash
(
set -e
fc01_check=$(mktemp -d /tmp/fc01-position-step1.XXXXXX)
cmake -S native/tests -B "$fc01_check/native" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$fc01_check/native" -j 6
ctest --test-dir "$fc01_check/native" --output-on-failure
PYTHONPATH="$PWD/tests:$PWD" python3 -m unittest tests.test_stage3_status_readiness

source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon --log-base "$fc01_check/log" build \
  --base-paths native/src/jikl_fc01_system --build-base "$fc01_check/build" \
  --install-base "$fc01_check/install" --packages-select jikl_fc01_system --executor sequential
source "$fc01_check/install/local_setup.bash"
fc01_node="$fc01_check/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
python3 tests/px4_takeoff_position_model.py \
  --firmware /home/jikl62/FC01_DDS_V3_NATIVE_BUILD \
  --sitl-build /home/jikl62/PX4-Autopilot/build/px4_sitl_default \
  --output "$fc01_check/px4-model"

git ls-files -z native/src/jikl_fc01_system | python3 -c '
import hashlib, json, pathlib, sys
paths = [pathlib.Path(p.decode()) for p in sys.stdin.buffer.read().split(bytes([0])) if p]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
print(json.dumps(dict(binary_sha256=sha(pathlib.Path(sys.argv[1])),
                      production_sha256={str(p): sha(p) for p in paths}), indent=2))
' "$fc01_node" > "$fc01_check/ROS_BUILD.json"
for fc01_case in disturbance dds_loss kill offboard_loss rc_takeover xy_invalid xy_reset no_confirm late_slot; do
  /usr/bin/python3 tests/ros_step1_takeoff_position.py --domain 209 \
    --node "$fc01_node" --build-evidence "$fc01_check/ROS_BUILD.json" \
    --px4-model "$fc01_check/px4-model/px4_takeoff_position_model.so" \
    --scenario "$fc01_case" --output "$fc01_check/$fc01_case"
done
/usr/bin/python3 tests/ros_stage3_lifecycle.py --domain 209 --scenario normal \
  --node "$fc01_node" --build-evidence "$fc01_check/ROS_BUILD.json" --output "$fc01_check/legacy"
)
```

将现有 Stage2 两组严格差分的 `BASELINE` 参数临时指向已装 fix1 的发行源码。生产节点、控制状态及 sink 保持原样编译和比较。

```bash
PYTHONPATH="$PWD/tests:$PWD" python3 - <<'PY'
import unittest
from unittest.mock import patch
import test_stage2_core_equivalence as core
import test_stage2_powered_node_equivalence as powered
baseline = 'c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac'
with patch.object(core, 'BASELINE', baseline), patch.object(powered, 'BASELINE', baseline):
    suite = unittest.defaultTestLoader.loadTestsFromModule(core)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(powered))
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful())
PY
```

## 现场缺项

- 同一次板端连接的固件构建身份、两个 DDS reader、源时钟，以及 `xy_valid`、`v_xy_valid`、`heading_good_for_control`、`dead_reckoning` 和严格 `FailsafeFlags.local_position_invalid/local_velocity_invalid`。手动 Position 的放宽定位门与 Offboard Position 的严格门不同。已有 `log_82` 仅为约 2.2 秒接地记录，`heading_good_for_control=false`、`cs_opt_flow=0`，无法代表现在的 MTF‑02P 实飞或用于放宽候选准入。[历史 EKF 基线](../20261118_vision_step2/EKF_BASELINE_zh.md)
- ROS 合成端验证了实际 ROS 消息及 PX4 源码的控制律响应。机体是带地面的三轴点质量，航向是注入的实测反馈替身。未运行整套固件 SITL 的 Commander、EKF、着陆检测与执行器链，也未验证 ARM64 调度、UART/XRCE、地效或实际转向。**尚未证明达到手动 Position 的实飞表现。** 用户没有提供可配对的当前 MTF‑02P 空中 ULog。第 2 步应以本步最终源码、现有 fix1 回退点及现场新证据继续，不从本模型推定已经实飞通过。

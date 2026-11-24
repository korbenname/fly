# 第 2 步交接：定点起飞后的非接触观察

工作分支 `readability/fc01-chinese-20261111`。起点 HEAD `cbcd41eed3452c780c566d3db9df362c1e5003a3` 包含第 1 步最终代码 `2a1348990976999a0549c8f00c85a2a6e6993b6b` 与仿真证据。本步代码 `b2ee5215e03a7c375714efb5f562f788473369a9`，ROS 测试提交 `ecd3f9e557de093d82c283bad08c049770d51096`。原未跟踪 MicoVIO 测试和工具仍保留。未改 PX4、EKF、UART、已安装 fix1 或现场文件。

重新读取了 `korbenname/fly` 的 `target/`，确认 `main` HEAD 仍为 `c4b32b92455a9c25fb1bce1262e8426f249f8521`，没有上游更新。核对文件为 `robot.launch.py`、`camera_source.cpp`、`vision_node.cpp`、`mission.yaml` 和 `mission_node.py`，原始 blob 身份见 [UPSTREAM.json](../../../target/UPSTREAM.json)。

| 状态 | 上游（固定 SHA） | 本步实机入口 |
|---|---|---|
| 起飞、悬停 | `_on_pose:350` 保存首个仿真位姿，`_update_mission:719–746` 到 2.78 m 后定时过渡 | 只采用第 1 步未解锁地面锚。USB 可在地面采图，任务运动要等真实离地且高度、垂速稳定在 1.5 m。 |
| 无目标 | `_publish_navigation:1035–1084` 的 `takeoff/seek` 可发布仿真航点，`_seek_command:861–907` 还有仿真地图先验和搜索 | 默认守 `x0,y0`。只有已显式配置、当前原点对齐、原生核对同一 XY 且在任务半径内的航点才允许前移。 |
| 航点撤销 | `_publish_navigation:1035–1041` 有新鲜目标或离开指定阶段即发送无效航点 | 连续新帧有效锁定后撤销航点，跟踪优先。速度封套稍后到时仅允许有时限的零水平速度，已撤销航点不得复活。 |
| 视觉有效 | `_on_state:396–405` 用位置有效位与有限性，`_seek_command:766–792` 计算跟踪距离和速度 | 现有相机时效、独立 PX4 自机反馈时间对齐、校准、安全间距及原生授权都成立才消费水平速度。 |
| 输入丢失 | `_tick:513–518` 自机过期即返回，`_seek_command:841–860` 丢目标后可继续搜索 | 初始无图可守点。有过视觉移动后断图、旧帧或失锁，撤销速度并在当前 XY 受监督下降。若 PX4 参考已变或定位无效，停止旧坐标目标和心跳，原生锁存并让 PX4 保护接管。 |

上游函数行号见 [mission_node.py](https://github.com/korbenname/fly/blob/c4b32b92455a9c25fb1bce1262e8426f249f8521/target/src/fc01_mission/fc01_mission/mission_node.py)。

## 实际入口与出口

`position` 是第 1 步的单独定点模式，USB 相机任务不启动。`position-track` 在同一个原生进程上接任务。包内 `navigation_prebuilt_service.py` 按已经核验的现场配置调用 `horizontal_first_test.launch.py → robot.launch.py`。V4L2 相机由 `CameraSource` 单独采图，`VisionNode` 保留帧戳和序号、识别并发布相机相对观测。PX4 自机位置、姿态与速度来自现有 DDS 桥。MTF‑02P 的光流与测距仍直接供 PX4 融合。生产固定非接触 `track`，上游接触/冲撞分支只在仿真入口可用。

在完成后续候选安装且处于维护状态时，现有配置入口为 `python3 -B "$entry/navigation_vision_mode.py" position --entry "$entry" --maintenance`，或 `python3 -B "$entry/navigation_vision_mode.py" position-track --site "$HOME/fc01_site.json" --entry "$entry" --maintenance`。这里 `entry` 必须是包含本步源码与预编译产物的版本，调用前服务需停止，实际命令尚未在板端执行。现场配置需要已有设备、内外参、目标模型、延迟和安全间距记录。默认 `hold`。若显式选择 `waypoint`，还须当前 PX4 原点标识及测量后的 ENU 航点，配置工具转换为 NED 并绑定原生允许的精确点。维护重复安装、备份和原 `altitude`、`track` 选择继续兼容。

原生以同一 `TrajectorySetpoint` 发布器发送轨迹，以同一 `position=true` 心跳保活。起飞段沿用锚点。到高后航点生成受限的 NED XY 位置参考，跟踪经匹配的 FRD 姿态投影为 NED XY 速度，二者互斥。Z 均来自 `z0-1.5`，航点 z 和视觉垂速没有垂向权限。原生用当前高度稳定、原点、反馈时效、命令水位、发布者唯一性和各限界逐次复核。跟踪开始后的失效禁止退回旧航点；保护下降用新的固定 XY，位置估计重置时停止发布旧参考。PX4 原位置控制器负责推力及姿态，实际电机响应仍受 PX4 参数影响。

## 最终验证

四包 Humble 构建成功。C++ 测试 89/89，Python 维护与状态 102/102、任务与保护 33/33。第 1 步旧路线五项严格差分通过，并用最终 ROS 构建复跑第 1 步的无相机扰动、降落、上锁和重新准备。最终原生 ELF 与提交逐文件对应。[构建身份](evidence/ROS_BUILD.json)

隔离 ROS 10/10。缺 USB 时实际生产 V4L2 节点报告设备缺失，定点升降通过；其余视觉场景用合成图像喂真实图像节点。验证了到高前的有效目标只能用于观察、无目标默认守点、只有授权航点可前移、错误原点继续守点、有效锁定撤销航点并产生水平运动。测试中有效运动始终保持 `z0-1.5`，原生控制图有一个轨迹写者、一个心跳写者、零姿态目标写者。

重放旧图、停止图像和继续发送新空图后，仿真接收端分别约在 69、322、71 ms 收到保护下降目标。XY reset 后在约 125 ms 观察到 Safety，并确认停止旧轨迹和心跳。受控下降保留退出时的位置，不复用旧航点或速度。航点发布者停止发包约 333 ms 后收到保护下降目标。初次测试错误地将静默超时预期写为先刹停，保留原失败并使用同一 ELF 修正测试、重跑全部十个场景。上述耗时含本机观察与调度延迟，现场需另测。[各场景结果](evidence/SCENARIOS.json) · [原始消息和反馈](evidence/ROS_TRACES.tar.gz)

## 复验

从 FC01 仓库根运行以下命令。PX4 模型复用第 1 步固定源码和生成消息头，并在真实 ROS 端点用三轴动力学回放。两条图像链分别验证生产 V4L2 节点识别设备缺失，以及 `vision_image_node` 接合成 ROS 图像和生产 mission/桥接。实际 USB 光学测量、120 fps 与现场校准未测。

~~~bash
(
set -e
fc01_check=$(mktemp -d /tmp/fc01-position-vision-step2.XXXXXX)
cmake -S native/tests -B "$fc01_check/native" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$fc01_check/native" -j 6
ctest --test-dir "$fc01_check/native" --output-on-failure
PYTHONPATH="$PWD/tests:$PWD" python3 -m unittest \
  tests.test_position_track_mode tests.test_navigation_vision_mode \
  tests.test_navigation_prebuilt_service tests.test_navigation_prebuilt_install \
  tests.test_stage3_status_readiness tests.test_mission_horizontal \
  tests.test_mission_pending_alignment tests.test_vision_step5_safety_edges \
  tests.test_prebuilt_vision_status
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon --log-base "$fc01_check/log" build \
  --base-paths native/src/jikl_fc01_system target/src/fc01_vision target/src/fc01_mission target/src/fc01_bringup \
  --build-base "$fc01_check/build" --install-base "$fc01_check/install" \
  --packages-select jikl_fc01_system fc01_vision fc01_mission fc01_bringup --executor sequential
source "$fc01_check/install/local_setup.bash"
python3 tests/px4_takeoff_position_model.py --build-only \
  --firmware /home/jikl62/FC01_DDS_V3_NATIVE_BUILD \
  --sitl-build /home/jikl62/PX4-Autopilot/build/px4_sitl_default \
  --output "$fc01_check/px4-model"
fc01_node="$fc01_check/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
git ls-files -z native/src/jikl_fc01_system | python3 -c '
import hashlib, json, pathlib, sys
paths = [pathlib.Path(p.decode()) for p in sys.stdin.buffer.read().split(bytes([0])) if p]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
print(json.dumps(dict(binary_sha256=sha(pathlib.Path(sys.argv[1])),
                      production_sha256={str(p): sha(p) for p in paths}), indent=2))
' "$fc01_node" > "$fc01_check/ROS_BUILD.json"
for fc01_case in unplugged hold waypoint wrong_origin waypoint_silence handoff old_frame stream_loss lock_loss xy_reset; do
  /usr/bin/python3 tests/ros_step2_position_vision.py --domain 209 \
    --prefix "$fc01_check/install" --build-evidence "$fc01_check/ROS_BUILD.json" \
    --px4-model "$fc01_check/px4-model/px4_takeoff_position_model.so" \
    --scenario "$fc01_case" --output "$fc01_check/$fc01_case"
done
)
~~~

第 3 步仍须核对当前定制 PX4 的严格位置有效性和 DDS 端点、Orange Pi Zero 3W 的负载及 USB 到帧/时延、现场标定与实际 PX4 是否采纳目标。本阶段没有 ARM64 新包、现场相机或实飞验证，不能从仿真推出手动 Position 的实飞表现。

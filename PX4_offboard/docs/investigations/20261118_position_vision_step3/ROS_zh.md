# 定点起飞和视觉观察集成验证

使用 Ubuntu 22.04.5 x86_64、ROS 2 Humble 和本机隔离的 Fast DDS。四个实用 ROS 包的新构建通过。构建输入来自 `53a37ddb807c7a39c324e506808cd5bbcac9891a`，126 个输入文件逐项与发行源码 `c7de1387c635cf1733e1d1e1a20a8c24d7ad7752` 一致。测试夹具提交为 `18bb181ddf09c01179fd53d69d47a4d3e7b84655`。上游 `target/` 为 `c4b32b92455a9c25fb1bce1262e8426f249f8521`，本轮重新核对五个入口文件的 Git blob 和 SHA‑256。本轮一次 `git ls-remote` 返回相同 HEAD，随后复查遇到 TLS 连接中止。 [四包构建](evidence/humble-build.log) · [逐文件构建摘要](evidence/ROS_BUILD.json)

位置输出由实际 `fc01_system_node` 通过 ROS 发送。定制 PX4 `07983bfb894b9372d7520d43ac10dc792d0b4b9f` 的 PositionControl 和 Takeoff 源码驱动带地面的三轴点质量模型。RC、PX4 状态和姿态为模拟反馈。图像来自合成 ROS 发布者，缺相机场景则运行实际 V4L2 节点并指向不存在的设备。以下数据属于 x86 仿真，Orange Pi、ARM64 飞行调度、完整 PX4 固件链路及 MTF‑02P 融合均需各自的现场证据。

| 流程 | 爬升峰值横漂 | 悬停误差 | 1.5 m 超调 | 任务授权 |
|---|---:|---:|---:|---|
| A 基础定点 | 0.318 m，另次 0.333 m | 末两秒峰值 0.0110 m，RMS 0.0089 m | 0 m | 反馈进入 Offboard 后 10.041 s 达到当前高度稳定门，外部任务权限保持关闭 |
| B 任务模式，默认无目标守点 | 0.209 m | 末两秒峰值 0.0081 m | 0 m | 9.820 s |
| B 任务模式，实际 V4L2 节点无设备 | 0.208 m | 末两秒峰值 0.0079 m | 0 m | 9.870 s，视觉观测无效 |
| B 显式航点转视觉跟踪 | 0.209 m | 切换后主动水平运动 | 0 m | 9.924 s，此时模型高度 1.449 m、NED 垂速 −0.024 m/s |

A 在 `(2.25, −1.75, 2.0) m` 锁定起飞点。约 0.025 m 时注入 `(+0.35, −0.20) m/s` 的横向速度。高度 0.211 m 时 ROS 目标已保持原始 XY，模型加速度指向起飞点。注入航向变化 90° 后，在约 0.399 m 时模型速度已反向收敛。末两秒最大高度误差 0.0305 m，仿真接收者最长心跳间隔 51.5 ms。正常降落、独立上锁确认、Offboard 释放与再次准备均通过。

B 的航点场景按 1.5 m 目标上升并在 0.213 m 纠偏。授权前所有轨迹都守原起飞点，地面已识别目标的各场景也未提前给出有效任务意图。航点场景在进入 Offboard 后 11.907 s 接近配置的 NED `(2.63, −1.55)`。加入目标后确认航点撤销，在 12.442 s 记录到模型运动及对应 ROS 轨迹 `velocity=[0.2665, 0.0472, null]` m/s、`position=[null, null, 0.5]` m。此刻模型 NED 位置为 `(2.598, −1.565, 0.516) m`。图中始终只有一个轨迹发布者和一个位置模式心跳发布者。

| 视觉活动期间注入 | 实际 ROS 输出与状态结果 |
|---|---|
| 捕获戳相同的重复帧 | 相机报告 `DUPLICATE_FRAME`，约 35 ms 发下行目标，水平跟踪撤销 |
| 超过 600 ms 的旧图 | 相机报告 `SOURCE_TIME_STALE_OR_INVALID`，约 35 ms 发下行目标 |
| 停图 | 约 336 ms 发下行目标，撤销旧水平速度 |
| 新图丢锁 | 约 81 ms 发下行目标 |
| 人工 RC 摇杆介入 | 约 32 ms 发下行目标，模拟 PX4 确认退出 Offboard 后锁存 `OPERATOR_TAKEOVER_LATCHED` |
| 仅 PX4 局部位置过期，其他 DDS 与图像持续 | 约 83 ms 撤销跟踪并发下行目标，约 634 ms 观察到 `DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE`，原生 `local_fresh=false` |
| PX4 XY 坐标重置 | 约 131 ms 锁存 `PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID`，撤销轨迹和心跳 |

保护下降阶段未续发旧水平速度。已有锁存时，即便恢复输入也未恢复本轮控制。上表时延从测试端注入到接收端观测，参考同一次试验内的单调时间。结果共通过 11 项。尚未证明达到手动 Position 的实飞表现。 [结构化结果与事件链](evidence/ROS_RESULTS.json) · [ROS 轨迹、心跳和反馈](evidence/ROS_TRACES.tar.gz)

## 复验命令

在仓库根目录执行。需要 x86 Humble、现有 FC01 v1.16 消息 SDK、定制 PX4 固定提交和生成的板级消息头。

```bash
(
set -eo pipefail
fc01_check=$(mktemp -d /tmp/fc01-position-step3.XXXXXX)
fc01_source=c7de1387c635cf1733e1d1e1a20a8c24d7ad7752
mkdir "$fc01_check/source"
git archive "$fc01_source" native/tools native/src/jikl_fc01_system \
  target/src/fc01_vision target/src/fc01_mission target/src/fc01_bringup |
  tar -xf - -C "$fc01_check/source"
source /opt/ros/humble/setup.bash
source /home/jikl62/FC01_R632_P0_ATTITUDE_AUDIT_CANDIDATE/install_r632/px4_msgs/share/px4_msgs/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$fc01_check/log" build \
  --base-paths "$fc01_check/source" --build-base "$fc01_check/build" \
  --install-base "$fc01_check/install" --packages-select \
  jikl_fc01_system fc01_vision fc01_mission fc01_bringup --executor sequential
source "$fc01_check/install/local_setup.bash"
python3 tests/px4_takeoff_position_model.py --build-only \
  --firmware /home/jikl62/FC01_DDS_V3_NATIVE_BUILD \
  --sitl-build /home/jikl62/PX4-Autopilot/build/px4_sitl_default \
  --output "$fc01_check/px4-model"
fc01_node="$fc01_check/install/jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node"
fc01_model="$fc01_check/px4-model/px4_takeoff_position_model.so"
fc01_proof="$fc01_check/ROS_BUILD.json"
python3 - "$fc01_node" "$fc01_source" "$fc01_proof" <<'PY'
import hashlib
import json
from pathlib import Path
import subprocess
import sys

node, commit, out = sys.argv[1:]
pinned = json.loads(Path('docs/investigations/20261118_position_vision_step3/evidence/ROS_BUILD.json').read_text())
sources = {name: hashlib.sha256(subprocess.check_output(['git', 'show', commit+':'+name])).hexdigest()
           for name in pinned['production_sha256']}
Path(out).write_text(json.dumps(dict(binary_sha256=hashlib.sha256(Path(node).read_bytes()).hexdigest(),
                                    production_sha256=sources, source_commit=commit), indent=2))
PY
/usr/bin/python3 -B tests/ros_step1_takeoff_position.py --domain 217 --node "$fc01_node" \
  --build-evidence "$fc01_proof" --px4-model "$fc01_model" --scenario disturbance \
  --output "$fc01_check/disturbance"
for fc01_case in old_frame duplicate_frame rc_takeover px4_local_stale handoff stream_loss unplugged hold lock_loss xy_reset; do
  /usr/bin/python3 -B tests/ros_step2_position_vision.py --domain 218 --prefix "$fc01_check/install" \
    --build-evidence "$fc01_proof" --px4-model "$fc01_model" --scenario "$fc01_case" \
    --output "$fc01_check/$fc01_case"
done
)
```

#!/usr/bin/env bash
# 唯一原生消息包+三个目标业务包，在同一工作空间生成C++/Python类型支持。
set -eo pipefail
FC01_REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -e "$FC01_REPO_DIR/target/src/jikl_fc01_system" ]]; then
  echo '拒绝重复 jikl_fc01_system；消息唯一来源为 native/src/jikl_fc01_system' >&2
  exit 2
fi
# 显式提供当前PX4 v1.16 SDK安装前缀，不把旧overlay消息碰巧可用当作构建依据。
if [[ -z "${FC01_PX4_PREFIX:-}" || ! -f "$FC01_PX4_PREFIX/share/px4_msgs/local_setup.bash" ]]; then
  echo '请设置 FC01_PX4_PREFIX 为已有兼容 px4_msgs 安装前缀；不需重装ROS。' >&2
  exit 2
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source "$FC01_PX4_PREFIX/share/px4_msgs/local_setup.bash"
FC01_BUILD_DIR="${FC01_TARGET_BUILD_DIR:-$FC01_REPO_DIR/target}"
mkdir -p "$FC01_BUILD_DIR"
colcon --log-base "$FC01_BUILD_DIR/log" build \
 --base-paths "$FC01_REPO_DIR/native/src" "$FC01_REPO_DIR/target/src" \
 --packages-select jikl_fc01_system fc01_vision fc01_mission fc01_bringup \
 --build-base "$FC01_BUILD_DIR/build" --install-base "$FC01_BUILD_DIR/install" \
 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
 -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"

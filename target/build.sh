#!/usr/bin/env bash
# Incremental colcon build for this repository.
#
#   ./build.sh                     # deployment packages (default)
#   ./build.sh fc01_vision         # only the named package(s)
#   ./build.sh sim                 # simulation workspace (sim_ws)
#   ./build.sh sim fc01_vision_sim # simulation, only one package
#
# Colcon already rebuilds only what changed (same build/ directory, same
# targets). This script adds --symlink-install so Python/YAML/launch edits
# need no rebuild, and enables ccache for the C++ package when available.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ROS setup scripts reference optional variables, so relax `set -u` while
# sourcing them.
set +u
source /opt/ros/humble/setup.bash
set -u

CMAKE_ARGS=()
if command -v ccache >/dev/null 2>&1; then
  CMAKE_ARGS=(
    --cmake-args
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  )
fi

if [[ "${1:-}" == "sim" ]]; then
  shift
  if [[ -f "${ROOT}/install/setup.bash" ]]; then
    set +u
    source "${ROOT}/install/setup.bash"
    set -u
  else
    echo "warning: ${ROOT}/install/setup.bash not found; build the deployment workspace first" >&2
  fi
  cd "${ROOT}/sim_ws"
  if [[ $# -eq 0 ]]; then
    set -- fc01_vision_sim
  fi
  exec colcon build --symlink-install --packages-select "$@" "${CMAKE_ARGS[@]}"
fi

cd "${ROOT}"
if [[ $# -eq 0 ]]; then
  set -- jikl_fc01_system fc01_vision fc01_mission fc01_bringup
fi
exec colcon build --symlink-install --packages-select "$@" "${CMAKE_ARGS[@]}"

# 第6步验证命令与证据

工作目录：`/home/jikl62/FC01_OrangePi_Navigation_20261028`。发行源码`c6fe7edaec84ce3dad68d6584b1d7fbb5c3c7bac`，原生/target候选`5c4dc14ddd84371d60387e7ff5227c94b7b6ee47`。实际原始日志目录为`/home/jikl62/.cache/fc01-arm64-step6-20261118/`。本文区分最终fix1包、早期失败和最终实际事务；只把有真实完成日志的项目写为通过。

## 宿主Python回归

实际宿主为x86_64 Ubuntu22.04。最终统一回归180项通过，包含此前分组及独立审查的重叠测试，不把历次通过计数相加。

```bash
PYTHONPATH=.:tests python3 -m unittest tests.test_arm64_bundle tests.test_arm64_runtime tests.test_navigation_prebuilt_install tests.test_navigation_prebuilt_service tests.test_navigation_vision_mode tests.test_prebuilt_vision_status tests.test_navigation_feedback_freshness tests.test_stage3_status_readiness tests.test_navigation_service tests.test_prebuilt_maintenance_lifecycle tests.test_release_payload_policy tests.test_overnight_stage5_ros_contract -v
git diff --check
```

最终证据：`python-final-fix1.log`，`Ran 180 tests in 14.450s`、`OK`。`python-step6-final.log`、`python-release-contract.log`是较早分组记录，保留但不当作最终最新计数。

补跑未修改的监督器、Agent恢复和日志堵塞回归：

```bash
PYTHONPATH=.:tests python3 -m unittest tests.test_standby tests.test_standby_writer -v
```

`python-transport-unchanged.log`记录26/26 PASS、9.000秒。Agent替身断开/恢复时native不被替换、只收到STATUS；不等于真实XRCE/UART恢复验证。

## ARM64构建

使用已有QEMU-aarch64用户态/PRoot Ubuntu22.04、GCC11、ROS2 Humble。不是x86交叉产物，也不是真实Zero3W。`/delivery`是构建cache绑定目录；构建前将已提交候选原生/target源码归档到build-work，px4_msgs源码与已审查SDK消息逐文件对照零差异。

```bash
/home/jikl62/.cache/fc01-arm64-20261116/arm-run /bin/bash -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /delivery/build-work
colcon --log-base log-step6 build --merge-install   --base-paths native/src target/src   --packages-select jikl_fc01_system fc01_vision fc01_mission fc01_bringup   --executor sequential   --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF   -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
'
```

证据：`CONTROL_SOURCE.json`、`arm-colcon-build.log`、包内`ARM64_BUILD.json`及`ACTUAL_BUILD_SOURCE_BINDING.json`。4包11min34s成功；vision仅有未使用PYTHON_EXECUTABLE变量的CMake警告。构建时发行脚本为6a9d0b3；后续发行修复没有改动111份实际编译输入，逐字SHA绑定通过，不将历史构建记录改写为c6fe7ed。构建机为编译安装OpenCV/cv_bridge/launch开发依赖，**没有在目标VM额外apt安装这些运行库以伪装离线包完整**。

## 依赖扫描与收集

以下是最终实际扫描选项；不使用先前失败扫描结果。运行前将当前已提交`build_arm64_runtime.py`复制为cache中的`step6-runtime-collector.py`。

```bash
/home/jikl62/.cache/fc01-arm64-20261116/arm-run /bin/bash -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source /delivery/build-work/install/local_setup.bash
python3 /delivery/step6-runtime-collector.py scan  --install-tree /delivery/build-work/install --report /delivery/step6-runtime-ldd-full.json  --ros-package ros-humble-cv-bridge --ros-package ros-humble-sensor-msgs  --ros-package ros-humble-ros2launch --ros-package ros-humble-ros2cli  --ros-package ros-humble-ros2pkg --ros-package ros-humble-launch  --ros-package ros-humble-launch-ros --ros-package ros-humble-launch-xml  --ros-package ros-humble-launch-yaml --ros-package ros-humble-osrf-pycommon  --ros-package ros-humble-composition-interfaces --ros-package ros-humble-ament-copyright  --ros-package ros-humble-ament-lint --ros-package ros-humble-lifecycle-msgs  --python-package python3-numpy --python-package python3-netifaces  --python-package python3-argcomplete --python-package python3-importlib-metadata  --python-package python3-packaging --python-package python3-lark  --python-package python3-yaml --python-package python3-empy  --python-package python3-catkin-pkg-modules --python-package python3-zipp  --python-package python3-more-itertools --python-package python3-pyparsing  --python-package python3-dateutil --python-package python3-docutils  --python-package python3-six --python-package python3-roman  --python-package python3-pkg-resources
'
python3 build_arm64_runtime.py collect  --rootfs /home/jikl62/.cache/fc01-arm64-20261116/rootfs  --install-tree /home/jikl62/.cache/fc01-arm64-20261116/build-work/install  --output /home/jikl62/.cache/fc01-arm64-step6-20261118/install-full  --ldd-report /home/jikl62/.cache/fc01-arm64-20261116/step6-runtime-ldd-full.json
```

`collect`目标必须不存在，拒绝覆盖已收集目录。证据`runtime-scan-full.log`报告89输入ELF、278来源包、1700外部文件；`runtime-collect-full.log`报告7582安装文件、2187复制记录、59基础目标依赖。完整清单位于包内`install/RUNTIME_DEPENDENCIES.json`，不把这些数量解释为实际设备性能。

## 生成最终发行与独立解包

`ARM64_BUILD.json`须由实际源码和上述新安装树逐文件SHA产生，覆盖59原生、52target输入和7582安装文件，不能重新贴旧控制器身份。实际生成命令：

```bash
python3 build_arm64_bundle.py  --install-tree /home/jikl62/.cache/fc01-arm64-step6-20261118/install-full  --build-evidence /home/jikl62/.cache/fc01-arm64-step6-20261118/ARM64_BUILD.json  --version 20261118_vision_step6_fix1  --control-candidate 5c4dc14ddd84371d60387e7ff5227c94b7b6ee47  --output dist/arm64/20261118_vision_step6_fix1
```

最终证据为`build-bundle-fix1.log`和`FINAL_DELIVERY.json`；`build-bundle-full.log`和`DELIVERY.json`保留早期包身份，不与最终包混用。另调用`navigation_prebuilt_install.extract_checked(archive, expected_sha, fresh_output_dir)`独立解包，核对source_commit与完整清单，取得`FIX1_SHA_MANIFEST_EXTRACT_OK 7748`；7748项payload与1份manifest全部通过。重跑时使用全新空目录。最终包SHA256为`06e596b44b5aad5b8dab79a3eafe44147c0abd6f8d94495dcf25e2cebcebb830`。

```bash
cd dist/arm64/20261118_vision_step6_fix1
sha256sum -c FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz.sha256
sha256sum -c install_fc01.sh.sha256
```

## 已实际执行的x86辅助链生命周期

实际无相机、无飞机反馈，用生产launch、当前消息typesupport及明确软件测试现场配置；不是现场标定。独立观察器收到camera disconnected、mission无控制授权/未锁定，四个辅助节点均存活到请求退出并清理完成。

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /home/jikl62/ros2_px4_ws/install/local_setup.bash
source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash
export ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1
/usr/bin/python3 /tmp/fc01-step6-service-live-test.py
'
```

证据`/tmp/fc01-step6-aux-live-run.log`、`/tmp/fc01-step6-aux-live/result.json`。最终标记`AUXILIARY_NO_CAMERA_NO_DDS_WAIT_AND_CLEANUP_PASS`。临时脚本与site测试值不得复制为真实现场标定。

## ARM64 systemd虚拟机：最终fix1包

环境：QEMU system-aarch64，Cortex-A72/2核/2GB，Ubuntu22.04.5 ARM64，systemd249，Humble，5.15通用内核；无真实飞控、相机、UART。测试Agent/PTY没有真实XRCE会话。root仅通过127.0.0.1转发SSH操作guest，不操作宿主或真实设备服务，密钥不入包或证据。

早期不同前缀搬迁导入证据`vm-first-relocation.log`输出`ARM64_RELOCATED_IMPORTS_OK`，只按早期包记录。最终fix1包在实际安装前后都执行本包完整类型、模块、SHA、全部ELF及ldd预检，已成功通过；没有借用旧包成功标记代替最终包验收。

guest内以普通用户安装，并在成功后使用只读辅助脚本观察实际进程和运行状态：

```bash
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh /home/orangepi/Desktop/FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-after-install.json
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-no-feedback --output /root/step6-evidence/fix1-no-feedback.json
```

`vm-install-fix1.log`实际输出`FC01_PREBUILT_INSTALL_OK`。旧Supervisor21502/native22053正常退出后，新Supervisor25096/native28390启动。旧VM目标1.23m为继承专用输入，`vm-runtime-inheritance.log`证实保留目标/RC/DDS/推力；不是本包默认目标被改成1.23m。

重复完全相同安装命令，再比较前后快照：

```bash
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-repeat --before /root/step6-evidence/fix1-before-repeat.json --after /root/step6-evidence/fix1-after-repeat.json
```

`vm-repeat-fix1.log`再次输出INSTALL_OK和`repeat_keeps_supervisor_and_controller=true`，PID25096/28390未变。无飞机反馈的六类回调均0，native IDLE且无运动授权；`vm-no-feedback-ros.log`及`no-feedback-ros-final.json`记录30秒真实ROS只读观察中setpoint/command/heartbeat均0，未创建测试发布者，没有/fmu/out来源。它不证明PX4实际接收或安全飞行。

手动回退使用这次真实备份路径（只属于VM，不作为现场用户的固定路径）：

```bash
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh --rollback /home/orangepi/FC01_navigation/prebuilt-backups/20261118T035408_258282Z-22282
```

`vm-manual-rollback-fix1.log`输出`FC01_PREBUILT_ROLLBACK_OK`，旧版active/唯一native，前后runtime字节一致。进程监视器实际1秒采样，只作辅助唯一实例证据；停旧在先的严格事务检查来自安装器实际PID及start_ticks验证。

### 最终11种消息的独立生成物/CDR检查

原脚本`tests/ros_generated_interface_check.py`第一次运行因目标最小Humble没有测试辅助模块`rosidl_runtime_py`失败，保留`vm-final-11-messages.log`。生产程序没有使用此辅助模块，未向guest补装它。测试专用副本`generated-check-direct.py`仅将`get_message`实现替换为`getattr(importlib.import_module(package + '.msg'), name)`；保留全部生成物来源、字段、默认值、typesupport、实际序列化与反序列化检查。明确设置SOURCE_PACKAGE为最终entry内native源码，prefix为最终entry/install；不初始化ROS，不创建节点或发布者。

```bash
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/orangepi/FC01_versions/20261118_vision_step6_fix1/install/local_setup.bash
python3 -B - <<'PYMSG'
import importlib.util
from pathlib import Path
script=Path('/root/step6-evidence/generated-check-direct.py')
spec=importlib.util.spec_from_file_location('check',script)
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
entry=Path('/home/orangepi/FC01_versions/20261118_vision_step6_fix1')
module.SOURCE_PACKAGE=entry/'native/src/jikl_fc01_system'
raise SystemExit(module.main(['--prefix',str(entry/'install'),'--report','/root/step6-evidence/fix1-unified-11-types.json']))
PYMSG
```

`vm-final-11-messages-direct.log`输出`passed=true, messages=11`，详细报告`fix1-unified-11-types.json`。保留原脚本失败和直接生成类检查通过两个结果，不写成原脚本原样通过。

### 最终包模式和inactive生命周期

完整实际脚本保留为`evidence/vm-mode-suite.sh`，在guest普通用户执行最终包与配置工具；脚本不创建ROS测试发布者。启动活性复验为`vm-mode-accept.py`，调用真实Backend/verify_active，检查当前4辅助/1原生、配置及IPC推进，另断言无飞行准入。仅VM旧参数显式建立1.5m测试基线，记录`VM_ONLY_1_5_BASELINE.json`，不修改包或现场配置。

已完成：inactive安装、pending待机、VM-only synthetic来源track待机、track重复安装。日志`vm-mode-lifecycle-fix1.log`中分别包含`FINAL_FIX1_INACTIVE_BASELINE_INSTALL_PASS`、`FINAL_FIX1_ARM64_PENDING_TRACK_PASS`、`FINAL_FIX1_ARM64_CONFIRMED_FIXTURE_TRACK_PASS`、`FINAL_FIX1_ARM64_TRACK_REPEAT_PASS`。track重复安装仍为PID52149/55665，原回退点不变。pending五类标定缺失、mission false；合成来源track mission true但无图像/飞机，目标锁定与原生授权均false。

track再用subscriber-only observer观察30秒，`track-no-feedback-ros-final.json`及实际日志显示setpoint/command/heartbeat均0，无真实PX4接收声明。

切回altitude完整验收300.8秒通过，原视觉4辅助进程均退出；随后手动回退恢复旧目录、inactive/MainPID0、零native以及逐字一致runtime。日志结尾为`FINAL_FIX1_ARM64_RETURN_ALTITUDE_PASS`、`FINAL_FIX1_TRACK_AUXILIARIES_EXITED_ALTITUDE_PASS`、`FINAL_FIX1_INACTIVE_ROLLBACK_RUNTIME_BYTE_IDENTICAL_PASS`及`FINAL_FIX1_ARM64_MODE_LIFECYCLE_SUITE_PASS`，整个脚本退出0。`fix1-transition-assertions.json`记录17次1秒采样转换、无控制器重叠。此前x86真实辅助链结果不代替ARM64事务。

### 软件启动失败恢复

`vm-failure-suite.sh`在事务开始前增加VM-only ExecStartPre条件退出23；最终包验收600秒超时后返回1并自动恢复原活动服务、唯一旧native和逐字一致runtime，`vm-failure-recovery-fix1.log`包含`FINAL_FIX1_START_FAILURE_AUTOMATIC_RECOVERY_PASS`。继承单元的auto-restart不会绕过有限预算。注入drop-in已删除。

现场命令见[最终操作说明](DELIVERY_OPERATIONS_zh.md)。安装必须输出`FC01_PREBUILT_INSTALL_OK`，回退必须输出`FC01_PREBUILT_ROLLBACK_OK`，缺反馈仍须阻止READY/导航授权和非零推力。

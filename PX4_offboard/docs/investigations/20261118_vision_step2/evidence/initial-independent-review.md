# 第2步独立只读复核

审查对象为2026-11-18当前未提交的step2改动，基线HEAD为9e9fb3ef4c4e71d8b1b646bfe0d125a0e71c59a8。未编辑生产源码、未使用共享build/cache、未启动飞控节点、未访问串口/硬件/服务。

## 已复现问题 R1：目标撤销后保留旧终端阶段需求

风险：后续若启用mission控制，P1；当前发布默认control_enabled=false、原生接口仍只读，因此本次默认配置无实际飞行权限。

位置：`mission_node.py::_clear_target`（审查时389）、`_on_platform_state`（402）、`_update_mission` terminal分支（878）。

`_clear_target`撤销vision_rel、est_pos、target锁定及前馈估计，但没有撤销terminal_start/terminal_cmd或strike/push/retreat缓存。新的平台失效/坐标重置路径调用_clear_target后，只在平台尚无效时发布valid=false；平台恢复后可重新走旧阶段并输出缓存需求。终端分支在没有当前目标时直接沿用terminal_cmd。

独立复现：`/usr/bin/python3 -B /tmp/fc01-step2-independent-repro.py`。脚本以AST从实际生产源码取出原样的_clear_target与_update_mission，不import ROS，不重新实现被测方法。场景为impact任务terminal_start=10.0、terminal_cmd=[1.2,-0.3,0.1]，调用_clear_target('PLATFORM_ORIGIN_CHANGED')后下一tick(10.2)返回同一非零速度、valid=true，虽然vision_rel=null且target_locked=false。结果：`/tmp/fc01-step2-independent-repro.json`。

建议：仅在本轮任务层对来源/坐标/平台失效锁存任务失效，清除所有旧阶段需求；恢复数据不自动恢复旧任务。保留传感器重锁定用于观察，重新飞行任务须显式重新准备。不要修改原生RC/推力/定高状态机。

## 代码确认问题 R2：相机标定没有畸变语义

风险：P2；默认标定未确认，当前默认拒绝目标测量，尚不赋予实际飞行权限。

`types.hpp::CameraIntrinsics::distortion`（42）恒返回全零；`VisionImageNode::onCameraInfo`（149）只读取K，不处理或拒绝CameraInfo.d和distortion_model，也不将标定宽高变化计入重新确认。若K与已确认参数一致，非零D不会撤销calibration_valid，raw图像可能被错误当作无畸变图像。

这是现有模型限制在本次实机接口中仍未明确的边界。本复核未将静态代码证据冒充真实相机性能复现。建议至少显式要求已矫正图像或拒绝不支持畸变并上报状态；另一选择是支持可查的完整标定并做undistort/PnP一致验证。

## 独立通过的检查

- `/usr/bin/python3 -B -m unittest discover -s tests -p test_px4_platform_feedback.py -v`：16/16通过。
- `/usr/bin/python3 -B -m unittest discover -s tests -p test_mission_sensor_contract.py -v`：11/11通过。
- `/usr/bin/python3 -B /tmp/fc01-step2-independent-coordinates.py`：固定seed7289生成1000随机姿态，使用独立Rodrigues形式矩阵验证NED/FRD→ENU/FLU四元数转换，最大矩阵误差8.881784197001252e-16，通过。结果`/tmp/fc01-step2-independent-coordinates.json`。
- 源与monotonic双TTL、invalid同源戳无法续鲜、publisher会话退休、回退锁存、NaN/非单位四元数拒绝、fieldwise速度/角速度过期、插值到图像源时刻等纯模块逻辑，未复现额外问题。
- 检查本机Humble rclpy/executors.py:398-409，标准回调只收到take_message结果的msg_info[0]，当前桥关于缺逐包MessageInfo的说明符合本机实现。图中唯一GID是独占证据，不能视为逐包来源认证。
- 固定PX4 v1.16源码Tools/msg/templates/ucdr/msg.h.em:138-144同时对timestamp和timestamp_sample加time_offset，符合桥将其当同一Agent同步时域的前提；实际板端仍须取证确认同步。

## 真实残余边界

- USB源seq是每次cap.read成功的应用序号；stamp是读取完成系统时间，不是V4L2硬件曝光戳。已通过stamp_kind和exposure_timestamp_measured=false公开。未知曝光/缓冲延迟不能宣称已测；现场必须确认latency bound后才可将calibration_valid置true。
- OpenCV cap.read可能在驱动阻塞；CameraSource::stop需要join，尚无硬截止保证。当前纯视觉进程与原生控制分离，不代表该资源风险已消失。
- 兼容PoseStamped用local源戳但姿态可能相差≤alignment_s；生产mission使用原子PlatformState及插值，因此不走该混合戳路径。若后续启用兼容simulation输入应另核对。
- 固定FC01固件DDS配置的vehicle_angular_velocity仍注释，require_angular_velocity=true时应明确报缺，不能以ULog内部gyro存在证明DDS已接入，也不能补零。

## 审查时源码SHA256

- mission_node.py：c9a44ba6172dbd4a7b73613ea8820a02def4fa093799b48ff826acd9668a52af
- sensor_contract.py：26ae9a5f3e501a3b87c88c4b118233c9239123051873872efdce0c0cbf39191e
- platform_feedback.py：1092206385c760186cef0e475f0bed74ad382ca714235d15f80077de2d92768d
- px4_state_bridge.py：724f358efeec06ad13fc971b30a7b9dcc9fad23d382ca2b8507e40aa58e84713

这是修复前独立审查快照。生产源码若后续调整，应对照上述身份重跑受影响场景，不能将本报告的通过数量直接归于新源码。

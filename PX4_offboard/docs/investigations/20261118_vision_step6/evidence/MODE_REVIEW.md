# Step 6 视觉模式独立只读复核

环境：x86 开发机，2026-11-18。只读生产源码，未操作 systemd、VM、串口或控制器；未编辑或提交生产文件。

## 结论

未复现 pending-calibration 获得任务控制权。configured_mode 每次从 canonical PENDING_SITE 重新派生配置并比对全部 YAML；pending 的五类相机确认均 false、来源 ID 为空、专用不存在设备路径、mission control_enabled=false。VisionAuxiliary.start 传入 control_enabled:=false，production mission 的 _update_native_mission/_publish_command/_publish_navigation 也独立检查该开关。地面配置和服务持同一版本 mode.lock，配置必须先停服且无实际原生控制器；软件检查失败回写 runtime、模式、三个视觉文件和安装收据。

## 已验证

命令：PYTHONPATH=.:tests python3 -m unittest tests.test_navigation_vision_mode -v
结果：19 tests PASS，0.845s。覆盖 pending 未确认、篡改重算摘要、pending 迁移、真实 site 替换、launch 控制开关、异常恢复和安装收据。

## P2 配置一致性缺口（安全拒绝，不会越权）

位置：navigation_vision_mode.py:195 与 target/src/fc01_mission/fc01_mission/mission_node.py:1170。
validate_site 接受 0 < standoff_m <= 20，但 runtime policy 还要求 standoff_m <= horizontal_max_radius_m（默认 2）。因此合法 JSON 可通过 FC01_VISION_MODE_CONFIG_OK、mission control_enabled=true，但未来持续报告 STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE，永远无法跟踪。复现：以 tests.test_navigation_vision_mode.site() 为基础，仅设 mission.standoff_m=3.0；validate_site 返回 standoff=3.0、radius=2.0、control_enabled=true。建议软件预检增加 standoff_m <= horizontal_max_radius_m 的关系校验和回归。此处运行期拒绝合理，不能放宽运行保护。

## 非阻断观察与限制

- validate_site 的 camera 对象允许所有已知 camera 参数，部分启动约束（例如身份 ID <=128、有限正 stale_timeout_s、延迟小于 stale_timeout_s）只由真实 C++ vision 节点判定。模式工具的 check 只检查原生控制配置，不实例化相机；故 FC01_VISION_MODE_CONFIG_OK 不保证任意用户 site 的辅助软件启动成功。当前交付待机 canonical/site fixture 与已验证路径不受影响；可在将来把相机参数关系集中成可离线调用的软件验证器，避免复制大量 C++规则。
- canonical pending 会显式开启原生水平诊断路线；它不是普通定高模式。若用户要验证原无视觉定高，应选 altitude。pending 下 mission 不产生有效水平意图，但原生仍保留自身反馈保护和初始定点逻辑。
- 实机图像、目标锁定、现场标定真实性、PX4 接收及飞行性能不在本次只读审查证据范围内。

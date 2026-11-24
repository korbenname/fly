# Step 6 最终安装代码独立复核（只读）

日期：2026-11-18；复核时 HEAD=05aff4af6869451f01fd9a5b6d3db36119dd252c，含待提交 Step 6 工作区差异。未编辑生产文件，未操作 systemd、VM、真实设备，未提交。

## 结论

未发现本次差异中需要阻止维护安装的 P0/P1 缺陷。确认一项 P2 配置前检不一致，已交主代理处理。最终交付仍须以同一最终包的真实 ARM64 VM 事务结果为准；本审查不是安装实测。

## 检查路径

1. navigation_prebuilt_install.install / Backend.dependencies / config_check / verify_active / software_ipc_stamp：安装不读取 require_ground/live_state，不依赖六路飞机反馈、READY、Offboard 或图像。IPC 验收绑定 boot、监督器PID、实际原生PID和单调源戳，要求本轮验收后推进；10秒是软件活性观察寿命，没有修改运行期飞行反馈TTL。
2. stop_and_wait / owned_processes / same_process_alive：按服务祖先或 unit cgroup 捕获实际 FC01 角色，记录PID+start_ticks，避免PID复用。除旧原生外等待mission、bridge、相机及只读接口退出；复用Agent和服务外程序不在停止范围。120秒未退出时保留原unit，不启动新版，不强杀。
3. install失败恢复 / restore / rollback：先验备份摘要、保留外部编辑；旧进程实际退出后才写专用drop-in；失败后先等候选退出再恢复旧unit配置及runtime，不用全机pkill。重复安装保留原回退点，合法地面配置更新安装收据。
4. navigation_prebuilt_service.check：逐路径清单和架构保留；ldd仅同一次调用、相同目录和相同已验SHA复用成功，跨目录不复用，失败不缓存。软件预检及启动预算与飞行控制TTL分离。
5. VisionAuxiliary / configured_mode：pending canonical配置会重新派生并精确比对；mission control_enabled=false通过实际launch参数生效。相机/目标诊断缺失不让软件待机失败，真实执行仍受原生门控。
6. navigation_status / flight_readiness：软件安装成功与飞机反馈、目标锁定分别呈现；旧身份、旧boot、过期子观测和多个发布者不能续鲜。显示不产生运动授权。
7. git diff 5c4dc14ddd84371d60387e7ff5227c94b7b6ee47 -- native target 为空。原生与target业务控制输入未被本轮发行修改。

## 已执行验证

命令：
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests python3 -m unittest tests.test_prebuilt_maintenance_lifecycle tests.test_navigation_prebuilt_install tests.test_navigation_prebuilt_service tests.test_navigation_vision_mode tests.test_prebuilt_vision_status -q
结果：106 tests PASS，2.339秒。日志：/tmp/fc01-step6-independent-review-tests.log。AST四文件及git diff --check PASS。

子复核额外运行19项模式测试（属于106项子集，不能相加）；记录 /tmp/fc01-step6-mode-review.md。

## 确认 P2：安全间距与水平包络关系未提前检查

navigation_vision_mode.validate_site 允许 standoff_m=3.0、horizontal_max_radius_m=2.0，返回control_enabled=true；mission_node.py:1170随后持续安全拒绝STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE。运行拒绝合理，不得放宽。建议配置前检增加standoff<=horizontal_max_radius_m并加回归。
复现：在 tests.test_navigation_vision_mode.site() 上仅把mission.standoff_m改为3.0，调用validate_site；实际输出accepted_standoff=3.0,radius=2.0,control=True。

## 限制与注意

- dependencies独立调用当时仍捕获stdout且未传总timeout；主代理正在补有界软件检查与进度。最终复核应确认该修复。
- verify_tree严格拒绝清单外pyc，现有回归通过只读入口禁写字节码避免污染；不能描述为任意污染pyc受容忍。
- 静态检查不能覆盖运行时OpenCV插件加载和真实相机驱动。需要最终VM搬迁/启动验证，真实Zero3W相机、PX4消息和飞行仍待现场。
- 回退的verify_original校验当前软件身份和一条本轮IPC，但不观察完整1秒稳定；当前合同可接受，不是飞行READY。

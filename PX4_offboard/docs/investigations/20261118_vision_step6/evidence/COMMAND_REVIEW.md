# 第6步现场命令独立复核

复核文件：docs/investigations/20261118_vision_step6/DELIVERY_OPERATIONS_zh.md。

- 7个bash代码块均bash -n通过，三个bash -c的内层脚本另行bash -n通过。
- 回退命令的python -c AST通过；INSTALL_STATE.json路径与backup字段正确，命令仅在成功取值后执行rollback。
- 安装、--status、--rollback以及altitude/track/--pending-calibration/--site/--entry/--maintenance均与当前CLI参数一致。
- pending明确mission_control_enabled=false；真实site与VM synthetic严格区分，未将配置成功等同运动许可。
- 定高和track的当前高度/槽位条件都由配置工具统一检查。已仅在此文档澄清：不是1.5m/槽1时，定高与视觉配置均拒绝，保持现场参数。
- 现场相机需要真实V4L2图像；standoff<=半径<=2m限制、无反馈待机和安装/飞行就绪分离已明确。
- 模式命令start后立即查询可能仍在初始化，说明已有提醒；需要完整软件验收可重复安装，仍保留回退点。

未执行服务命令，未操作VM或真实硬件，未修改源码/安装包，未提交。只修改了上述一句说明。

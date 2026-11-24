# Step6 软件检查耗时修复交付

提交：05aff4af6869451f01fd9a5b6d3db36119dd252c
范围：navigation_prebuilt_install.py、navigation_prebuilt_service.py、navigation_vision_mode.py及两个对应测试文件；未改native/target/控制时限。

- 同一次check内，仅相同parent.resolve()及已验证SHA256的ELF普通别名复用成功ldd结果。逐文件清单、架构、typesupport和config_check保留。没有跨调用缓存；失败不缓存。
- installer内层脚本逐个ELF实际计算SHA并比对ARM64_RELEASE.json，逐路径验AArch64后才允许复用ldd。
- STARTUP_ACCEPTANCE_SECONDS 180→600，只影响安装/回退的软件启动验收，vision-mode软件检查300→600。控制反馈TTL和所有原生控制行为未改。
- 回归增加相同目录重复一次/不同目录两次、损坏hash/架构拒绝、失败后重新检查、实际嵌入脚本执行。现有延迟测试改成240秒后软件出现仍成功，600秒始终无控制器则失败。

执行：
PYTHONPATH=.:tests python3 -m unittest tests.test_arm64_bundle tests.test_arm64_runtime tests.test_navigation_prebuilt_install tests.test_navigation_prebuilt_service tests.test_navigation_vision_mode tests.test_prebuilt_vision_status tests.test_navigation_feedback_freshness tests.test_stage3_status_readiness tests.test_navigation_service tests.test_overnight_stage5_ros_contract -v
结果：158 tests PASS，/tmp/fc01-step6-wait-fix-full.log

PYTHONPATH=.:tests python3 -m unittest tests.test_release_payload_policy tests.test_overnight_stage5_ros_contract -v
结果：11 tests PASS，/tmp/fc01-step6-wait-fix-release.log（含3个重复contract测试，独立总数166）。
AST 5 files及git diff --check PASS。

首次整组缺少PYTHONPATH=.:tests导致test_navigation_service导入test_native_profile失败，已使用仓库文档入口重跑通过；不是代码缺陷。
首次git commit缺少作者信息，按最近本地历史作者使用一次性git -c identity完成提交，未改global/local config。

后续必须从该源码提交重新构建tar/installer、重新生成SHA，不能继续交付旧6a9d0b3包。实际VM完整路径由root负责，未在子任务修改任何VM文件/服务。

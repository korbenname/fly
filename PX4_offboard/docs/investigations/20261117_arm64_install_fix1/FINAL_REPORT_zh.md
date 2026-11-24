# FC01 ARM64 安装缓存缺陷修复

2026-11-17。本轮只处理用户实际安装失败及必要回归，未推送、未部署真实香橙派、未改变飞行算法/配置/权限。

## 已确认根因与现场状态

P2 发行安装阻断：原20261116包在严格解包校验时完整；用户直接执行包内navigation.py status（甚至--help）会导入navigation_status，CPython默认写入 __pycache__/navigation_status.cpython-310.pyc。再次安装时发现多出该文件而拒绝。原始实际tar在开发机独立临时目录精确复现同一路径/错误；不是推测串口/ROS/二进制损坏。

用户输出显示旧20261112_release服务active、无drop-in、唯一控制器仍在旧目录。新版状态脚本读取共享status.json，不能证明新版运行。STALE是独立的反馈新鲜度提示：现有原生0.5秒等各反馈TTL还会被显示快照留存时间消耗；不能凭“最近0.9秒”就认定当前反馈有效。此次没有现场源timing与同期PX4日志，不能进一步断言STALE来自DDS失联或控制器停转。缓存修复不取消任何TTL、地面或RC检查。

## 最小修复与发布身份

代码修复提交：4c85a9e7e0c66193104164f03f4bfe28de709b9d。

- navigation.py：在导入项目模块前设置sys.dont_write_bytecode=True，使直接管理查询不写新缓存。状态判断/命令分派不改。不会清理或信任已有缓存。
- navigation_prebuilt_install.py：保持精确集合/文件摘要/路径/ELF验证，仅增加前30个缺失/额外路径及总数量的错误提示。
- tests/test_navigation_prebuilt_install.py：增加真实Python入口/查询后重装与额外缓存仍被拒绝的两项回归。

发行到全新独立版本20261117_stage3_fix1，保留旧失败目录而非忽略/删除未验证文件。包内只有上述两个Python载荷字节改变；全部26个ARM ELF、4874安装产物、控制源码、配置和消息接口逐字节不变。ARM64 GCC11/Humble编译证据继续绑定同一Stage3控制候选adc55d0b7c67e80bf9d77b1082208c898f95b756，不冒称重新编译或新增实飞验证。

文件：dist/arm64_fix1/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz，6498104字节，SHA256：

`5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373`

相配install_fc01.sh SHA256：

`c52f285e32241cf3a12a8daf67e60cc4ae8ff61a2afe91eb94648dd2626eecff`

## 真实验证

宿主79项相关测试通过（12.470秒）：旧服务26、预编译服务6、安装器37、打包器2、反馈新鲜度8。原始日志见evidence/python-tests.log。新增入口回归在没有PYTHONDONTWRITEBYTECODE/PYTHONPYCACHEPREFIX/PYTHONPATH的真实子解释器执行；只替换只读状态来源，不接触宿主服务。查询后无缓存、严格清单通过、重装不重启/不覆盖备份。人为加额外.pyc仍停旧前拒绝并保留其字节。

初次测试夹具把navigation.py作为import模块导入，解释器在模块开始执行前已为navigation本身写缓存，导致1项失败；实际用户直接脚本路径不缓存主脚本。已将测试改为runpy脚本语义，并保留原始失败日志initial-test-harness-failure.log，没有删除或放宽断言。

实际新tar严格独立解包4974文件通过；对原tar直接--help后再校验复现报错，对新tar相同操作后无新增文件，精确校验通过。独立审查还核对嵌入安装器与提交源码完全一致（只增加固定摘要pin）、Shell/Python语法、新旧包逐文件差异。见evidence/ACTUAL_ARCHIVE_REPRODUCTION.json、INDEPENDENT_REVIEW.md、ROOT_CAUSE_REVIEW.md、DELIVERY.json。

[现场安装、状态及回退命令](DEPLOY_zh.md)。原有未知未跟踪文件保留，不纳入提交或运行包。任何配置/状态前置不符仍会拒绝，不通过此修复自动改变现场配置。

## 完整ARM64虚拟机的实际结果边界

最终修正版包已在QEMU-system-aarch64完整Ubuntu22.04/Humble、真实systemd PID1安装成功。仍是合成未解锁/接地/静止DDS与持PTY的Agent替身，不是OrangePi/UART/XRCE/PX4硬件验证。首轮安装因快照新鲜度拒绝，旧服务保留；有界第二次正常命令通过所有原有门槛并成功。没有放宽0.25秒快照窗口或修改TTL。

普通Python直接status退出0，随后无缓存且精确文件校验通过；显示本次为STALE（历史快照年龄1.2秒），不能将状态命令成功冒称当前反馈READY。安装器切换的实时地面验收是另一份有来源/时效的证据。原始失败、重试和实际查询输出保留于evidence/vm/。

本次修正版实际重复安装返回already_active=true，控制器PID2370、备份路径和runtime摘要保持不变。旧20261116目录内原缓存仍保持原摘要，没有删除或忽略缓存来达成成功。

本次手动回退三次有界尝试均被原有快照新鲜度门槛拒绝，未执行停服/改写；不能声称本次fix1包手动回退已通过。此前20261116完整VM回退成功证据仍有效但不替代本次结果。修复代码只改变缓存写入与错误文本，未修改回退逻辑或其地面保护。此环境的回退验收仍未完成，现场若同样拒绝，应保留服务、采集最新状态/日志后处理，不能强制覆盖或取消TTL。具体最终状态见evidence/vm/下结果。

[完整ARM64 VM结果JSON](evidence/vm/RESULT.json)记录所有通过/拒绝项。测试完成后仅在隔离VM地面夹具中正常停止候选服务再关闭VM，不将测试清理冒称回退成功。没有访问或修改真实香橙派。

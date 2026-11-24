# 第2步修复后独立复核（最终限定复验）

仅只读生产源并运行测试；没有构建、修改生产代码、访问相机/PX4/串口/systemd。没有占用root统一构建目录进行写入。只向/tmp写入本次独立证据。

## 结论

本次复验中，R1、R2、R3均通过所列限定检查，未复现新的阻断问题。此结论不是实机飞行安全批准，也不表示后续任务准入和水平仲裁已完成。

### R3：畸形ready快照同样本复活已关闭

`PlatformHistory.accept`先保留旧水位，再消耗整条快照全部测量水位，校验过程中只暂存值，所有载荷检查完成才写历史。任一载荷异常会清空历史而保留已消费水位。对原R3复现脚本原样重跑，现输出`same_sample_revalidated=false`；旧复现JSON未覆盖。

命令：`/usr/bin/python3 -B /tmp/fc01-step2-final-review-extra-repro.py > /tmp/fc01-step2-postfix-review-r3.json`。

额外验证非法四元数位于字段遍历后段时，前面正确位置/速度也不会部分入队；只推进local字段不足以恢复，其余必要字段也须推进；所有字段推进后恢复正常。脚本`/tmp/fc01-step2-postfix-review-transaction.py`，结果`/tmp/fc01-step2-postfix-review-transaction.json`，三项布尔检查均true。

### R1：旧冻结阶段需求仍被撤销

真实方法回归`test_mission_cached_demand`与当前传感器模块一起运行；terminal/strike/push/retreat失效后清缓存并锁存，重新有反馈不会自动恢复旧需求。之前最终发布方法边界检查仍适用，mission_node本次未改变。生产显式重新准备入口仍属于后续步骤。

实际命令：
```
/usr/bin/python3 -B -m unittest tests.test_mission_cached_demand tests.test_mission_sensor_contract tests.test_px4_platform_feedback -v
```
实际30/30通过，日志`/tmp/fc01-step2-postfix-review-tests.log`。不把其与先前29项相加。

### R2：零畸变模型边界已明确并经过真实ROS复验

核对`image_rectified_confirmed`默认false且参与统一calibration_valid；Image的CameraInfo分支验证完整有限K、匹配尺寸、支持的零畸变D/model。非法/变化CameraInfo锁存失效，不会被随后匹配的旧CameraInfo恢复。USB入口同样要求零畸变适用性确认，但没有声称程序实现了undistort。

独立实际运行当前统一安装树的C++图像节点：Ubuntu22.04 x86_64、ROS2 Humble、ROS_DOMAIN_ID=181、localhost-only、合成Image/CameraInfo，无飞控节点。命令为source Humble、现有SDK px4_msgs、统一install后：
```
/usr/bin/python3 tests/ros_camera_calibration_smoke.py --binary /home/jikl62/.cache/fc01-vision-step2-20261118/install/fc01_vision/lib/fc01_vision/vision_image_node --output /tmp/fc01-step2-postfix-review-camera
```
实际8项有名检查通过，结果`/tmp/fc01-step2-postfix-review-camera/RESULT.json`和`/tmp/fc01-step2-postfix-review-camera.log`。这8项不是额外8个unittest，不相加制造总数。

## 剩余边界

- 未验证真实镜头是否满足零畸变模型；现场需要预矫正来源及对应标定，或可查的模型适用证据。确认开关不是自动校准。
- USB `cap.read`完成时刻不是硬件曝光时刻；实际采集/缓存延迟仍需现场核对。驱动read阻塞、stop/join没有已证明的硬截止。
- PlatformState中的monotonic接收戳只适合同一香橙派主机。跨主机发布不能直接比较；生产设计当前为同机桥与mission。
- Humble图中唯一GID是来源独占图证据，不是逐包认证；现场DDS与同步时域、角速度导出仍缺实测。
- 上游任务2.78m与冲撞逻辑仍默认关闭；原生1.5m起飞/到高/RC准入及水平仲裁尚未接通，不因传感器链通过而获得飞行权限。

## 本次实际测试源码SHA256

- mission_node.py: e14ada291933fdc17c5e125dd056fa84de0629f076934625bce28e2071f0e1f1
- sensor_contract.py: 6a08ad3c5b85c655adbc01bd8c83bc4e23fd53209dcf8cd7d6ae9fb15bb1a639
- tests/test_mission_sensor_contract.py: 5032d31d5ae7cf884bc5af9c6da9836fd24551b96704ddd9b1f4347a7415c1ac
- vision_image_node.cpp: 4ddc22dca6e6d7146a232bed94e24aab0b432318251a2c65dee731bd35abd155
- vision_state_adapter.cpp: 185f7e1e0705e8892669e66e8bb5e0a9264dc896ee703634cc0ce496a2e8ebba
- tests/ros_camera_calibration_smoke.py: f85a6dda99527a998d5986489055546e6b57c0474dd49f9b8fb7ef1465a7151f

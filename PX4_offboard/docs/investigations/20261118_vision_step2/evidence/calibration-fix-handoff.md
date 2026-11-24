# R2相机畸变边界修复交接

已完成并停止编辑。仅编辑target/src/fc01_vision/**与新增tests/ros_camera_calibration_smoke.py，未提交，未使用root统一build cache，未触及原生控制、PX4或串口。

## 真实缺陷复现

修前实际C++ image node +真实Humble合成CameraInfo/Image：K保持一致、D从全零改为[0.1,0,0,0,0]，TargetObservation仍tracking=true/calibration_valid=true。证据/tmp/fc01-step2-calibration-before/{RESULT.json,nonzero-distortion.log}，stdout /tmp/fc01-step2-calibration-before.log。新永久测试支持--expect-unfixed，只有显式复现时才用它，不作为通过验收。

## 最小修复

估计器现有零畸变模型保持不变；新增image_rectified_confirmed=false，由两入口统一AdapterConfig声明，calibration_valid必须包含该确认。状态missing显示CAMERA_RECTIFICATION。注释明确需要已矫正输入+对应K/尺寸，或现场校准证明零畸变模型适用，不声称自动undistort。

Image节点use_camera_info=true时：先收到与配置K/尺寸完全一致的CameraInfo才能处理有效图像；完整K须有限且符合针孔布局，D必须有限全零且与支持的plumb_bob/rational_polynomial/empty模型布局相符。非零/NaN D、不支持模型、非法K或K/分辨率改变立即撤销并锁存标定失效；随后原CameraInfo不能复活，现场核对后更新配置并重启。重复匹配CameraInfo不使当前目标无效，也不续鲜图像。

Root已在两个合成夹具中加入image_rectified_confirmed true（本代理未编辑这两文件）。旧手工缺相机测试若断言exactly4 missing需改为5，因为本次新增一个实际缺项。

## 实际验证

- /tmp/fc01-step2-calibration-final-build.log：独立camera cache四包最新构建PASS。
- /tmp/fc01-step2-calibration-final-ctest.log：2/2 CTest PASS；vision_contract新增rectification未确认与其他缺项不被隐藏的断言。
- /tmp/fc01-step2-calibration-after/RESULT.json与stdout /tmp/fc01-step2-calibration-after.log：实际Humble domain179 localhostonly，8个有名检查PASS：非零D撤销、匹配旧CameraInfo不能复活、NaN K、NaN D、分辨率改变、主点改变、不支持畸变模型、零畸变模型必须显式确认。
- Python compile及限定git diff --check PASS。

复验命令（source Humble+SDK px4_msgs+正确统一install后）：
```
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_camera_calibration_smoke.py --binary <install>/fc01_vision/lib/fc01_vision/vision_image_node --output /tmp/fc01-step2-calibration-final-unified
```

说明：未连接真实相机，没有给用户填标定值，不证明实际镜头已被矫正。完整raw镜头畸变校正仍不支持，现场需提供预矫正来源或模型适用性证据。本轮不强行引入OpenCV undistort链。

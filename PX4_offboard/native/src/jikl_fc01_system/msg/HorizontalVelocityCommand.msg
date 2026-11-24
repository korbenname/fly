# 生产水平制导封套V1；旧VelocityCommandV2的字段布局和诊断端点保持不变。
# command.stamp是指令生成ROS时刻，attitude_sample_stamp_ns是编码FRD向量所用的PX4姿态采样时刻。
# 原生端必须找到相同采样戳、同一估计参考的真实姿态，完整FRD->NED旋转后仅使用XY。
# origin_id必须匹配PlatformState的PX4 local原点身份；切换/重置撤销旧意图。
# 本封套中command.wz解释为期望NED航向变化率(rad/s，下轴顺时针为正)，不是实测机体角速度。
# command.wz=0保持已提交航向；command.vz仅保留三维编码投影信息，无垂向目标权限。
# 纯body-Z请求(vx=vy=0)明确视为无水平需求；其余请求保留完整三维投影，再丢弃NED Z。
# command.valid=false撤销跟踪需求；是否正常切换或故障由单一原生仲裁器的阶段/通道证据决定。
uint16 schema_version 1
jikl_fc01_system/VelocityCommandV2 command
uint64 attitude_sample_stamp_ns
string origin_id

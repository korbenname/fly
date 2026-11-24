# 相机看到的目标相对观测 V1，与 VisionState 的自机世界状态语义严格隔离。
# position：相对相机测量原点的目标坐标，m；velocity：该相对量的变化率，m/s。
# body_frd 表示分量方向前/右/下，不能据名称假定测量原点已经从相机移到机体。
# 相机安装旋转/杆臂及时间对齐在后续状态桥处理；本消息不替代 PX4 自机 z/vz/姿态。
# ATTITUDE/QUALITY 等未观测字段不得因默认零/单位四元数而宣称有效；有效位单独声明。
# stamp 与 sequence 保留原始测量身份；目前 target 上游新鲜度缺口留待第2步修复。
# 本接口不授权 ARM、Offboard、起飞或动力发布。仅变更消息类型不会完成视觉飞行接入。
uint16 schema_version 1
uint16 POSITION=1
uint16 VELOCITY=2
uint16 ATTITUDE=4
uint16 ANGULAR_VELOCITY=8
uint16 POSE_COVARIANCE=16
uint16 TWIST_COVARIANCE=32
uint16 QUALITY=64
uint64 sequence
builtin_interfaces/Time stamp
uint32 reset_counter
string frame_id
string body_frame_id
string source_id
string angular_velocity_source
bool initialized
bool tracking
bool calibration_valid
uint16 valid_fields
float64[3] position
float64[3] velocity
float64[4] attitude
float64[3] angular_velocity
float64[36] pose_covariance
float64[36] twist_covariance
float64 quality

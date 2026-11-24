# 原生发布边界后的只读原子快照；保留旧 NativeFlightState 类型和话题不变。
# state、原始姿态和原点由同一控制线程、同一次 publication_context 一起复制。
# 任务不能从独立订阅者的“最新姿态”猜测 state 引用的样本：两个 DDS 接收端可能丢失不同帧。
# attitude_wxyz 是 PX4 原始 Hamilton w,x,y,z，机体 FRD 旋转到世界 NED；不归一化、不插值。
# 它严格对应 state.attitude_sample_stamp_us 和 state.quat_reset_counter。
# origin_id 与 PlatformState 相同，含 PX4 局部参考及位置/速度/航向重置计数；不能跨原点续用目标。
# attitude_receive_monotonic_ns 是本原生进程首次收到该测量的本机单调时刻，重复快照不刷新它。
# 接收端仍须校验 state 的本次进程/任务身份、反馈/发布门、源时刻和接收时刻；此消息不授予控制权。
uint16 schema_version 1
NativeFlightState state
float64[4] attitude_wxyz
string origin_id
uint64 attitude_receive_monotonic_ns

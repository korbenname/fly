# 正常合成链短日志

时间相对人工RC解锁边沿；PX4确认均为本机合成peer发布的实际状态消息，不是真实固件。完整原始数据见ROS_FINAL.tar.gz/full-chain/normal。

| 相对秒 | 接收/阶段/意图 | 真实typed接收端或原生事实 |
|---:|---|---|
| 0.000 | manual_rc_arm_edge | manual_rc_arm_edge |
| 0.401 | native_phase | RC_ARMED_WAIT_NAVIGATION / RC_WAIT_MATCHED_SWITCH_AND_NEW_ARMED_STATUS / h=0.000m |
| 0.402 | synthetic_px4_manual_arm_confirmed | synthetic_px4_manual_arm_confirmed |
| 0.902 | rc_navigation_slot_1 | rc_navigation_slot_1 |
| 1.053 | synthetic_px4_offboard_confirmed | synthetic_px4_offboard_confirmed |
| 1.403 | native_phase | ARMED_GROUND / RC_MANUAL_ARM_AND_NEW_OFFBOARD_CONFIRMED / h=0.000m |
| 2.405 | native_phase | THRUST_RAMP / THRUST_RAMP / h=0.000m |
| 5.915 | native_phase | ASCEND / ASCEND / h=0.205m |
| 12.429 | native_phase | HEIGHT_HOLD / VISUAL_HORIZONTAL_WAYPOINT / h=1.596m |
| 12.432 | image_target_right | 像素x=395；随后t=12.580s收到q=[0.9995046854019165, 0.00461870850995183, -0.030197473242878914, 0.007562900427728891]，FRD推力=[0.0, 0.0, -0.5012142658233643]；原生源=HORIZONTAL_TRACK_PX4_VELOCITY_CLOSED_LOOP |
| 12.932 | image_target_left | 像素x=245；随后t=13.030s收到q=[0.9983761310577393, -0.024183673784136772, -0.018664665520191193, 0.0480823777616024]，FRD推力=[0.0, 0.0, -0.5006961226463318]；原生源=HORIZONTAL_TRACK_PX4_VELOCITY_CLOSED_LOOP |
| 15.032 | normal_land_request | normal_land_request |
| 15.537 | native_phase | DESCENT / LAND_ACCEPTED / h=1.632m |
| 25.959 | native_phase | RECOVERY_DESCENT / DESCENT_TIMEOUT_CONTROLLED_RECOVERY_ACTIVE_HANDOFF_UNVERIFIED / h=0.170m |
| 28.963 | native_phase | LANDED / LANDED / h=0.000m |
| 31.718 | actual_px4_topic_command_received | 合成DDS接收端收到command=400 param1=0.0；h=0.000m |
| 31.868 | disarm_ack | disarm_ack |
| 31.969 | native_phase | REQUEST_DISARM / REQUEST_DISARM / h=0.000m |
| 32.470 | synthetic_px4_new_disarmed_state | synthetic_px4_new_disarmed_state |
| 32.569 | actual_px4_topic_command_received | 合成DDS接收端收到command=176 param1=1.0；h=0.000m |
| 32.721 | manual_ground_release | manual_ground_release |
| 32.970 | native_phase | SAFETY / OPERATOR_TAKEOVER_LATCHED / h=0.000m |
| 36.979 | native_phase | RC_PRESTREAM / RC_PRESTREAM_WAIT_NEW_GROUND_REFERENCE / h=0.000m |
| 38.983 | native_phase | RC_READY_WAIT_ARM / RC_READY_WAIT_CH5_MANUAL_ARM / h=0.000m |
| 38.984 | service_graceful_stop_requested | service_graceful_stop_requested |

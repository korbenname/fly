"""Vision guidance / mission node.

This is the flight logic that used to live inside ``fc01_vision_sim``: the
takeoff -> seek -> strike -> push -> retreat -> hold state machine, plus the
``track`` and ``impact`` modes. It is deliberately independent of MuJoCo and
of the simulator package:

  inputs
    /fc01/vision/target_observation    相机中心相对目标 TargetObservation
    /fc01/platform/state              PX4融合自机状态，原local原点，ENU/FLU
    以下/sim端点仅platform_input_mode=simulation显式启用：
    /sim/platform_pose                platform position + attitude (world)
    /sim/platform_velocity            platform linear velocity (world)
    /sim/platform_angular_velocity    body rates (body FLU)
    /sim/target_contact_force         |contact force| with the target, N
    /sim/impact                       episode ended by contact (impact mode)
    /sim/reset_event                  simulator respawned the aircraft

  output
    /visioncontrol                    jikl_fc01_system/VelocityCommandV2,
                                      body FRD m/s, at vision_control_hz

实机任务只产生水平意图，等待原生人工RC起飞和1.5m连续到高。原生控制器独占姿态/推力发布，任务默认关闭。

The node never reads ground truth about the target: the light's map
coordinates are a configuration prior (``target_x/y/z``), used for the blind
cruise before vision locks on and for the static strike, exactly as the old
in-sim controller did.
"""

import math
import time
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Empty, Float32, String
from jikl_fc01_system.msg import PlatformState, NativeFlightSnapshot, HorizontalVelocityCommand
from .native_flight_gate import NativeFlightGate
from .platform_feedback import quaternion_ned_frd_to_enu_flu
from .sensor_contract import (MeasurementIdentity, PlatformHistory, finite,
                              fresh, rotation_xyzw, stamp_ns)

# 所有自定义消息来自唯一native生成包。构建/类型支持缺失保留ImportError原文。
from jikl_fc01_system.msg import NavigationCommand as NavigationCommandMsg
from jikl_fc01_system.msg import VelocityCommandV2 as VelocityCommandMsg
from jikl_fc01_system.msg import TargetObservation as TargetObservationMsg


# The vision image node publishes best-effort sensor data.
SENSOR_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


# 指令只保留最新一帧，不向后来加入的接收者重放历史任务；目标量和自机反馈仍分开。
COMMAND_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)


class MissionNode(Node):
    def __init__(self):
        super().__init__("fc01_mission")

        # ---- Parameters ---------------------------------------------------
        p = self.declare_parameter
        self.control_hz = max(10.0, float(p("control_hz", 100.0).value))
        self.control_enabled = bool(p("control_enabled", False).value)
        self.engage_mode = str(p("engage_mode", "track").value).strip().lower()
        if self.engage_mode not in ("track", "impact", "push"):
            raise ValueError(
                "engage_mode must be 'track', 'impact' or 'push', got '%s'"
                % self.engage_mode)

        # The RMUC green light on the map. This is a prior, not a
        # measurement: vision is the only feedback used in the loop.
        self.target_x = float(p("target_x", 0.315).value)
        self.target_y = float(p("target_y", -4.958).value)
        self.target_base_z = float(p("target_base_z", 1.373).value)
        # Yaw the airframe should hold while striking/pushing the light face
        # (the light faces -x, so the nose points +x at yaw 0).
        self.target_face_yaw = math.radians(
            float(p("target_face_yaw_deg", 0.0).value))
        self.target_motion = str(
            p("target_motion", "static").value).strip().lower()
        if self.target_motion not in ("random", "sinusoid", "static"):
            raise ValueError(
                "target_motion must be 'random', 'sinusoid' or 'static', "
                "got '%s'" % self.target_motion)
        self.target_pos = np.array(
            [self.target_x, self.target_y, self.target_base_z])

        # Camera mount offset in the body FLU frame (x forward, y left,
        # z up). Vision reports the target relative to the camera optical
        # centre, so the world estimate needs this offset. Must match the
        # simulator scene / the real airframe.
        self.camera_pos = np.array([
            float(p("camera_pos_x", 0.08).value),
            float(p("camera_pos_y", 0.0).value),
            float(p("camera_pos_z", 0.015).value),
        ])
        # 生产只接受实际PX4桥的原子快照；旧分离/sim回调需显式simulation模式。
        self.platform_input_mode = str(p('platform_input_mode', 'px4').value)
        if self.platform_input_mode not in ('px4', 'simulation'):
            raise ValueError('platform_input_mode must be px4 or simulation')
        self.platform_state_topic = str(p('platform_state_topic', '/fc01/platform/state').value)
        self.native_flight_topic = str(p('native_flight_topic','/fc01/native/flight_snapshot').value)
        self.native_flight_timeout_s = float(p('native_flight_timeout_s',.3).value)
        self.require_angular_velocity = bool(p('require_angular_velocity', True).value)
        self.sensor_timeout_s = float(p('sensor_timeout_s', .3).value)
        self.sensor_alignment_s = float(p('sensor_alignment_s', .08).value)
        self.target_lock_frames = int(p('target_lock_frames', 3).value)
        if not 2 <= self.target_lock_frames <= 30:
            raise ValueError('target_lock_frames must be in 2..30')
        self.camera_extrinsics_confirmed = bool(p('camera_extrinsics_confirmed', False).value)
        self.camera_extrinsics_id = str(p('camera_extrinsics_id', '').value)
        self.waypoint_alignment_confirmed = bool(p('waypoint_alignment_confirmed', False).value)
        self.waypoint_alignment_origin_id = str(p('waypoint_alignment_origin_id', '').value)
        # 位置模式由原生用本次未解锁锚点核对半径，桥的首帧不是该锚点。
        self.position_takeoff_anchor_authority = bool(p('position_takeoff_anchor_authority', False).value)
        # 定点不需要地图坐标；航点必须有可查的原点确认并限于本次起飞点周围。
        self.horizontal_navigation_mode = str(p('horizontal_navigation_mode','hold').value)
        if self.horizontal_navigation_mode not in ('hold','waypoint'):
            raise ValueError('horizontal_navigation_mode must be hold or waypoint')
        self.horizontal_max_radius_m = float(p('horizontal_max_radius_m',2.).value)
        self.horizontal_max_speed_mps = float(p('horizontal_max_speed_mps',.5).value)
        self.horizontal_max_yaw_rate_rps = float(p('horizontal_max_yaw_rate_rps',.35).value)
        self.standoff_calibration_confirmed = bool(p('standoff_calibration_confirmed',False).value)
        self.standoff_calibration_id = str(p('standoff_calibration_id','').value)
        if (not math.isfinite(self.horizontal_max_radius_m) or not 0 < self.horizontal_max_radius_m <= 2. or
                not math.isfinite(self.horizontal_max_speed_mps) or not 0 < self.horizontal_max_speed_mps <= .5 or
                not math.isfinite(self.horizontal_max_yaw_rate_rps) or not 0 < self.horizontal_max_yaw_rate_rps <= .35):
            raise ValueError('horizontal limits exceed native first-test envelope')
        if self.platform_input_mode == 'px4' and self.engage_mode != 'track':
            raise ValueError('production PX4 mission requires engage_mode=track; push/impact are simulation only')
        if not np.all(np.isfinite(self.camera_pos)):
            raise ValueError('camera lever arm must be finite')

        # ---- Flight profile ----------------------------------------------
        # Takeoff: vertical climb, a short hover, then the chase.
        self.takeoff_altitude_m = float(
            p("takeoff_altitude_m", 1.2).value)
        self.takeoff_climb_mps = max(
            0.1, float(p("takeoff_climb_mps", 0.8).value))
        self.takeoff_hover_s = max(
            0.0, float(p("takeoff_hover_s", 0.8).value))
        self.seek_ramp_s = max(
            0.05, float(p("seek_ramp_s", 1.2).value))
        self.cruise_descent_mps = max(
            0.1, float(p("cruise_descent_mps", 0.9).value))
        self.standoff_m = float(p("standoff_m", 2.0).value)
        self.controller_gain = float(p("controller_gain", 2.0).value)
        self.max_speed = float(p("max_speed", 2.5).value)
        self.search_speed = max(0.1, float(p("search_speed", 0.9).value))
        self.track_max_speed = float(p("track_max_speed", 2.0).value)

        # Sustained push.
        self.push_speed = float(p("push_speed", 0.35).value)
        self.push_engage_m = float(p("push_engage_m", 0.55).value)
        self.push_release_m = float(p("push_release_m", 0.95).value)
        self.push_max_force_n = float(p("push_max_force_n", 90.0).value)
        self.push_altitude_m = float(p("push_altitude_m", 0.0).value)
        self.push_altitude_offset_m = float(
            p("push_altitude_offset_m", 0.05).value)
        self.push_hold_s = float(p("push_hold_s", 3.0).value)
        self.retreat_speed = max(0.1, float(p("retreat_speed", 0.8).value))
        self.retreat_distance_m = max(
            0.1, float(p("retreat_distance_m", 1.0).value))
        self.retreat_ramp_s = max(
            0.05, float(p("retreat_ramp_s", 0.5).value))

        # Terminal strike before the push.
        self.strike_speed = float(p("strike_speed", 2.0).value)
        self.strike_trigger_m = float(p("strike_trigger_m", 2.6).value)
        self.strike_engage_m = float(p("strike_engage_m", 0.10).value)
        self.strike_timeout_s = float(p("strike_timeout_s", 3.0).value)
        self.strike_settle_s = float(p("strike_settle_s", 0.25).value)
        self.strike_settle_brake_mps = float(
            p("strike_settle_brake_mps", 0.4).value)

        # Impact mode: single terminal dive, motors cut by the plant on
        # contact.
        self.impact_speed = float(p("impact_speed", 3.0).value)
        self.terminal_trigger_m = float(
            p("terminal_trigger_m", 1.0).value)
        self.terminal_timeout_s = float(p("terminal_timeout_s", 1.0).value)

        self.min_altitude_m = float(p("min_altitude_m", 0.16).value)

        # ---- Topics -------------------------------------------------------
        self.relative_topic = p(
            "relative_topic", "/fc01/vision/target_relative").value
        self.vision_state_topic = p(
            "vision_state_topic", "/fc01/vision/target_observation").value
        self.use_vision_state = bool(p("use_vision_state", True).value)
        self.platform_pose_topic = p(
            "platform_pose_topic", "/sim/platform_pose").value
        self.platform_velocity_topic = p(
            "platform_velocity_topic", "/sim/platform_velocity").value
        self.platform_omega_topic = p(
            "platform_angular_velocity_topic",
            "/sim/platform_angular_velocity").value
        self.contact_force_topic = p(
            "contact_force_topic", "/sim/target_contact_force").value
        self.impact_topic = p("impact_topic", "/sim/impact").value
        self.reset_event_topic = p(
            "reset_event_topic", "/sim/reset_event").value
        self.state_topic = p("state_topic", "/mission/state").value
        self.state_publish_hz = max(
            0.0, float(p("state_publish_hz", 10.0).value))

        self.publish_vision_control = bool(
            p("publish_vision_control", True).value)
        self.vision_control_topic = str(
            p("vision_control_topic", "/visioncontrol").value)
        # Heading is commanded as a body yaw RATE in VelocityCommandV2.wz, not
        # as an absolute angle: the mission closes the heading loop using the
        # platform attitude it already receives, and the flight controller
        # only has to track the rate.
        self.yaw_rate_gain = max(
            0.0, float(p("yaw_rate_gain", 3.0).value))
        # Per-axis body-FRD velocity limits applied just before publishing.
        # 0 disables a limit.
        self.max_vx_mps = max(0.0, float(p("max_vx_mps", 0.0).value))
        self.max_vy_mps = max(0.0, float(p("max_vy_mps", 0.0).value))
        self.max_vz_mps = max(0.0, float(p("max_vz_mps", 0.0).value))
        self.vision_control_hz = max(
            1.0, float(p("vision_control_hz", 50.0).value))
        self.vision_control_source_id = str(
            p("vision_control_source_id", "fc01_mission").value)
        self.horizontal_command_topic = str(p('horizontal_command_topic',
            '/fc01/vision/horizontal_command').value)

        # Waypoint command for the flight controller, published on its own
        # topic while the target is not yet detected. The FC follows this
        # when it is valid/fresh and falls back to /visioncontrol otherwise.
        self.publish_navigation_command = bool(
            p("publish_navigation_command", True).value)
        self.navigation_command_topic = str(
            p("navigation_command_topic", "/navigation_command").value)
        self.navigation_command_hz = max(
            1.0, float(p("navigation_command_hz", 10.0).value))
        # Waypoint used after takeoff and before vision lock. z <= 0 means
        # "use push_altitude_m".
        self.navigation_waypoint_x = float(
            p("navigation_waypoint_x", self.target_x).value)
        self.navigation_waypoint_y = float(
            p("navigation_waypoint_y", self.target_y).value)
        self.navigation_waypoint_z = float(
            p("navigation_waypoint_z", 0.0).value)

        # Freshness limits. Vision older than this is ignored; a command is
        # held for command_timeout_s by the plant if this node stalls.
        self.command_timeout_s = float(p("command_timeout_s", 0.5).value)
        self.pose_timeout_s = float(p("pose_timeout_s", 0.5).value)
        self.contact_timeout_s = float(p("contact_timeout_s", 0.2).value)
        self.control_debug_hz = float(p("control_debug_hz", 0.0).value)

        # ---- ROS I/O ------------------------------------------------------
        self.vision_control_pub = None
        if self.publish_vision_control:
            if VelocityCommandMsg is not None:
                self.vision_control_pub = self.create_publisher(
                    VelocityCommandMsg, self.vision_control_topic, COMMAND_QOS)
            else:
                self.get_logger().error(
                    "publish_vision_control=true but VelocityCommandV2 is not "
                    "built; rebuild jikl_fc01_system")
        # 独立生产封套保留V2生成时刻，同时携带旋转实际用的姿态采样戳和PX4原点。
        # 旧/visioncontrol V2继续发布作兼容诊断；原生仅消费本封套作为水平速度输入。
        self.horizontal_command_pub = self.create_publisher(
            HorizontalVelocityCommand,self.horizontal_command_topic,COMMAND_QOS)
        self.navigation_command_pub = None
        if self.publish_navigation_command:
            if NavigationCommandMsg is not None:
                self.navigation_command_pub = self.create_publisher(
                    NavigationCommandMsg, self.navigation_command_topic, COMMAND_QOS)
            else:
                self.get_logger().error(
                    "publish_navigation_command=true but NavigationCommand "
                    "is not built; rebuild jikl_fc01_system")
        self.relative_sub = None  # 旧PointStamped缺测量身份，生产不再订阅。
        self.state_sub = None
        self.vision_state_time = 0.0
        if self.use_vision_state:
            if TargetObservationMsg is not None:
                self.state_sub = self.create_subscription(
                    TargetObservationMsg, self.vision_state_topic, self._on_state,
                    SENSOR_QOS)
                self.get_logger().info(
                    "consuming %s (takes precedence over %s)"
                    % (self.vision_state_topic, self.relative_topic))
            else:
                self.get_logger().warn(
                    "use_vision_state=true but jikl_fc01_system is not "
                    "available; using %s" % self.relative_topic)
        else:
            self.get_logger().info(
                "TargetObservation disabled (use_vision_state=false); legacy relative topic is diagnostic only")

        self.platform_history = PlatformHistory(self.sensor_timeout_s,
                                               self.sensor_alignment_s,
                                               self.require_angular_velocity)
        # 与原生控制器同机运行；boot必须来自本机，不能接受上次启动留下的READY。
        self.native_gate = NativeFlightGate(
            Path('/proc/sys/kernel/random/boot_id').read_text().strip(), self.native_flight_timeout_s)
        # 原子绑定到原生已确认的raw姿态。tuple只保存原值，不依赖桥是否恰好收到同一帧。
        self.native_attitude_snapshot = None
        self.native_navigation_was_current = False
        self.platform_reference_counters = None
        self.platform_attitude_stamp_ns = 0
        self.native_task_identity = None
        self.native_tracking_started = False
        self.horizontal_fault_reason = ''
        self.horizontal_policy_reason = 'WAIT_NATIVE'
        self.target_identity = MeasurementIdentity()
        self.target_consecutive = 0
        self.last_target_sample = 0
        self.pending_target = None
        self.sensor_reason = 'PLATFORM_NOT_RECEIVED'
        self.target_reason = 'TARGET_NOT_RECEIVED'
        self.target_locked = False
        self.last_sensor_status_time = -1e9
        # 来源/坐标失效不能让缓存终端阶段在反馈恢复后自行续跑。
        self.guidance_invalidated = False
        self.platform_state_sub = None
        if self.platform_input_mode == 'px4':
            self.native_flight_sub = self.create_subscription(
                NativeFlightSnapshot,self.native_flight_topic,self._on_native_snapshot,SENSOR_QOS)
            self.platform_state_sub = self.create_subscription(
                PlatformState, self.platform_state_topic, self._on_platform_state, SENSOR_QOS)
        else:
            # 仅离线仿真可订阅上游分离端点；生产绝不退回模拟数据或零值。
            self.pose_sub = self.create_subscription(
                PoseStamped, self.platform_pose_topic, self._on_pose, SENSOR_QOS)
            self.velocity_sub = self.create_subscription(
                Vector3Stamped, self.platform_velocity_topic, self._on_velocity, SENSOR_QOS)
            self.omega_sub = self.create_subscription(
                Vector3Stamped, self.platform_omega_topic, self._on_omega, SENSOR_QOS)
            self.contact_sub = self.create_subscription(
                Float32, self.contact_force_topic, self._on_contact, 10)
            self.impact_sub = self.create_subscription(
                Bool, self.impact_topic, self._on_impact, 10)
            self.reset_sub = self.create_subscription(
                Empty, self.reset_event_topic, self._on_reset_event, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.sensor_status_pub = self.create_publisher(String, '/fc01/mission/sensors/status', COMMAND_QOS)
        self.last_state_time = 0.0

        # ---- State --------------------------------------------------------
        self.platform_pos = None
        self.platform_vel = None
        self.platform_omega = None   # 缺测不能等同静止，实际body FLU角速度。
        self.platform_rot = np.eye(3)
        self.platform_pose_time = 0.0
        self.platform_start = None
        self.yaw_cmd = None
        # Yaw-rate feedforward published in VelocityCommandV2.wz. Derived from
        # the rate of change of yaw_cmd; body FRD (z down), so the world yaw
        # rate is negated when it goes into the message.
        self.last_published_yaw = None
        self.last_published_yaw_time = 0.0
        self.yaw_rate_cmd = 0.0
        self.yaw_rate_ff = 0.0

        # Vision-only target world position and velocity (never ground
        # truth). Velocity is estimated from successive estimates.
        self.vision_rel = None
        self.vision_time = 0.0
        self.vision_source = "none"
        self.est_pos = None
        self.est_time = 0.0
        self.target_vel_est = np.zeros(3)
        self.last_los_world = None

        self.contact_force = 0.0
        self.contact_time = 0.0

        self.flight_state = 'wait_native' if self.platform_input_mode == 'px4' else 'takeoff'
        self.takeoff_complete_time = None
        self.seek_start = -1.0e9
        self.push_aim = None
        self.pushing = False
        self.push_time = 0.0
        self.push_contact_force = 0.0
        self.retreat_start = 0.0
        self.retreat_dir = np.array([-1.0, 0.0, 0.0])
        self.strike_start = 0.0
        self.terminal_start = None
        self.terminal_cmd = np.zeros(3)
        self.impacted = False
        self.impact_time = 0.0
        self.reset_time = 0.0

        self.vision_control_seq = 1
        self.navigation_seq = 1
        self.reset_counter = 0
        self.last_vision_control_time = 0.0
        self.last_navigation_time = 0.0
        self.last_control_debug = 0.0
        self.last_v_cmd = np.zeros(3)
        self.last_tick_time = time.monotonic()
        self.start_time = self.last_tick_time

        self.get_logger().info(
            "mission: mode=%s, vision=%s, plant pose=%s, out=%s at %.0f Hz"
            % (self.engage_mode, self.vision_state_topic,
               self.platform_state_topic if self.platform_input_mode=='px4' else self.platform_pose_topic,
               self.vision_control_topic,
               self.vision_control_hz))
        self.control_timer = self.create_timer(
            1.0 / self.control_hz, self._tick)

    # ---- Inputs ----------------------------------------------------------
    def _message_age(self, stamp):
        """真实源年龄；旧/未来帧直接拒绝，绝不压缩成固定0.15秒。"""
        value = stamp_ns(stamp)
        now = self.get_clock().now().nanoseconds
        if not fresh(value, now, self.sensor_timeout_s):
            raise ValueError('TARGET_SOURCE_STALE_OR_FUTURE')
        return (now-value)*1e-9

    def _clear_target(self, reason):
        """撤销锁定与所有目标估计；同一失效不能留下非零前馈或上一帧位置。"""
        # 跟踪已经真正发布后，失锁/重置/来源变化不能伪装为“从未见过目标”，
        # 否则旧航点会重新接管。原生独立执行故障下降；这里撤销全部水平和偏航意图。
        if (self.platform_input_mode == 'px4' and self.native_tracking_started and
                not self.horizontal_fault_reason):
            self.horizontal_fault_reason = reason
        self.vision_rel = None
        self.est_pos = None
        self.vision_time = 0.
        self.vision_state_time = 0.
        self.target_vel_est[:] = 0.
        self.target_consecutive = 0
        self.target_locked = False
        self.pending_target = None
        self.last_target_sample = 0
        self.target_reason = reason
        # terminal/strike/push/retreat都含上一目标或旧坐标系的冻结需求。
        # 先锁存，再清缓存；恢复传感器只允许观察重锁定，不自动恢复旧任务。
        cached_phase = (self.terminal_start is not None or
                        self.flight_state in ('strike', 'push', 'retreat'))
        if cached_phase:
            self.guidance_invalidated = True
        self.terminal_start = None
        self.terminal_cmd[:] = 0.
        self.push_aim = None
        self.pushing = False
        self.push_contact_force = 0.
        self.last_los_world = None
        self.last_v_cmd[:] = 0.
        self.yaw_rate_cmd = self.yaw_rate_ff = 0.
        self.last_published_yaw = None

    def _native_navigation_current(self, now):
        if self.platform_input_mode != 'px4':
            return True
        # 即使原生消息未到期，图中writer失踪或争抢也应立即撤销。
        if self.count_publishers(self.native_flight_topic) != 1:
            return self.native_gate.invalidate('NATIVE_PUBLISHER_NOT_UNIQUE')
        if not self.native_gate.current(self.get_clock().now().nanoseconds,now):
            return False
        native=self.native_gate.latest
        expected=(int(native.z_reset_counter),int(native.vz_reset_counter),int(native.quat_reset_counter))
        if self.platform_reference_counters != expected:
            # 原生已换估计参考而平台桥还未更新时，不混用旧自机姿态/位置发布水平需求。
            self.native_gate.reason='NATIVE_PLATFORM_REFERENCE_MISMATCH'
            return False
        return self._native_attitude_current(now) is not None

    @staticmethod
    def _native_snapshot_binding(state):
        """不可变身份：原生进程/任务/重置、该状态的序号/时刻及其raw姿态戳。"""
        return (state.source_id,state.boot_id,int(state.controller_pid),int(state.task_epoch),
                int(state.z_reset_counter),int(state.vz_reset_counter),int(state.quat_reset_counter),
                int(state.sequence),stamp_ns(state.stamp),int(state.attitude_sample_stamp_us))

    def _native_attitude_current(self, now):
        """返回与当前原生状态原子绑定的ENU/FLU旋转；缺证据返回None并说明原因。

        源采样年龄和原生首次接收年龄分别复核。新状态外壳或DDS重复不能给旧姿态续鲜；
        bridge的50Hz历史只用于图像时间对齐，不能代替原生确认的200Hz raw样本。
        """
        snapshot = self.native_attitude_snapshot
        native = self.native_gate.latest
        if snapshot is None or native is None:
            self.native_gate.reason = 'NATIVE_ATOMIC_ATTITUDE_NOT_RECEIVED'
            return None
        binding, raw_q, origin, received_ns = snapshot
        if binding != self._native_snapshot_binding(native):
            self.native_gate.reason = 'NATIVE_ATOMIC_STATE_BINDING_MISMATCH'
            return None
        identity = self.platform_history.identity
        if identity is None or origin != identity[2]:
            self.native_gate.reason = 'NATIVE_PLATFORM_ORIGIN_MISMATCH'
            return None
        limit = min(self.sensor_timeout_s,self.native_gate.timeout)
        if (not fresh(binding[-1]*1000,self.get_clock().now().nanoseconds,limit) or
                not 0 <= round(now*1e9)-received_ns <= limit*1e9):
            self.native_gate.reason = 'NATIVE_ATOMIC_ATTITUDE_SOURCE_OR_RECEIVE_STALE'
            return None
        try:
            return rotation_xyzw(quaternion_ned_frd_to_enu_flu(raw_q))
        except (ValueError,OverflowError):
            self.native_gate.reason = 'NATIVE_ATOMIC_ATTITUDE_INVALID'
            return None

    def _on_native_snapshot(self, msg):
        """校验只读原子封套，再复用既有阶段门；不发送解锁、起飞或高度命令。

        整条消息失效会撤销当前授权。相同状态/相同raw测量的q、原点、首次接收戳必须
        保持不变，不能用新的外壳将相互矛盾的数据拼成一次“有效”控制上下文。
        """
        now = time.monotonic()
        try:
            if msg.schema_version != 1:
                raise ValueError('NATIVE_ATOMIC_SCHEMA_INVALID')
            state = msg.state
            binding = self._native_snapshot_binding(state)
            raw_q = tuple(float(value) for value in finite(msg.attitude_wxyz,4))
            # 复用已有Hamilton NED/FRD转换；零、NaN和明显非单位四元数会抛异常。
            quaternion_ned_frd_to_enu_flu(raw_q)
            origin = msg.origin_id
            if not isinstance(origin,str) or not origin or len(origin.encode('utf-8')) > 256:
                raise ValueError('NATIVE_ATOMIC_ORIGIN_INVALID')
            received_ns = int(msg.attitude_receive_monotonic_ns)
            limit = min(self.sensor_timeout_s,self.native_gate.timeout)
            if (received_ns <= 0 or received_ns > int(state.native_monotonic_ns) or
                    not 0 <= round(now*1e9)-received_ns <= limit*1e9 or
                    not fresh(binding[-1]*1000,self.get_clock().now().nanoseconds,limit)):
                raise ValueError('NATIVE_ATOMIC_ATTITUDE_SOURCE_OR_RECEIVE_STALE')
            candidate = (binding,raw_q,origin,received_ns)
            old = self.native_attitude_snapshot
            if old is not None:
                old_binding = old[0]
                # 同seq/stamp必须是整条重复；同raw戳跨新状态外壳也不能改q或续接收TTL。
                same_state = binding[:-1] == old_binding[:-1]
                same_sample = binding[:7] == old_binding[:7] and binding[-1] == old_binding[-1]
                if ((same_state and candidate != old) or
                        (same_sample and candidate[1:] != old[1:])):
                    raise ValueError('NATIVE_ATOMIC_DUPLICATE_INCONSISTENT')
                if binding[:7] == old_binding[:7] and binding[-1] < old_binding[-1]:
                    raise ValueError('NATIVE_ATOMIC_ATTITUDE_REGRESSED')
            self._on_native_flight(state)
            # 阶段门可能因进程/任务变化或时间/权限失效而拒绝；绝不单靠q有效恢复授权。
            accepted = self.native_gate.latest
            if accepted is not None and self._native_snapshot_binding(accepted) == binding:
                if candidate != old:
                    self.native_attitude_snapshot = candidate
            else:
                self.native_attitude_snapshot = None
        except (AttributeError,TypeError,ValueError,OverflowError) as error:
            reason = str(error) if isinstance(error,ValueError) else 'NATIVE_ATOMIC_ENVELOPE_INVALID'
            self.native_attitude_snapshot = None
            self.native_gate.invalidate(reason)
            if self.native_navigation_was_current:
                self._clear_target(reason)
                self.flight_state = 'wait_native'
            self.native_navigation_was_current = False

    def _on_native_flight(self, msg):
        now=time.monotonic()
        generation=self.native_gate.generation
        current=self.native_gate.accept(msg,self.get_clock().now().nanoseconds,now,
                                        self.count_publishers(self.native_flight_topic))
        if generation != self.native_gate.generation:
            self.native_attitude_snapshot = None
            # 只有原生新任务epoch推进才解除本任务跟踪故障；相机恢复/来源切换不能解除。
            task = (msg.boot_id, int(msg.controller_pid), msg.source_id, int(msg.task_epoch))
            new_task = (self.native_task_identity is not None and
                        task[:3] == self.native_task_identity[:3] and
                        task[3] > self.native_task_identity[3])
            self._clear_target('NATIVE_TASK_OR_PROCESS_CHANGED')
            if new_task:
                self.native_tracking_started = False
                self.horizontal_fault_reason = ''
                self.platform_start = None
            self.native_task_identity = task
            self.flight_state='wait_native'
            self.takeoff_complete_time=None
            self.seek_start=now
            self.reset_counter+=1
        if self.native_navigation_was_current and not current:
            self._clear_target(self.native_gate.reason)
            self.flight_state='wait_native'
        self.native_navigation_was_current=current

    def _on_platform_state(self, msg):
        now, ros = time.monotonic(), self.get_clock().now().nanoseconds
        generation = self.platform_history.generation
        accepted = self.platform_history.accept(msg, ros, now,
                          self.count_publishers(self.platform_state_topic))
        if generation != self.platform_history.generation:
            self._clear_target(self.platform_history.reason)
            self.platform_start = None
            self.yaw_cmd = None
        self.sensor_reason = self.platform_history.reason
        if accepted:
            self.platform_attitude_stamp_ns = int(msg.attitude_stamp_ns)
            self.platform_reference_counters=(int(msg.z_reset_counter),int(msg.vz_reset_counter),
                                              int(msg.quat_reset_counter))
            sample = self.platform_history.latest
            self.platform_pos = sample.position.copy()
            self.platform_vel = sample.velocity.copy()
            self.platform_rot = sample.rotation.copy()
            self.platform_omega = sample.omega.copy() if sample.omega is not None else None
            self.platform_pose_time = now
            if self.platform_start is None:
                self.platform_start = self.platform_pos.copy()
            if self.yaw_cmd is None:
                self.yaw_cmd = math.atan2(self.platform_rot[1,0], self.platform_rot[0,0])
            self._consume_pending(now)
        else:
            self.platform_reference_counters=None
            self.platform_pos = self.platform_vel = self.platform_omega = None

    def _on_pose(self, msg):
        """显式simulation模式的兼容输入；生产模式永不消费该回调。"""
        if self.platform_input_mode != 'simulation':
            return
        try:
            self._message_age(msg.header.stamp)
            self.platform_pos = finite([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z],3)
            q = msg.pose.orientation
            self.platform_rot = rotation_xyzw([q.x,q.y,q.z,q.w])
            self.platform_pose_time = time.monotonic()
            if self.platform_start is None:
                self.platform_start = self.platform_pos.copy()
            if self.yaw_cmd is None:
                self.yaw_cmd = math.atan2(self.platform_rot[1,0],self.platform_rot[0,0])
        except ValueError:
            self.platform_pos = None

    def _on_velocity(self, msg):
        if self.platform_input_mode != 'simulation':
            return
        try:
            self._message_age(msg.header.stamp)
            self.platform_vel = finite([msg.vector.x,msg.vector.y,msg.vector.z],3)
            self.platform_velocity_time = time.monotonic()
        except ValueError:
            self.platform_vel = None

    def _on_omega(self, msg):
        if self.platform_input_mode != 'simulation':
            return
        try:
            self._message_age(msg.header.stamp)
            self.platform_omega = finite([msg.vector.x,msg.vector.y,msg.vector.z],3)
            self.platform_omega_time = time.monotonic()
        except ValueError:
            self.platform_omega = None

    def _on_contact(self, msg):
        if math.isfinite(msg.data):
            self.contact_force = max(0., float(msg.data))
            self.contact_time = time.monotonic()

    def _on_impact(self, msg):
        if self.platform_input_mode == 'simulation' and bool(msg.data) and not self.impacted:
            self.impacted = True
            self.impact_time = time.monotonic()

    def _on_reset_event(self, _msg):
        if self.platform_input_mode == 'simulation':
            self._reset_mission()
            self._clear_target('SIMULATION_RESET')

    def _on_relative(self, msg):
        # 无来源、序号、标定字段的旧PointStamped不作为生产目标测量。
        # 即使主TargetObservation失效，也不偷偷降级绕过它的质量检查。
        return

    def _on_state(self, msg):
        now, ros = time.monotonic(), self.get_clock().now().nanoseconds
        try:
            self._message_age(msg.stamp)
            if self.count_publishers(self.vision_state_topic) != 1:
                raise ValueError('TARGET_PUBLISHER_NOT_UNIQUE')
            if (msg.schema_version != 1 or msg.frame_id != 'body_frd' or
                    msg.body_frame_id != 'body_frd'):
                raise ValueError('TARGET_SCHEMA_OR_FRAME_INVALID')
            identity = self.target_identity.observe(msg.source_id,msg.reset_counter,
                                                    msg.sequence,stamp_ns(msg.stamp),now)
            # 无效的新帧也消耗身份水位，随后同帧改成“有效”不能使它复活。
            if not msg.initialized or not msg.tracking or not msg.calibration_valid:
                raise ValueError('TARGET_TRACKING_OR_CALIBRATION_INVALID')
            if not msg.valid_fields & msg.POSITION:
                raise ValueError('TARGET_POSITION_INVALID')
            position = finite(msg.position, 3)
            if msg.valid_fields & msg.VELOCITY:
                finite(msg.velocity, 3)
            for bit, values in ((msg.POSE_COVARIANCE,msg.pose_covariance),
                                (msg.TWIST_COVARIANCE,msg.twist_covariance)):
                if msg.valid_fields & bit:
                    covariance = finite(values,36)
                    if np.any(covariance[[0,7,14,21,28,35]] < 0):
                        raise ValueError('TARGET_COVARIANCE_INVALID')
            if msg.valid_fields & msg.QUALITY and not (math.isfinite(msg.quality) and 0<=msg.quality<=1):
                raise ValueError('TARGET_QUALITY_INVALID')
            if identity == 'DUPLICATE':
                # 不改vision_time或连续帧计数；延迟失效由原始源时刻与接收TTL触发。
                return
            if identity != 'OK':
                raise ValueError(identity)
            if not self.camera_extrinsics_confirmed or not self.camera_extrinsics_id:
                raise ValueError('CAMERA_LEVER_ARM_UNCONFIRMED')
            # 相机通常比PX4平台反馈快。待帧需要后一个平台样本才能插值；
            # 若每帧都覆盖单槽，待帧会始终跑在平台前面，持续有图也可能误报过期。
            # 先尝试消费原待帧，仍未对齐就保留它；只持有一帧，不积压、不外推，
            # 原始源戳/接收时刻和既有TTL都不刷新。新无效帧仍由上方检查立即撤销。
            self._consume_pending(now)
            if self.pending_target is None:
                self.pending_target = (position,stamp_ns(msg.stamp),now,msg.source_id)
                self._consume_pending(now)
        except ValueError as error:
            self._clear_target(str(error))

    def _consume_pending(self, now):
        if self.pending_target is None:
            return
        position, stamp, received, source = self.pending_target
        ros = self.get_clock().now().nanoseconds
        if not fresh(stamp,ros,self.sensor_timeout_s) or not 0<=now-received<=self.sensor_timeout_s:
            self._clear_target('TARGET_PENDING_STALE')
            return
        if not self.platform_history.current(ros,now):
            self.target_reason = 'TARGET_WAIT_PLATFORM_CURRENT'
            return
        try:
            aligned = self.platform_history.aligned(stamp)
        except ValueError as error:
            self.target_reason = str(error)
            return
        self.pending_target = None
        # 相邻锁定样本必须都是新采集帧；空档超过预算重新累计。
        if self.last_target_sample and (stamp-self.last_target_sample)*1e-9>self.sensor_timeout_s:
            self.target_consecutive = 0
        self.last_target_sample = stamp
        self.target_consecutive += 1
        self.target_locked = self.target_consecutive >= self.target_lock_frames
        self._ingest(position,received,source,aligned=aligned,measurement_stamp=stamp)
        self.vision_state_time = received
        self.target_reason = 'TARGET_LOCKED' if self.target_locked else 'TARGET_WAIT_CONTINUOUS_NEW_FRAMES'
        if not self.target_locked:
            self.vision_rel = None

    # ---- Helpers ---------------------------------------------------------
    @staticmethod
    def _quat_rotation(w, x, y, z):
        """Quaternion (w, x, y, z) to a body-FLU -> world rotation."""
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-12:
            return np.eye(3)
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ])

    def _camera_world_position(self):
        if self.platform_pos is None:
            return None
        return self.platform_pos + self.platform_rot @ self.camera_pos

    def _ingest(self, p_frd, now, source, age=0.0, aligned=None, measurement_stamp=None):
        """相机原点的FRD目标量转到PX4原local ENU；杆臂只在这里加一次。

        使用图像源时刻内插的真实自机姿态/位置，不以“当前角速度×猜测延迟”外推。
        相机节点已把相机安装旋转应用到分量，原点仍在光心。FLU杆臂来自显式现场配置。
        """
        if aligned is None:
            return  # 禁止绕过时间对齐，旧调用方不能拿最新平台姿态凑一帧。
        p = finite(p_frd,3)
        rel_world = aligned.rotation @ np.array([p[0],-p[1],-p[2]])
        origin = aligned.position + aligned.rotation @ self.camera_pos
        estimate = origin+rel_world
        if not np.all(np.isfinite(estimate)):
            self._clear_target('TARGET_WORLD_NONFINITE')
            return
        self.last_los_world = rel_world/max(float(np.linalg.norm(rel_world)),1e-6)
        sample_time = measurement_stamp*1e-9
        dt_est = sample_time-self.est_time
        if self.est_pos is not None and 1e-3<dt_est<self.sensor_timeout_s:
            raw_velocity = (estimate-self.est_pos)/dt_est
            blend = min(1.,dt_est/.3)
            self.target_vel_est += (raw_velocity-self.target_vel_est)*blend
            speed = float(np.linalg.norm(self.target_vel_est))
            if speed>2.:
                self.target_vel_est *= 2./speed
        self.est_pos, self.est_time = estimate, sample_time
        self.vision_rel, self.vision_time, self.vision_source = p, now, source

    def _blend_angle(self, current, target, alpha):
        diff = math.atan2(math.sin(target - current),
                          math.cos(target - current))
        return current + diff * alpha

    def _reset_mission(self):
        self.flight_state = 'wait_native' if self.platform_input_mode == 'px4' else 'takeoff'
        self.guidance_invalidated = False  # 仅显式任务重置解除，非传感器恢复自动解除。
        self.takeoff_complete_time = None
        self.seek_start = -1.0e9
        self.terminal_start = None
        self.terminal_cmd = np.zeros(3)
        self.impacted = False
        self.pushing = False
        self.push_time = 0.0
        self.push_contact_force = 0.0
        self.strike_start = 0.0
        self.retreat_start = 0.0
        self.retreat_dir = np.array([-1.0, 0.0, 0.0])
        self.last_published_yaw = None
        self.last_published_yaw_time = 0.0
        self.yaw_rate_cmd = 0.0
        self.yaw_rate_ff = 0.0
        self.push_aim = None
        self.vision_rel = None
        self.est_pos = None
        self.est_time = 0.0
        self.target_vel_est[:] = 0.0
        self.last_los_world = None
        self.last_v_cmd[:] = 0.0
        # Re-capture the start pose from the next plant message: the sim
        # respawns on the H pad.
        self.platform_start = None
        self.yaw_cmd = None
        self.reset_counter += 1
        self.last_navigation_time = 0.0
        self.reset_time = time.monotonic()
        self.get_logger().info("mission reset: takeoff from the plant pose")

    # ---- Guidance --------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        ros = self.get_clock().now().nanoseconds
        dt = float(min(max(now - self.last_tick_time, 0.0), 0.05))
        self.last_tick_time = now
        if self.platform_input_mode == 'px4':
            pose_fresh = (self.platform_history.clock(ros, now) and
                          self.platform_history.current(ros, now) and
                          self.platform_pos is not None)
        else:
            pose_fresh = (self.platform_pos is not None and self.platform_vel is not None and
                          self.platform_omega is not None and
                          all(0 <= now-getattr(self,name,-1e9) <= self.sensor_timeout_s for name in
                              ('platform_pose_time','platform_velocity_time','platform_omega_time')))
        if not pose_fresh:
            self.sensor_reason = self.platform_history.reason if self.platform_input_mode=='px4' else 'SIMULATION_FIELDS_STALE'
            self._clear_target(self.sensor_reason)
            # 显式撤销，不靠接收方最终超时继续沿用旧非零指令；不更改原生RC保护。
            self._publish_command([0.,0.,0.], now, valid=False)
            self._publish_navigation(now, force_invalid=True)
            self._publish_sensor_status(now)
            return
        self._consume_pending(now)
        if (self.last_target_sample and
                (not fresh(self.last_target_sample,ros,self.sensor_timeout_s) or
                 not 0 <= now-self.vision_state_time <= self.sensor_timeout_s)):
            self._clear_target('TARGET_SOURCE_OR_RECEIVE_STALE')
        self.sensor_reason = 'OK'
        self._publish_sensor_status(now)
        v_cmd, valid = self._update_mission(now, dt)
        self.last_v_cmd = v_cmd.copy()
        self._publish_command(v_cmd, now, valid=valid)
        self._publish_navigation(now)
        self._publish_state(now)
        self._debug_log(now)

    def _publish_sensor_status(self, now):
        """独立状态页：缺项逐项可见；不把传感器ready解释为RC/飞行授权。"""
        if now-self.last_sensor_status_time < .2:
            return
        self.last_sensor_status_time = now
        missing = []
        if self.sensor_reason != 'OK':
            missing.append(self.sensor_reason)
        if not self.camera_extrinsics_confirmed or not self.camera_extrinsics_id:
            missing.append('CAMERA_LEVER_ARM_UNCONFIRMED')
        if not self.target_locked:
            missing.append(self.target_reason)
        if self.horizontal_fault_reason:
            missing.append('HORIZONTAL_FAULT:'+self.horizontal_fault_reason)
        if not self.standoff_calibration_confirmed or not self.standoff_calibration_id:
            missing.append('STANDOFF_CALIBRATION_UNCONFIRMED')
        native_current = self._native_navigation_current(now)
        if self.platform_input_mode=='px4' and not native_current:
            missing.append(self.native_gate.reason)
        origin = self.platform_history.identity[2] if self.platform_history.identity else ''
        if not self.waypoint_alignment_confirmed or self.waypoint_alignment_origin_id != origin:
            missing.append('WAYPOINT_ORIGIN_UNCONFIRMED')
        self.sensor_status_pub.publish(String(data=json.dumps({
            'kind':'mission_sensors','control_authority':False,
            'platform_input_mode':self.platform_input_mode,'missing':missing,
            'target_locked':self.target_locked,'continuous_new_frames':self.target_consecutive,
            'target_reason':self.target_reason,'platform_reason':self.sensor_reason,
            'guidance_invalidated':self.guidance_invalidated,
            'native_navigation_current':native_current,
            'native_phase':self.native_gate.latest.phase if self.native_gate.latest is not None else '',
            'native_reason':self.native_gate.reason,
            'origin_id':origin,'last_target_stamp_ns':self.last_target_sample,
            'target_source':self.target_identity.identity,
            'engage_mode':self.engage_mode,
            'horizontal_policy_reason':self.horizontal_policy_reason,
            'horizontal_fault':self.horizontal_fault_reason,
            'standoff_calibration_id':self.standoff_calibration_id,
            'note':'原生独占人工RC起飞和1.5m垂直闭环；本节点只在原生到高授权后产生水平意图'
        },ensure_ascii=False,allow_nan=False)))

    def _update_mission(self, now, dt):
        """Run the state machine; return a world-frame velocity command."""
        v_cmd = np.zeros(3)
        valid = True
        platform_pos = self.platform_pos
        platform_vel = self.platform_vel
        if self.guidance_invalidated:
            return v_cmd, False
        if self.platform_input_mode == 'px4':
            # 实机起飞/定高唯一归原生Core所有；不进入下方上游模拟起飞或冲撞阶段。
            return self._update_native_mission(now, dt)

        # Sustained impact is a timed phase: after pressing for push_hold_s
        # the aircraft backs straight out and holds a hover. Check before the
        # state chain so the push branch does not run one more tick of
        # forward thrust into the contact.
        if (self.engage_mode == "push" and self.control_enabled and
                self.flight_state == "push" and self.push_hold_s > 0.0 and
                now - self.push_time >= self.push_hold_s):
            self.flight_state = "retreat"
            self.retreat_start = now
            if self.push_aim is not None:
                away = platform_pos - self.push_aim
            else:
                away = -np.array([math.cos(self.target_face_yaw),
                                  math.sin(self.target_face_yaw), 0.0])
            if float(np.linalg.norm(away[:2])) < 1e-3:
                away = -np.array([math.cos(self.target_face_yaw),
                                  math.sin(self.target_face_yaw), 0.0])
            self.retreat_dir = away[:2] / float(
                np.linalg.norm(away[:2]))
            self.pushing = False
            self.get_logger().info(
                "push hold done: %.1f s pressed -> retreat %.2f m "
                "at %.2f m/s"
                % (self.push_hold_s, self.retreat_distance_m,
                   self.retreat_speed))

        if self.impacted:
            valid = False
        elif (self.engage_mode == "push" and self.control_enabled and
              self.flight_state == "push"):
            # Sustained impact: keep a small velocity into the fixed light.
            # The plant's velocity loop turns that setpoint into a
            # continuous contact force.
            #
            # Push toward the world point captured at hand-over instead of
            # the live estimate: within about a metre the light fills most
            # of the frame, its contour gets clipped and the monocular range
            # grows wildly.
            if self.push_aim is None and self.est_pos is not None:
                self.push_aim = self.est_pos.copy()
            if self.push_aim is not None:
                to_target = self.push_aim - platform_pos
                range_m = float(np.linalg.norm(to_target))
                if range_m > self.push_release_m:
                    self.flight_state = "seek"
                    self.pushing = False
                    self.push_aim = None
                    self.push_contact_force = 0.0
                    self.get_logger().info(
                        "push released: moved %.2f m off the aim point -> seek"
                        % range_m)
                else:
                    self.pushing = True
                    # Hold the heading captured when push engaged: re-aiming
                    # every step is unstable when the drone is ~0.1 m away
                    # (a 2 cm lateral offset becomes a 15-20 deg yaw
                    # command).
                    if range_m > 1e-3:
                        dir_xy = to_target[:2] / max(
                            float(np.linalg.norm(to_target[:2])), 1e-6)
                    else:
                        dir_xy = np.array([
                            math.cos(self.yaw_cmd), math.sin(self.yaw_cmd)])
                    speed = self.push_speed
                    settling = (self.strike_settle_s > 0.0 and
                                now - self.push_time < self.strike_settle_s)
                    contact = (now - self.contact_time <=
                               self.contact_timeout_s)
                    force = self.contact_force if contact else 0.0
                    self.push_contact_force = force
                    if contact and self.push_max_force_n > 0.0:
                        if force > self.push_max_force_n:
                            speed *= max(
                                0.15, self.push_max_force_n / force)
                    v_cmd = np.array(
                        [dir_xy[0] * speed, dir_xy[1] * speed, 0.0])
                    if settling:
                        # Right after a strike, damp the horizontal velocity
                        # instead of pressing: commanding forward thrust into
                        # the contact pumps energy into it and the drone
                        # rebounds one to two metres.
                        brake = max(0.0, self.strike_settle_brake_mps)
                        v_cmd[:2] = np.clip(
                            -2.0 * platform_vel[:2], -brake, brake)
                    altitude = self._push_altitude()
                    vz = (1.5 * (altitude - float(platform_pos[2])) -
                          0.6 * float(platform_vel[2]))
                    v_cmd[2] = float(np.clip(vz, -0.6, 0.8))
                    if (float(platform_pos[2]) < self.min_altitude_m
                            and v_cmd[2] < 0.0):
                        v_cmd[2] = 0.0
        elif (self.engage_mode == "push" and self.control_enabled and
              self.flight_state == "strike"):
            # Terminal strike: charge the light's map point at a constant
            # speed instead of letting the position loop brake with
            # distance. The strike does not depend on the detection staying
            # in frame: for a static map target its world position is exact.
            to_target = self.target_pos - platform_pos
            range_m = float(np.linalg.norm(to_target))
            # Hold the airframe square to the light's face, not along the
            # bearing to the target: at 0.1 m the bearing is dominated by
            # the last centimetres of lateral offset, and striking the flat
            # panel at an angle spins the 240 g frame.
            self.yaw_cmd = self._blend_angle(
                self.yaw_cmd, self.target_face_yaw, min(1.0, dt / 0.15))
            dir_xy = to_target[:2] / max(
                float(np.linalg.norm(to_target[:2])), 1e-6)
            v_cmd = np.array([dir_xy[0] * self.strike_speed,
                              dir_xy[1] * self.strike_speed, 0.0])
            altitude = self._push_altitude()
            v_cmd[2] = float(np.clip(
                1.5 * (altitude - float(platform_pos[2])) -
                0.6 * float(platform_vel[2]), -0.6, 0.5))
            if range_m <= self.strike_engage_m:
                # Hand over to the push loop, which presses with push_speed
                # and holds the altitude. The charge momentum is absorbed by
                # the soft, damped target contact.
                self.flight_state = "push"
                self.pushing = True
                self.push_time = now
                self.push_contact_force = 0.0
                self.push_aim = self.target_pos.copy()
                self.get_logger().info(
                    "strike contact: range=%.2f m, speed=%.2f m/s "
                    "-> hold/push at %.2f m/s"
                    % (range_m, float(np.linalg.norm(platform_vel)),
                       self.push_speed))
                v_cmd = np.zeros(3)
            elif now - self.strike_start > self.strike_timeout_s:
                # Overshot or was deflected: drop the charge and let seek
                # re-acquire. The map position is still known.
                self.flight_state = "seek"
                self.get_logger().warn(
                    "strike timeout at range=%.2f m -> seek" % range_m)
        elif (self.engage_mode == "push" and self.control_enabled and
              self.flight_state == "retreat"):
            # Back straight out along the line the drone flew in on, nose
            # still facing the light, then hold a stable hover. The speed is
            # eased in with a smoothstep and eased out over the last 25 cm.
            if self.push_aim is not None:
                offset = platform_pos - self.push_aim
            else:
                offset = np.array(
                    [self.retreat_dir[0], self.retreat_dir[1], 0.0])
            d_xy = float(np.linalg.norm(offset[:2]))
            remaining = self.retreat_distance_m - d_xy
            if remaining <= 0.02:
                self.flight_state = "hold"
                self.get_logger().info(
                    "retreat complete: holding %.2f m off the light at "
                    "(%.2f, %.2f, %.2f) m"
                    % (d_xy, platform_pos[0], platform_pos[1],
                       platform_pos[2]))
                v_cmd = np.zeros(3)
            else:
                ease = min(1.0, max(
                    0.0, (now - self.retreat_start) / self.retreat_ramp_s))
                ease = ease * ease * (3.0 - 2.0 * ease)
                brake = min(1.0, remaining / 0.25)
                speed = self.retreat_speed * ease * brake
                v_cmd = np.array([self.retreat_dir[0] * speed,
                                  self.retreat_dir[1] * speed, 0.0])
            altitude = self._push_altitude()
            v_cmd[2] = float(np.clip(
                1.5 * (altitude - float(platform_pos[2])) -
                0.6 * float(platform_vel[2]), -0.6, 0.8))
            if (float(platform_pos[2]) < self.min_altitude_m
                    and v_cmd[2] < 0.0):
                v_cmd[2] = 0.0
        elif self.terminal_start is not None:
            if now - self.terminal_start >= self.terminal_timeout_s:
                # A real contact still ends it; otherwise report a miss and
                # stop commanding.
                self.impacted = True
                self.impact_time = now
                self.terminal_start = None
                self.get_logger().warn(
                    "terminal guidance timeout without contact")
                valid = False
            else:
                # Keep refining the aim while detections are fresh.
                if (self.vision_rel is not None and
                        (now - self.vision_time) <= self.command_timeout_s):
                    self._update_terminal_command()
                v_cmd = self.terminal_cmd
        elif self.control_enabled:
            if self.flight_state == "takeoff":
                remaining = self.takeoff_altitude_m - float(
                    platform_pos[2])
                if remaining <= 0.03:
                    if self.takeoff_complete_time is None:
                        self.takeoff_complete_time = now
                        self.get_logger().info(
                            "takeoff complete: z=%.2f m, hover %.2f s "
                            "before the chase"
                            % (platform_pos[2], self.takeoff_hover_s))
                    # Hold zero velocity: stops the residual climb and lets
                    # the attitude and the camera settle before the chase.
                    v_cmd = np.zeros(3)
                    if (now - self.takeoff_complete_time >=
                            self.takeoff_hover_s):
                        self.flight_state = "seek"
                        self.seek_start = now
                        self.get_logger().info(
                            "takeoff hover done: seeking at up to %.2f m/s"
                            % self.search_speed)
                else:
                    # Climb straight up only: the nose already points at the
                    # target and the camera must stabilise before the chase.
                    v_cmd = np.array([
                        0.0, 0.0,
                        min(self.takeoff_climb_mps,
                            max(0.1, 2.5 * remaining))])
            if self.flight_state == "seek":
                v_cmd = self._seek_command(now, dt)
        else:
            v_cmd = np.zeros(3)

        if (float(platform_pos[2]) < self.min_altitude_m and
                v_cmd[2] < 0.0):
            v_cmd[2] = 0.0
        return v_cmd, valid

    def _tracking_policy_current(self, now):
        """仅目标相对制导准入；不把目标锁定、标定或任务开关当成原生飞行授权。"""
        if self.horizontal_fault_reason:
            self.horizontal_policy_reason = 'TRACKING_FAULT:' + self.horizontal_fault_reason
            return False
        if not self.standoff_calibration_confirmed or not self.standoff_calibration_id:
            self.horizontal_policy_reason = 'STANDOFF_CALIBRATION_UNCONFIRMED'
            return False
        if not self.camera_extrinsics_confirmed or not self.camera_extrinsics_id:
            self.horizontal_policy_reason = 'CAMERA_LEVER_ARM_UNCONFIRMED'
            return False
        if not math.isfinite(self.standoff_m) or not 0 < self.standoff_m <= self.horizontal_max_radius_m:
            self.horizontal_policy_reason = 'STANDOFF_OUTSIDE_HORIZONTAL_ENVELOPE'
            return False
        # 目标源戳与本机单调接收时刻独立校验；频繁计算/重发不能给旧图像续鲜。
        if not (self.last_target_sample and
                fresh(self.last_target_sample,self.get_clock().now().nanoseconds,self.sensor_timeout_s) and
                0. <= now-self.vision_state_time <= self.sensor_timeout_s and
                self.target_locked and self.vision_rel is not None and self.est_pos is not None and
                np.all(np.isfinite(self.vision_rel)) and
                np.all(np.isfinite(self.est_pos)) and np.all(np.isfinite(self.target_vel_est)) and
                0. <= now-self.vision_time <= self.command_timeout_s):
            self.horizontal_policy_reason = 'TARGET_NOT_CURRENTLY_LOCKED'
            return False
        self.horizontal_policy_reason = 'TRACKING_CURRENT'
        return True

    def _waypoint_policy_current(self):
        """绝对ENU航点需匹配当前PX4原点，且相对本次起飞点的水平半径受限。"""
        origin = self.platform_history.identity[2] if self.platform_history.identity else ''
        if (not self.waypoint_alignment_confirmed or not origin or
                self.waypoint_alignment_origin_id != origin):
            self.horizontal_policy_reason = 'WAYPOINT_ORIGIN_UNCONFIRMED'
            return False
        point = np.array([self.navigation_waypoint_x,self.navigation_waypoint_y],dtype=float)
        if not np.all(np.isfinite(point)) or (self.platform_start is None and
                                              not self.position_takeoff_anchor_authority):
            self.horizontal_policy_reason = 'WAYPOINT_OR_TASK_ORIGIN_INVALID'
            return False
        if not self.position_takeoff_anchor_authority:
            radius = float(np.linalg.norm(point-self.platform_start[:2]))
            if not math.isfinite(radius) or radius > self.horizontal_max_radius_m:
                self.horizontal_policy_reason = 'WAYPOINT_OUTSIDE_TASK_RADIUS'
                return False
        self.horizontal_policy_reason = 'WAYPOINT_CURRENT'
        return True

    def _update_native_mission(self, now, dt):
        """实际起飞和高度完全由原生控制器负责；这里只选择航点/定点/目标水平意图。

        未锁定时V2失效，由独立NavigationCommand表达航点或定点；连续锁定后撤销
        航点并发布V2。跟踪中失锁明确故障，不自动倒退到历史地图航点。
        """
        velocity = np.zeros(3)
        if not self._native_navigation_current(now):
            if self.native_tracking_started:
                self._clear_target(self.native_gate.reason)
            self.flight_state = 'wait_native'
            return velocity, False
        if not self.control_enabled:
            self.flight_state = 'native_height_hold'
            return velocity, False
        if self.horizontal_fault_reason:
            self.flight_state = 'horizontal_fault'
            return velocity, False
        if not self._tracking_policy_current(now):
            if self.native_tracking_started:
                self._clear_target(self.horizontal_policy_reason)
                self.flight_state = 'horizontal_fault'
            elif self.horizontal_navigation_mode == 'waypoint' and self._waypoint_policy_current():
                self.flight_state = 'waypoint'
            else:
                self.flight_state = 'native_height_hold'
            return velocity, False
        self.flight_state = 'track'
        offset = (self.est_pos-self.platform_pos)[:2]
        distance = float(np.linalg.norm(offset))
        if distance > 1e-6:
            direction = offset/distance
            # 距离低于经确认的安全间距时也会向后修正，而不是继续逼近目标。
            error = offset-direction*self.standoff_m
        else:
            error = np.zeros(2)
        velocity[:2] = self.target_vel_est[:2]+self.controller_gain*error
        speed = float(np.linalg.norm(velocity[:2]))
        if not math.isfinite(speed):
            self._clear_target('TRACKING_VELOCITY_NONFINITE')
            return np.zeros(3), False
        speed_limit = min(self.track_max_speed,self.horizontal_max_speed_mps)
        if speed > speed_limit > 0.:
            velocity[:2] *= speed_limit/speed
        if distance > 1e-6:
            target_yaw = math.atan2(offset[1],offset[0])
            self.yaw_cmd = self._blend_angle(self.yaw_cmd,target_yaw,min(1.,dt/.2))
        # ENU世界垂向明确为0；倾斜时机体系z分量仍须保留，才能完整旋回原水平向量。
        return velocity, True

    def _push_altitude(self):
        return (self.push_altitude_m if self.push_altitude_m > 0.0 else
                self.target_base_z + self.push_altitude_offset_m)

    def _seek_command(self, now, dt):
        """Seek: vision chase when locked, map-point cruise otherwise."""
        platform_pos = self.platform_pos
        platform_vel = self.platform_vel
        v_cmd = np.zeros(3)
        fresh = (self.vision_rel is not None and
                 (now - self.vision_time) <= self.command_timeout_s)
        if fresh:
            to_target = self.est_pos - platform_pos
            # Keep the nose (and the fixed camera) pointed at the target,
            # otherwise it leaves the narrow FOV in a cross-track chase.
            target_yaw = math.atan2(to_target[1], to_target[0])
            self.yaw_cmd = self._blend_angle(
                self.yaw_cmd, target_yaw, min(1.0, dt / 0.2))
            error = to_target.copy()
            if self.engage_mode == "track":
                # Hold standoff_m before the target along the line of sight
                # (subtracting it from world x only works at +x targets).
                los = to_target / max(
                    float(np.linalg.norm(to_target)), 1e-6)
                error = to_target - los * self.standoff_m
            # Feed the target's estimated world velocity forward, otherwise
            # track mode lags and oscillates on a moving ball. The velocity
            # comes from the vision estimate, never ground truth.
            v_cmd = (self.target_vel_est +
                     self.controller_gain * error)
            speed = float(np.linalg.norm(v_cmd))
            speed_limit = (self.track_max_speed
                           if self.engage_mode == "track"
                           else self.max_speed)
            if speed > speed_limit:
                v_cmd *= speed_limit / speed
            if (self.engage_mode == "push" and
                    self.push_altitude_m > 0.0):
                # Approach at the height the light will be pushed at, not
                # at the estimated light height.
                v_cmd[2] = float(np.clip(
                    1.5 * (self.push_altitude_m -
                           float(platform_pos[2])) -
                    0.6 * float(platform_vel[2]), -0.5, 0.6))

            if (self.engage_mode == "push" and
                    self.target_motion == "static" and
                    self.strike_speed > 0.0 and
                    float(np.linalg.norm(to_target)) <=
                    self.strike_trigger_m):
                # Vision has locked the light: stop creeping in with the
                # position loop and charge. This tick keeps the seek
                # command; the strike branch takes over on the next tick.
                self.flight_state = "strike"
                self.strike_start = now
                self.push_aim = self.target_pos.copy()
                self.get_logger().info(
                    "target locked: range=%.2f m -> strike at %.2f m/s"
                    % (float(np.linalg.norm(to_target)), self.strike_speed))
            elif (self.engage_mode == "impact" and
                    float(np.linalg.norm(self.vision_rel)) <=
                    self.terminal_trigger_m):
                self._begin_terminal(now)
                v_cmd = self.terminal_cmd
            elif (self.engage_mode == "push" and
                  float(np.linalg.norm(to_target)) <=
                  self.push_engage_m):
                # Hand over from the position chase to the sustained push
                # while the light is still in front of the nose.
                self.flight_state = "push"
                self.pushing = True
                self.push_time = now
                self.push_contact_force = 0.0
                # Freeze the aim while the target still fits in the frame;
                # close-range measurements are unreliable. For a static map
                # target the map coordinates are exact.
                if self.target_motion == "static":
                    self.push_aim = self.target_pos.copy()
                else:
                    self.push_aim = self.est_pos.copy()
                self.get_logger().info(
                    "push engage: range=%.2f m, closing at %.2f m/s"
                    % (float(np.linalg.norm(to_target)), self.push_speed))
                v_cmd = np.zeros(3)
        elif self.last_los_world is not None:
            # Detection lost: point the camera down the last known line of
            # sight, sink back to the working altitude and sweep the nose
            # slowly until the light reappears.
            lost_s = now - self.vision_time
            search_altitude = (
                self.push_altitude_m
                if (self.engage_mode == "push" and
                    self.push_altitude_m > 0.0)
                else self.takeoff_altitude_m)
            search_yaw = math.atan2(self.last_los_world[1],
                                    self.last_los_world[0])
            if lost_s > 1.0:
                search_yaw += 0.6 * (lost_s - 1.0)
            self.yaw_cmd = self._blend_angle(
                self.yaw_cmd, search_yaw, min(1.0, dt / 0.5))
            if lost_s > 0.5:
                dz = search_altitude - float(platform_pos[2])
                v_cmd = np.array([
                    0.0, 0.0, float(np.clip(2.0 * dz, -0.4, 0.4))])
        else:
            # 还没看到目标：按地图上灯的已知位置巡航过去。起飞点（白色 H
            # 牌）离灯约 10 m，5.7 cm 的灯在那时不到一个像素，视觉要飞到
            # 2 m 左右才能锁定；在那之前就朝灯的地图坐标盲飞，速度随距离
            # 收窄（离灯 1 m 时减到零），一出现检测立刻切回视觉闭环。
            to_target = self.target_pos - platform_pos
            target_yaw = math.atan2(to_target[1], to_target[0])
            self.yaw_cmd = self._blend_angle(
                self.yaw_cmd, target_yaw, min(1.0, dt / 0.2))
            dist_xy = float(np.linalg.norm(to_target[:2]))
            # 制动剖面：v = 1.0 * (d - 1.4)。斜率 1.0 意味着 dv/dt = -v，
            # 4 m/s 时减速度 4 m/s²（约 22° 机头上仰），相机轴只抬到
            # 32°，灯仍在 35° 半视场内。2.6 m 视觉交接时速度约 1.2 m/s。
            speed = min(self.search_speed,
                        1.0 * max(dist_xy - 1.4, 0.0))
            # Ease the cruise in: zero acceleration at both ends, so the
            # nose comes down smoothly instead of stepping to the tilt
            # limit.
            ease = min(1.0, max(
                0.0, (now - self.seek_start) / self.seek_ramp_s))
            speed *= ease * ease * (3.0 - 2.0 * ease)
            v_cmd = np.array([
                math.cos(self.yaw_cmd) * speed,
                math.sin(self.yaw_cmd) * speed, 0.0])
            cruise_altitude = (
                self.push_altitude_m
                if (self.engage_mode == "push" and
                    self.push_altitude_m > 0.0)
                else self.takeoff_altitude_m)
            # 起飞点是一块离地面 1.73 m 的 H 牌，牌面只有约 0.35 m 宽。
            # 离起飞点 0.4 m 内保持起飞高度，0.4–1.0 m 之间平滑过渡到
            # 巡航高度（0.3–0.8 m 的过渡带）。
            start = (self.platform_start
                     if self.platform_start is not None else platform_pos)
            clear = float(np.clip(
                (float(np.linalg.norm(
                    (platform_pos - start)[:2])) - 0.3) / 0.5, 0.0, 1.0))
            cruise_altitude = (self.takeoff_altitude_m +
                               (cruise_altitude -
                                self.takeoff_altitude_m) * clear)
            # Gain 2.5 (tau 0.4 s) rather than 1.5: the last 20 cm of the
            # 1.4 m descent are an exponential tail; arriving high is what
            # makes the aircraft catch the head's top edge on the way down.
            v_cmd[2] = float(np.clip(
                2.5 * (cruise_altitude - float(platform_pos[2])) -
                0.6 * float(platform_vel[2]),
                -self.cruise_descent_mps, 0.6))
        return v_cmd

    # ---- Terminal (impact mode) -----------------------------------------
    def _terminal_aim(self):
        range_m = float(np.linalg.norm(self.vision_rel))
        time_to_go = min(range_m / max(self.impact_speed, 0.1), 0.5)
        if self.est_pos is not None:
            lead = self.target_vel_est * time_to_go
            # A real light does not move; the vision velocity estimate can
            # spike during the takeoff->seek pitch-over. Cap the lead.
            lead[2] = float(np.clip(lead[2], -0.25, 0.25))
            lead_norm = float(np.linalg.norm(lead))
            if lead_norm > 0.6:
                lead *= 0.6 / lead_norm
            aim = self.est_pos + lead
        else:
            aim = self.platform_pos + np.array(
                [self.vision_rel[0], -self.vision_rel[1],
                 -self.vision_rel[2]])
        return aim, range_m, time_to_go

    def _update_terminal_command(self):
        aim, range_m, time_to_go = self._terminal_aim()
        direction = aim - self.platform_pos
        distance = float(np.linalg.norm(direction))
        if distance < 1e-3:
            direction = np.array([1.0, 0.0, 0.0])
            distance = 1.0
        self.terminal_cmd = direction / distance * self.impact_speed
        # Follow the refreshed aim with a short ~0.1 s lag instead of
        # rotating the nose in one step.
        desired_yaw = math.atan2(self.terminal_cmd[1],
                                 self.terminal_cmd[0])
        self.yaw_cmd = self._blend_angle(self.yaw_cmd, desired_yaw, 0.06)
        return aim, range_m, time_to_go

    def _begin_terminal(self, now):
        self.terminal_start = now
        aim, range_m, time_to_go = self._update_terminal_command()
        self.get_logger().info(
            "terminal guidance: range=%.2f m, lead=%.2f s, aim=(%.2f, %.2f, "
            "%.2f), speed=%.1f m/s"
            % (range_m, time_to_go, aim[0], aim[1], aim[2],
               self.impact_speed))

    # ---- Outputs ---------------------------------------------------------
    def _publish_command(self, v_world, now, valid):
        """发布水平意图；生产封套保留生成时刻与实际用于旋转的姿态采样戳。

        本函数不触碰高度环或PX4。失效时输出全零并撤销，不沿用旧yaw速率；
        body FRD三维编码保留完整旋转分量，原生只执行旋回NED后的XY。
        """
        period = 1.0 / max(self.vision_control_hz, 1.0)
        if now - self.last_vision_control_time < period:
            return
        self.last_vision_control_time = now

        valid = bool(valid and self.control_enabled and not self.guidance_invalidated)
        command_rotation = self.platform_rot
        command_attitude_stamp = self.platform_attitude_stamp_ns if self.platform_input_mode == 'px4' else 0
        if self.platform_input_mode == 'px4':
            # 最后发布处重核授权和跟踪资格，避免直接调用绕过正常_tick路径。
            valid = (valid and self._native_navigation_current(now) and
                     self._tracking_policy_current(now) and self.platform_rot is not None and
                     np.shape(self.platform_rot) == (3,3) and np.all(np.isfinite(self.platform_rot)))
            if valid:
                # 原子封套提供原生确认的精确raw姿态，不再在降采样桥历史里猜共同样本。
                # 原生仍逐周期验证同一戳、来源/reset、原点与原始接收TTL。
                command_rotation = self._native_attitude_current(now)
                if command_rotation is None:
                    valid = False
                else:
                    command_attitude_stamp = self.native_attitude_snapshot[0][-1]*1000
            # Python min/max会把NaN裁成有限边界，接收端便看不见原错误。
            # 先校验航向内部量，任何非有限值都撤销，不能变成固定方向的满速偏航。
            yaw_values = (self.yaw_rate_gain,self.yaw_rate_ff,
                          self.yaw_cmd if self.yaw_cmd is not None else 0.,
                          self.last_published_yaw if self.last_published_yaw is not None else 0.)
            if not all(math.isfinite(value) for value in yaw_values):
                valid = False
            v_world = np.array([float(v_world[0]),float(v_world[1]),0.])
            speed = float(np.linalg.norm(v_world[:2]))
            if not math.isfinite(speed):
                valid = False
            elif speed > self.horizontal_max_speed_mps:
                v_world[:2] *= self.horizontal_max_speed_mps/speed

        # ENU航向逆时针为正，生产封套wz定义NED航向速率（顺时针为正），因此取负。
        # 这是期望量，不是PX4实测机体系角速度；倾斜时两者并不相等。
        # 旧V2仍作诊断兼容，生产执行只消费带独立姿态戳和原点的封套。
        self.yaw_rate_cmd = 0.0
        if self.yaw_cmd is not None and (valid or self.platform_input_mode != 'px4'):
            feedforward = 0.0
            if (self.last_published_yaw is not None and
                    now - self.last_published_yaw_time > 1e-4):
                dt = now - self.last_published_yaw_time
                delta = math.atan2(
                    math.sin(self.yaw_cmd - self.last_published_yaw),
                    math.cos(self.yaw_cmd - self.last_published_yaw))
                feedforward = max(-2.0, min(2.0, delta / dt))
                self.yaw_rate_ff += (
                    feedforward - self.yaw_rate_ff) * min(1.0, dt / 0.05)
            self.last_published_yaw = self.yaw_cmd
            self.last_published_yaw_time = now

            rot = self.platform_rot
            current_yaw = math.atan2(rot[1, 0], rot[0, 0])
            error = math.atan2(
                math.sin(self.yaw_cmd - current_yaw),
                math.cos(self.yaw_cmd - current_yaw))
            rate = self.yaw_rate_gain * error + self.yaw_rate_ff
            yaw_limit = self.horizontal_max_yaw_rate_rps if self.platform_input_mode == 'px4' else 2.
            self.yaw_rate_cmd = max(-yaw_limit, min(yaw_limit, rate))
        if self.vision_control_pub is None:
            return

        if not valid:
            # 失效不继续输出旧非零需求；尤其缺平台姿态时不能拿None/默认单位阵转换。
            v_world = [0.,0.,0.]
            self.yaw_rate_cmd = 0.

        # 撤销不需要坐标变换。缺姿态/姿态含NaN时，0乘无效矩阵也会产生NaN，
        # 所以直接构造全零；有效需求才消费实际PX4姿态。
        velocity_body_flu = (command_rotation.T @ np.asarray(v_world,dtype=float)
                             if valid else np.zeros(3))
        # 完整FRD三维表达与匹配姿态一同交原生。生产只先限世界水平向量模长，
        # 不能独立裁body各轴破坏完整旋转关系；旧simulation保留原逐轴限幅。
        vx_frd = float(velocity_body_flu[0])
        vy_frd = float(-velocity_body_flu[1])
        # ENU意图本身z=0，但倾斜机体系表达会有z分量，必须保留它再由原生完整旋转。
        # 丢掉body z会改变水平速度；封套不授予NED垂向控制，原生只取旋转后的XY。
        vz_frd = float(-velocity_body_flu[2])
        if self.platform_input_mode != 'px4' and self.max_vx_mps > 0.0:
            vx_frd = max(-self.max_vx_mps, min(self.max_vx_mps, vx_frd))
        if self.platform_input_mode != 'px4' and self.max_vy_mps > 0.0:
            vy_frd = max(-self.max_vy_mps, min(self.max_vy_mps, vy_frd))
        if self.platform_input_mode != 'px4' and self.max_vz_mps > 0.0:
            vz_frd = max(-self.max_vz_mps, min(self.max_vz_mps, vz_frd))
        msg = VelocityCommandMsg()
        msg.schema_version = 2
        msg.sequence = self.vision_control_seq
        msg.stamp = self.get_clock().now().to_msg()
        msg.reset_counter = self.reset_counter
        msg.source_id = self.vision_control_source_id
        msg.frame_id = "body_frd"
        msg.valid = bool(valid)
        msg.vx = vx_frd
        msg.vy = vy_frd
        msg.vz = vz_frd
        msg.wz = float(-self.yaw_rate_cmd)
        self.vision_control_seq += 1
        self.vision_control_pub.publish(msg)
        if self.platform_input_mode == 'px4':
            envelope = HorizontalVelocityCommand()
            envelope.schema_version = 1
            envelope.command = msg
            envelope.attitude_sample_stamp_ns = command_attitude_stamp
            envelope.origin_id = self.platform_history.identity[2] if self.platform_history.identity else ''
            self.horizontal_command_pub.publish(envelope)
            if valid:
                self.native_tracking_started = True

    def _publish_navigation(self, now, force_invalid=False):
        """原生到高后发布水平航点/定点；锁定后正常撤销航点，改由当前速度封套接续。

        valid=false是此航点通道的撤销，不是另一通道的失效。真正跟踪失效由封套
        valid=false和本任务锁存表达，原生仲裁不把它误认成普通模式切换。
        """
        if self.navigation_command_pub is None:
            return
        if now - self.last_navigation_time < 1.0 / self.navigation_command_hz:
            return
        self.last_navigation_time = now
        if self.platform_input_mode == 'px4':
            # 新PX4水平选择器对本消息只消费XY与yaw；z保持有限占位，无垂向权限。
            # frame_id绑定当前PX4完整原点，旧视觉估计器入口仍使用自己的frame合同。
            message = NavigationCommandMsg()
            message.schema_version = 1
            message.sequence = self.navigation_seq
            message.stamp = self.get_clock().now().to_msg()
            message.reset_counter = self.reset_counter
            message.source_id = self.vision_control_source_id
            origin = self.platform_history.identity[2] if self.platform_history.identity else ''
            message.frame_id = 'px4_local_ned:' + origin if origin else ''
            message.mode = NavigationCommandMsg.HOLD
            authorized = (self.control_enabled and not force_invalid and not self.guidance_invalidated and
                          not self.horizontal_fault_reason and self._native_navigation_current(now))
            tracking = self._tracking_policy_current(now) if authorized else False
            message.valid = bool(authorized and not tracking and not self.native_tracking_started and origin)
            message.position_valid = message.yaw_valid = message.sticks_valid = False
            message.position = [0.,0.,0.]
            message.sticks = [0.,0.,0.,0.]
            if message.valid and self.horizontal_navigation_mode == 'waypoint':
                if self._waypoint_policy_current():
                    message.mode = NavigationCommandMsg.CRUISE
                    message.position_valid = True
                    message.position = [float(self.navigation_waypoint_y),float(self.navigation_waypoint_x),0.]
                else:
                    # 未对齐或越界不能暗中降级成另一个坐标航点；当前点保持由原生捕获。
                    message.mode = NavigationCommandMsg.HOLD
            self.navigation_seq += 1
            self.navigation_command_pub.publish(message)
            return

        fresh_vision = (
            self.vision_rel is not None and
            now - self.vision_time <= self.command_timeout_s)
        waypoint_active = (
            self.control_enabled and not self.impacted and
            not fresh_vision and
            self.flight_state in ("takeoff", "seek"))
        origin = self.platform_history.identity[2] if self.platform_history.identity else ''
        waypoint_active = (waypoint_active and not force_invalid and
                          self.platform_input_mode == 'simulation' and
                          not self.guidance_invalidated and
                          self.waypoint_alignment_confirmed and
                          self.waypoint_alignment_origin_id == origin)

        msg = NavigationCommandMsg()
        msg.schema_version = 1
        msg.sequence = self.navigation_seq
        msg.stamp = self.get_clock().now().to_msg()
        msg.reset_counter = self.reset_counter
        msg.source_id = self.vision_control_source_id
        msg.frame_id = "fc01_vision_ned"
        msg.valid = bool(waypoint_active)

        # Sequence before vision lock:
        #   takeoff, below altitude -> CRUISE to the point above the pad
        #   takeoff, at altitude    -> HOLD there for takeoff_hover_s
        #   seek                    -> CRUISE to navigation_waypoint
        # 锁定后valid=false仅撤销意图；实际控制仲裁属于后续阶段。
        position_valid = False
        waypoint_x = float(self.target_x)
        waypoint_y = float(self.target_y)
        waypoint_z = float(self._push_altitude())
        if waypoint_active and self.flight_state == "takeoff":
            start = (self.platform_start if self.platform_start is not None
                     else self.platform_pos)
            waypoint_x = float(start[0])
            waypoint_y = float(start[1])
            waypoint_z = float(self.takeoff_altitude_m)
            if self.takeoff_complete_time is None:
                msg.mode = NavigationCommandMsg.CRUISE
                position_valid = True
            else:
                msg.mode = NavigationCommandMsg.HOLD
        elif waypoint_active:
            waypoint_x = float(self.navigation_waypoint_x)
            waypoint_y = float(self.navigation_waypoint_y)
            waypoint_z = (float(self.navigation_waypoint_z)
                          if self.navigation_waypoint_z > 0.0
                          else float(self._push_altitude()))
            msg.mode = NavigationCommandMsg.CRUISE
            position_valid = True
        else:
            msg.mode = NavigationCommandMsg.HOLD
        msg.position_valid = position_valid
        # World is ENU (x east, y north, z up); the message is NED.
        msg.position = [waypoint_y, waypoint_x, -waypoint_z]
        msg.yaw_valid = bool(waypoint_active and self.yaw_cmd is not None)
        if self.yaw_cmd is not None:
            # ENU yaw (CCW from +x/east) -> NED heading (CW from north).
            ned_yaw = math.pi / 2.0 - self.yaw_cmd
            msg.yaw = math.atan2(math.sin(ned_yaw), math.cos(ned_yaw))
        else:
            msg.yaw = 0.0
        msg.sticks_valid = False
        msg.sticks = [0.0, 0.0, 0.0, 0.0]
        self.navigation_seq += 1
        self.navigation_command_pub.publish(msg)

    def _publish_state(self, now):
        if self.state_publish_hz <= 0.0:
            return
        if now - self.last_state_time < 1.0 / self.state_publish_hz:
            return
        self.last_state_time = now
        self.state_pub.publish(String(data=self.flight_state))

    def _debug_log(self, now):
        if self.control_debug_hz <= 0.0:
            return
        if now - self.last_control_debug < 1.0 / self.control_debug_hz:
            return
        self.last_control_debug = now
        age = (now - self.vision_time
               if self.vision_rel is not None else -1.0)
        if self.push_aim is not None:
            push_range = float(np.linalg.norm(
                self.push_aim - self.platform_pos))
        else:
            push_range = -1.0
        rot = self.platform_rot
        roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -rot[2, 0]))))
        yaw_meas = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
        self.get_logger().info(
            "ctrl state=%s v_cmd=(%.2f, %.2f, %.2f) "
            "vel=(%.2f, %.2f, %.2f) pos=(%.2f, %.2f, %.2f) "
            "rpy=(%.0f, %.0f) yaw=%.0fdeg yaw_meas=%.0fdeg "
            "wz=%.2frad/s age=%.3fs "
            "pose_age=%.3fs tvel=(%.2f, %.2f, %.2f) "
            "push_range=%.2fm force=%.1fN"
            % (self.flight_state, self.last_v_cmd[0], self.last_v_cmd[1],
               self.last_v_cmd[2], self.platform_vel[0],
               self.platform_vel[1], self.platform_vel[2],
               self.platform_pos[0], self.platform_pos[1],
               self.platform_pos[2], roll, pitch,
               math.degrees(self.yaw_cmd) if self.yaw_cmd is not None else 0.0,
               yaw_meas, self.yaw_rate_cmd,
               age, now - self.platform_pose_time,
               self.target_vel_est[0], self.target_vel_est[1],
               self.target_vel_est[2], push_range, self.push_contact_force))


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # Humble信号处理器可能已关闭Context；正常launch退出不应再次shutdown报错。
        try_shutdown()


if __name__ == "__main__":
    main()

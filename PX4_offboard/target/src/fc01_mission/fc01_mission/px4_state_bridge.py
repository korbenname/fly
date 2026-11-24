#!/usr/bin/env python3
"""真实PX4 DDS -> 实际平台状态，只读桥；无/fmu发布器、无任务或解锁入口。

消息源保留各PX4 timestamp/timestamp_sample；ROS桥每次发布的是有效性快照，
不是用桥发布时刻覆盖采样时刻。未有angular_velocity DDS端点时明确缺测。
"""
import json
import math
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import try_shutdown
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleAngularVelocity
from jikl_fc01_system.msg import PlatformState
from .platform_feedback import PlatformFeedback, POSITION, VELOCITY, ATTITUDE, ANGULAR_VELOCITY


class PX4StateBridge(Node):
    """单线程spin持有三路反馈；正常销毁节点后关闭Context，不重装任何PX4环境。"""

    def __init__(self, **kwargs):
        super().__init__('fc01_px4_state_bridge', **kwargs)

    def configure(self):
        """main先取得Node所有权再接线；配置异常也能显式destroy_node。"""
        p = self.declare_parameter
        self.topics = {
            'local': str(p('local_topic', '/fmu/out/vehicle_local_position').value),
            'attitude': str(p('attitude_topic', '/fmu/out/vehicle_attitude').value),
            'angular_velocity': str(
                p('angular_velocity_topic', '/fmu/out/vehicle_angular_velocity').value
            ),
        }
        output = str(p('output_topic', '/fc01/platform/state').value)
        status = str(p('status_topic', '/fc01/platform/status').value)
        self.pose_topic = str(p('pose_topic', '/fc01/platform/pose').value)
        self.velocity_topic = str(p('velocity_topic', '/fc01/platform/velocity').value)
        self.omega_topic = str(p('omega_topic', '/fc01/platform/angular_velocity').value)
        self.publish_compatibility = bool(p('publish_compatibility_topics', True).value)
        source = str(p('source_id', 'px4_dds').value)
        # UUID标识当前桥会话；重启后mission不得复用上次锁定目标。
        self.feedback = PlatformFeedback(
            source + ':' + uuid.uuid4().hex,
            timeout_s=float(p('feedback_timeout_s', 0.25).value),
            alignment_s=float(p('feedback_alignment_s', 0.08).value),
            future_s=float(p('future_tolerance_s', 0.02).value),
            require_angular_velocity=bool(p('require_angular_velocity', True).value),
        )
        rate = float(p('publish_hz', 50.0).value)
        if not math.isfinite(rate) or not 1 <= rate <= 200:
            raise ValueError('publish_hz must be finite in [1,200]')
        # 校验一次最终解析名，再传原始名创建实体，避免二次remap越过只读边界。
        outputs = [output, status, self.pose_topic, self.velocity_topic, self.omega_topic]
        resolved = [self.resolve_topic_name(name) for name in outputs]
        if any(not name.startswith('/') or name.startswith('/fmu/') for name in resolved):
            raise ValueError('platform bridge output must not use /fmu endpoints')
        if len(set(resolved)) != len(resolved):
            raise ValueError('platform bridge outputs must be distinct')
        sensor = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PlatformState, output, sensor)
        self.status_pub = self.create_publisher(String, status, sensor)
        self.compatibility = {}
        if self.publish_compatibility:
            self.compatibility = {
                'pose': self.create_publisher(PoseStamped, self.pose_topic, sensor),
                'velocity': self.create_publisher(Vector3Stamped, self.velocity_topic, sensor),
                'omega': self.create_publisher(Vector3Stamped, self.omega_topic, sensor),
            }
        self.inputs = []
        for name, cls in (
            ('local', VehicleLocalPosition),
            ('attitude', VehicleAttitude),
            ('angular_velocity', VehicleAngularVelocity),
        ):
            self.inputs.append(
                self.create_subscription(
                    cls,
                    self.topics[name],
                    lambda msg, stream=name: self._receive(stream, msg),
                    sensor,
                )
            )
        self.timer = self.create_timer(1.0 / rate, self.publish_snapshot)
        self.get_logger().info('PX4 actual-state bridge; control_authority=false; output=' + output)

    def _endpoints(self, name):
        return self.get_publishers_info_by_topic(self.topics[name])

    def _graph_source(self, name):
        endpoints = self._endpoints(name)
        gid = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else ''
        return (gid if gid.strip('0') else ''), len(endpoints)

    def _count(self, name):
        return len(self._endpoints(name))

    def _receive(self, name, message):
        # Humble rclpy回调没有MessageInfo参数：只在图中唯一writer时接受，记录真实端点GID。
        # 这是来源独占的图证据，不等于逐包认证；图变更窗口仍由后续DDS/现场验收核对。
        gid, count = self._graph_source(name)
        getattr(self.feedback, name)(
            message, self.get_clock().now().nanoseconds, time.monotonic_ns(), gid, count
        )

    def publish_snapshot(self):
        host = self.get_clock().now().nanoseconds
        graph = {name: self._graph_source(name) for name in self.topics}
        # 在下一包到达前也观察writer替换/争抢，不能让旧会话缓存等待TTL才失效。
        for name, (gid, count) in graph.items():
            self.feedback._source(name, gid, count)
        values = self.feedback.snapshot(
            host, time.monotonic_ns(), {name: count for name, (_gid, count) in graph.items()}
        )
        message = PlatformState()
        message.stamp.sec = max(0, host) // 1_000_000_000
        message.stamp.nanosec = max(0, host) % 1_000_000_000
        for name, value in values.items():
            setattr(message, name, value)
        self.publisher.publish(message)
        diagnostic = dict(
            kind='px4_platform',
            control_authority=False,
            source_id=message.source_id,
            sequence=message.sequence,
            reset_counter=message.reset_counter,
            origin_id=message.origin_id,
            ready=message.ready,
            valid_fields=message.valid_fields,
            reason=message.reason,
            require_angular_velocity=message.require_angular_velocity,
            position_stamp_ns=message.position_stamp_ns,
            velocity_stamp_ns=message.velocity_stamp_ns,
            attitude_stamp_ns=message.attitude_stamp_ns,
            angular_velocity_stamp_ns=message.angular_velocity_stamp_ns,
            publisher_counts={name: self._count(name) for name in self.topics},
            coordinate_contract='PX4 local origin unchanged; ENU/FLU; xyzw',
        )
        self.status_pub.publish(String(data=json.dumps(diagnostic, ensure_ascii=False)))
        self._publish_compatibility(message)

    @staticmethod
    def _stamp(stamp, ns):
        stamp.sec, stamp.nanosec = divmod(int(ns), 1_000_000_000)

    def _publish_compatibility(self, state):
        """兼容Stamped只发布当前有效字段；生产mission仍以原子PlatformState撤销为准。"""
        if not self.compatibility:
            return
        if state.valid_fields & (POSITION | ATTITUDE) == POSITION | ATTITUDE:
            pose = PoseStamped()
            self._stamp(pose.header.stamp, state.position_stamp_ns)
            pose.header.frame_id = state.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state.position
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = state.attitude_xyzw
            self.compatibility['pose'].publish(pose)
        for flag, attr, key, frame in (
            (VELOCITY, 'velocity', 'velocity', state.frame_id),
            (ANGULAR_VELOCITY, 'angular_velocity', 'omega', state.body_frame_id),
        ):
            if state.valid_fields & flag:
                vector = Vector3Stamped()
                vector.header.frame_id = frame
                self._stamp(vector.header.stamp, getattr(state, attr + '_stamp_ns'))
                vector.vector.x, vector.vector.y, vector.vector.z = getattr(state, attr)
                self.compatibility[key].publish(vector)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PX4StateBridge()
        node.configure()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try_shutdown()


if __name__ == '__main__':
    main()

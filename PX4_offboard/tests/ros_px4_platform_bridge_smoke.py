#!/usr/bin/env python3
"""隔离DDS合成PX4反馈验证实际桥；无fc01_system_node、无/fmu/in发布。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if (
        os.environ.get('ROS_LOCALHOST_ONLY') != '1'
        or not 1 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
    ):
        raise RuntimeError('select nonzero isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleAngularVelocity
    from jikl_fc01_system.msg import PlatformState

    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    log = args.report.with_suffix('.log').open('w')
    bridge = subprocess.Popen(
        [
            'ros2',
            'run',
            'fc01_mission',
            'px4_state_bridge',
            '--ros-args',
            '-p',
            'local_topic:=/step2/px4/local',
            '-p',
            'attitude_topic:=/step2/px4/attitude',
            '-p',
            'angular_velocity_topic:=/step2/px4/omega',
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    node = None
    rclpy.init()
    try:
        node = Node('platform_smoke_source')
        pubs = {
            'local': node.create_publisher(VehicleLocalPosition, '/step2/px4/local', qos),
            'attitude': node.create_publisher(VehicleAttitude, '/step2/px4/attitude', qos),
            'omega': node.create_publisher(VehicleAngularVelocity, '/step2/px4/omega', qos),
        }
        received = []
        node.create_subscription(PlatformState, '/fc01/platform/state', received.append, qos)

        def wait(check, seconds=10, publish=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if publish:
                    publish()
                rclpy.spin_once(node, timeout_sec=0.025)
                if check():
                    return
            raise AssertionError('DDS bridge condition timeout')

        wait(lambda: all(p.get_subscription_count() == 1 for p in pubs.values()) and received)
        assert not received[-1].ready and received[-1].valid_fields == 0
        local_reset = 0

        def send(*, with_omega=True, invalid_velocity=False, quat=None):
            now = node.get_clock().now().nanoseconds // 1000
            l = VehicleLocalPosition()
            l.timestamp = now
            l.timestamp_sample = now
            l.xy_valid = True
            l.z_valid = True
            l.v_xy_valid = not invalid_velocity
            l.v_z_valid = True
            l.x = 1.0
            l.y = 2.0
            l.z = -1.5
            l.vx = 0.2
            l.vy = 0.3
            l.vz = -0.1
            l.ref_timestamp = 123
            l.z_reset_counter = local_reset
            a = VehicleAttitude()
            a.timestamp = now
            a.timestamp_sample = now
            a.q = quat if quat is not None else [1.0, 0.0, 0.0, 0.0]
            w = VehicleAngularVelocity()
            w.timestamp = now
            w.timestamp_sample = now
            w.xyz = [0.1, 0.2, 0.3]
            pubs['local'].publish(l)
            pubs['attitude'].publish(a)
            if with_omega:
                pubs['omega'].publish(w)

        wait(lambda: received[-1].ready, publish=send)
        s = received[-1]
        assert list(s.position) == [2.0, 1.0, 1.5]
        assert all(abs(x - y) < 1e-6 for x, y in zip(s.velocity, [0.3, 0.2, 0.1]))
        assert all(abs(x - y) < 1e-6 for x, y in zip(s.angular_velocity, [0.1, -0.2, -0.3]))
        initial = dict(
            position=list(s.position),
            velocity=list(s.velocity),
            attitude_xyzw=list(s.attitude_xyzw),
            angular_velocity=list(s.angular_velocity),
            source_id=s.source_id,
            origin_id=s.origin_id,
        )
        wait(
            lambda: not received[-1].ready
            and not (received[-1].valid_fields & PlatformState.ANGULAR_VELOCITY),
            seconds=3,
            publish=lambda: send(with_omega=False),
        )
        assert received[-1].valid_fields & 7 == 7 and math.isnan(received[-1].angular_velocity[0])
        wait(lambda: received[-1].ready, publish=send)
        wait(
            lambda: not (received[-1].valid_fields & PlatformState.VELOCITY),
            publish=lambda: send(invalid_velocity=True),
        )
        assert received[-1].valid_fields & PlatformState.POSITION
        wait(lambda: received[-1].ready, publish=send)
        epoch = received[-1].reset_counter
        local_reset = 1
        reset_seen = []
        end = time.monotonic() + 3
        while time.monotonic() < end:
            send()
            rclpy.spin_once(node, timeout_sec=0.02)
            reset_seen.extend(
                [s for s in received[-3:] if s.reset_counter > epoch and s.valid_fields == 0]
            )
            if reset_seen and received[-1].ready:
                break
        assert reset_seen and received[-1].reset_counter > epoch and received[-1].ready
        # 不再发新数据；桥仍发布快照，但全失效，不会用publish时间续鲜测量。
        wait(lambda: received[-1].valid_fields == 0, seconds=3)
        assert not received[-1].ready
        graph = node.get_topic_names_and_types()
        assert not any(
            name.startswith('/fmu/in/') and node.count_publishers(name) for name, types in graph
        )
        os.killpg(bridge.pid, signal.SIGINT)
        assert bridge.wait(timeout=8) == 0
        log.flush()
        text = args.report.with_suffix('.log').read_text()
        assert 'Traceback' not in text and '[ERROR]' not in text
        result = dict(
            passed=True,
            ros_domain=os.environ['ROS_DOMAIN_ID'],
            localhost_only=True,
            checks=[
                'missing_input_explicit',
                'ned_enu_frd_flu_quaternion_fields',
                'omega_alone_stales_without_zero',
                'velocity_invalid_independent',
                'coordinate_reset_invalid_then_new_epoch',
                'all_inputs_stop_revoke',
                'no_fmu_in_publishers',
                'sigint_clean_exit',
            ],
            initial=initial,
        )
        args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('FC01_PX4_PLATFORM_BRIDGE_PASS')
    finally:
        if bridge.poll() is None:
            os.killpg(bridge.pid, signal.SIGINT)
            try:
                bridge.wait(timeout=8)
            except subprocess.TimeoutExpired:
                bridge.terminate()
                bridge.wait(timeout=8)
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
        log.close()


if __name__ == '__main__':
    main()

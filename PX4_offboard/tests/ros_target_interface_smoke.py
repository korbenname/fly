#!/usr/bin/env python3
"""隔离Humble：真实Python发布→C++接收，及生产launch remap；无飞控控制节点。"""
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
    parser.add_argument('--prefix', type=Path, required=True, help='统一install根目录')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('必须显式设置隔离ROS_DOMAIN_ID和ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    from jikl_fc01_system.msg import (
        VelocityCommandV2,
        VelocityCommand,
        NavigationCommand,
        TargetObservation,
    )

    rclpy.init()
    node = Node('step1_wire_test')
    children = []
    logs = []
    samples = []
    report = {}
    mission = None
    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    obsq = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )

    def child(argv, label):
        log = (args.output / (label + '.log')).open('w')
        logs.append(log)
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p)
        return p

    def wait(predicate, seconds=12):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('ROS condition timeout')

    def cli(argv, name):
        p = subprocess.run(argv, capture_output=True, text=True, timeout=25)
        (args.output / name).write_text(p.stdout + p.stderr)
        if p.returncode:
            raise RuntimeError(str(argv) + ' ' + p.stderr)

    def stop(p):
        if p.poll() is None:
            # ros2 launch负责向子进程转发SIGINT；同时给进程组发信号会让子节点收两次。
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=8)

    try:
        status = node.create_subscription(
            String, '/fc01/interface/status', lambda m: samples.append(json.loads(m.data)), qos
        )
        binary = args.prefix / 'jikl_fc01_system/lib/jikl_fc01_system/fc01_target_interface_node'
        receiver = child([str(binary)], 'receiver')
        velocity = node.create_publisher(VelocityCommandV2, '/fc01/vision/velocity_command_v2', qos)
        nav = node.create_publisher(NavigationCommand, '/fc01/navigation/command', qos)
        target = node.create_publisher(TargetObservation, '/fc01/vision/target_observation', obsq)
        # 旧V1同话题类型不匹配；不得通过默认wz=0伪装为V2。
        # FastDDS可直接拒绝同participant内同名不同类型；拒绝本身就是隔离证据。
        try:
            conflicting = node.create_publisher(
                VelocityCommand, '/fc01/vision/velocity_command_v2', qos
            )
            node.destroy_publisher(conflicting)
            report['legacy_conflicting_topic'] = 'created; independent type match checked below'
        except rclpy._rclpy_pybind11.RCLError as error:
            report['legacy_conflicting_topic'] = str(error)
        legacy = node.create_publisher(VelocityCommand, '/fc01/vision/velocity_command', qos)
        wait(
            lambda: velocity.get_subscription_count() == 1
            and nav.get_subscription_count() == 1
            and target.get_subscription_count() == 1
            and samples
        )
        assert legacy.get_subscription_count() == 0

        def publish(pub, msg, key, check):
            pub.publish(msg)
            wait(lambda: samples[-1].get(key) is not None and check(samples[-1][key]))
            assert samples[-1]['control_authority'] is False
            return samples[-1][key]

        v = VelocityCommandV2()
        v.sequence = 19
        v.stamp.sec = 123
        v.stamp.nanosec = 456
        v.reset_counter = 4
        v.source_id = 'mission_wire'
        v.frame_id = 'body_frd'
        v.valid = True
        v.vx = 0.25
        v.vy = -0.125
        v.vz = 0.375
        v.wz = -0.5
        got = publish(velocity, v, 'velocity', lambda x: x['sequence'] == 19)
        for name in (
            'schema_version',
            'sequence',
            'reset_counter',
            'source_id',
            'frame_id',
            'valid',
            'vx',
            'vy',
            'vz',
            'wz',
        ):
            assert got[name] == getattr(v, name), (name, got)
        assert got['stamp_ns'] == 123000000456
        v.sequence = 20
        v.valid = False
        v.wz = 0.5
        publish(
            velocity,
            v,
            'velocity',
            lambda x: x['sequence'] == 20 and x['valid'] is False and x['wz'] == 0.5,
        )
        v.sequence = 21
        v.vx = float('nan')
        v.wz = 0.0
        publish(
            velocity,
            v,
            'velocity',
            lambda x: x['sequence'] == 21 and x['vx'] is None and x['wz'] == 0.0,
        )
        n = NavigationCommand()
        n.sequence = 23
        n.mode = n.CRUISE
        n.valid = True
        n.position_valid = True
        n.yaw_valid = True
        n.sticks_valid = True
        n.source_id = 'waypoint_wire'
        n.frame_id = 'fc01_vision_ned'
        n.position = [1.0, 2.0, -1.5]
        n.yaw = -0.75
        n.sticks = [0.1, 0.2, 0.3, 0.4]
        got = publish(nav, n, 'navigation', lambda x: x['sequence'] == 23)
        for name in ('mode', 'valid', 'position_valid', 'yaw_valid', 'sticks_valid', 'yaw'):
            assert got[name] == getattr(n, name)
        assert got['position'] == list(n.position) and got['sticks'] == list(n.sticks)
        t = TargetObservation()
        t.sequence = 29
        t.source_id = 'camera_wire'
        t.frame_id = 'body_frd'
        t.body_frame_id = 'body_frd'
        t.valid_fields = t.POSITION | t.VELOCITY
        t.initialized = True
        t.tracking = True
        t.calibration_valid = False
        t.position = [3.0, -0.5, 0.25]
        t.velocity = [0.1, 0.2, 0.3]
        t.attitude = [1.0, 0.0, 0.0, 0.0]
        t.angular_velocity_source = 'fixture_only'
        t.angular_velocity = [0.3, 0.2, 0.1]
        t.pose_covariance = [i / 8 for i in range(36)]
        t.twist_covariance = [i / 16 for i in range(36)]
        t.quality = 0.25
        got = publish(target, t, 'target', lambda x: x['sequence'] == 29)
        for name in (
            'position',
            'velocity',
            'attitude',
            'angular_velocity',
            'pose_covariance',
            'twist_covariance',
        ):
            assert got[name] == list(getattr(t, name))
        assert (
            got['angular_velocity_source'] == t.angular_velocity_source
            and got['quality'] == t.quality
        )
        assert got['calibration_valid'] is False
        report['wire_fields'] = samples[-1]
        endpoints = node.get_subscriptions_info_by_topic('/fc01/vision/velocity_command_v2')
        assert len(endpoints) == 1
        # Humble/FastDDS远端discovery的history/depth可能UNKNOWN/0，不能冒称可从图证明。
        assert endpoints[0].qos_profile.durability == DurabilityPolicy.VOLATILE
        assert samples[-1]['velocity_qos'] == {'depth': 1, 'keep_last': True, 'volatile': True}
        report['remote_depth_observed'] = endpoints[0].qos_profile.depth
        report['legacy_v1_subscription_matches'] = legacy.get_subscription_count()
        cli(
            ['ros2', 'interface', 'show', 'jikl_fc01_system/msg/VelocityCommandV2'],
            'interface-v2.txt',
        )
        cli(
            ['ros2', 'interface', 'show', 'jikl_fc01_system/msg/TargetObservation'],
            'interface-target.txt',
        )
        cli(
            ['ros2', 'topic', 'info', '-v', '/fc01/vision/velocity_command_v2'], 'topic-default.txt'
        )
        # 走真实C++图像节点和适配器，不只给接收器发送手工拼装的目标消息。
        # 黑图表示未检测到目标；红圆是已知内参/直径的合成夹具，不代表实物精度。
        from sensor_msgs.msg import Image
        import numpy as np

        image_binary = args.prefix / 'fc01_vision/lib/fc01_vision/vision_image_node'
        image_node = child(
            [
                str(image_binary),
                '--ros-args',
                '-p',
                'use_camera_info:=false',
                '-p',
                'image_topic:=/step1/image',
                '-p',
                'source_id:=synthetic_image',
                '-p',
                'frame_width:=160',
                '-p',
                'frame_height:=120',
                '-p',
                'camera_fx:=100.0',
                '-p',
                'camera_fy:=100.0',
                '-p',
                'camera_cx:=80.0',
                '-p',
                'camera_cy:=60.0',
                '-p',
                'target_shape:=circle',
                '-p',
                'target_diameter_m:=0.2',
                '-p', 'calibration_confirmed:=true',
                '-p',
                'image_rectified_confirmed:=true',
                '-p', 'calibration_id:=synthetic-fixture',
                '-p', 'extrinsics_confirmed:=true',
                '-p', 'extrinsics_id:=synthetic-identity',
                '-p', 'target_model_confirmed:=true',
                '-p', 'target_model_id:=synthetic-circle',
                '-p', 'capture_latency_confirmed:=true',
                '-p', 'capture_latency_bound_s:=0.01',
                '-p', 'stale_timeout_s:=0.5',
                '-p', 'max_dt_s:=0.5',
            ],
            'image-node',
        )
        image_pub = node.create_publisher(Image, '/step1/image', qos)
        wait(
            lambda: image_pub.get_subscription_count() == 1
            and target.get_subscription_count() == 1
            and node.count_publishers('/fc01/vision/target_observation') == 2
        )

        def image_result(image, tracking):
            # 每次只发一个当前源帧；反复发布相同header在第2步必须视作重复无效，
            # 不能通过测试夹具续鲜。160x120避免大图分片丢包掩盖接口语义。
            image.header.stamp = node.get_clock().now().to_msg()
            stamp = image.header.stamp.sec*1_000_000_000+image.header.stamp.nanosec
            image_pub.publish(image)
            wait(lambda: samples[-1].get('target') is not None
                 and samples[-1]['target']['stamp_ns'] == stamp
                 and samples[-1]['target']['source_id'].startswith('synthetic_image@')
                 and samples[-1]['target']['tracking'] is tracking)
            return samples[-1]['target']

        image = Image()
        image.height = 120
        image.width = 160
        image.encoding = 'bgr8'
        image.step = 160 * 3
        image.data = bytes(160 * 120 * 3)
        got = image_result(image, False)
        assert got['valid_fields'] == 0
        # 不依赖cv2 Python扩展，用numpy生成中心红圆；实际检测仍由C++OpenCV执行。
        pixels = np.zeros((120, 160, 3), dtype=np.uint8)
        yy, xx = np.ogrid[:120, :160]
        pixels[(xx - 80) ** 2 + (yy - 60) ** 2 <= 10**2, 2] = 255
        image.data = pixels.tobytes()
        got = image_result(image, True)
        assert got['frame_id'] == 'body_frd' and got['calibration_valid']
        assert got['valid_fields'] & TargetObservation.POSITION
        # 第一幅真实检测没有可差分前帧；零速度不能伪装成已观测速度。
        assert not got['valid_fields'] & TargetObservation.VELOCITY
        assert not got['valid_fields'] & (TargetObservation.ATTITUDE | TargetObservation.QUALITY)
        assert got['position'][0] > 0
        assert abs(got['position'][1]) < 0.02 and abs(got['position'][2]) < 0.02
        report['actual_cpp_image_adapter'] = got
        got = image_result(image, True)
        assert got['valid_fields'] & TargetObservation.VELOCITY
        report['actual_cpp_image_second_velocity_valid'] = True
        stop(image_node)
        stop(receiver)
        # 真正导入mission业务实现，给显式平台夹具，只检查其发布函数的坐标/类型，非飞行仿真。
        from fc01_mission.mission_node import MissionNode

        mission = MissionNode()
        # 本段只核对原发布方法的三维线缆变换，明确使用simulation语义；
        # 实机水平/垂直权限由独立native gate smoke验证，不在这里伪造授权。
        mission.platform_input_mode = 'simulation'
        mission.control_timer.cancel()
        mission.platform_rot = __import__('numpy').eye(3)
        mission.yaw_cmd = 0.0
        mission.last_published_yaw = None
        mission.vision_control_pub = mission.create_publisher(
            VelocityCommandV2, '/fc01/vision/velocity_command_v2', qos
        )
        receiver = child([str(binary)], 'receiver-mission')
        wait(lambda: mission.vision_control_pub.get_subscription_count() == 1)
        # 默认关闭任务时，即使调用者传valid=True也必须撤销、输出零需求。
        mission.vision_control_seq = 100
        mission.last_vision_control_time = -100.0
        assert mission.control_enabled is False
        mission._publish_command([0.2, -0.3, 0.1], 99.0, True)
        wait(lambda: samples[-1].get('velocity') is not None
             and samples[-1]['velocity']['sequence'] == 100)
        disabled = samples[-1]['velocity']
        assert disabled['valid'] is False
        assert all(disabled[k] == 0.0 for k in ('vx','vy','vz','wz'))
        report['actual_mission_default_disabled'] = disabled
        # 只给此发布函数的坐标夹具打开本地对象开关；不启动飞控/Runtime。
        mission.control_enabled = True
        mission.vision_control_seq = 101
        mission.last_vision_control_time = -100.0
        mission._publish_command([0.2, -0.3, 0.1], 100.0, True)
        wait(
            lambda: samples[-1].get('velocity') is not None
            and samples[-1]['velocity']['sequence'] == 101
        )
        got = samples[-1]['velocity']
        assert (
            got['schema_version'] == 2
            and got['vx'] == 0.2
            and got['vy'] == 0.3
            and got['vz'] == -0.1
        )
        report['actual_mission_publisher'] = got
        mission.yaw_rate_gain = 1.0
        for sequence, yaw in ((102, 0.5), (103, -0.5)):
            mission.vision_control_seq = sequence
            mission.last_vision_control_time = -100.0
            mission.last_published_yaw = None
            mission.yaw_rate_ff = 0.0
            mission.yaw_cmd = yaw
            mission._publish_command([0.0, 0.0, 0.0], 101.0, True)
            wait(lambda: samples[-1]['velocity']['sequence'] == sequence)
            assert samples[-1]['velocity']['wz'] == -yaw
        report['actual_mission_wz_sign'] = 'ENU positive yaw -> negative FRD wz, both signs checked'
        mission.destroy_node()
        mission = None
        stop(receiver)
        # 生产launch真实执行，自定义三个topic检查映射，不在默认旧topic碰巧接通。
        production_velocity = []
        production_navigation = []
        node.create_subscription(
            VelocityCommandV2, '/step1/remapped_velocity', production_velocity.append, qos
        )
        node.create_subscription(
            NavigationCommand, '/step1/remapped_navigation', production_navigation.append, qos
        )
        # 明确使用不存在的设备编号，不打开开发机真实摄像头；仍运行生产相机发布端。
        camera_fixture = (args.output / 'no-camera.yaml').resolve()
        camera_fixture.write_text('/**:\n  ros__parameters:\n    device_index: 99999\n')
        launch = child(
            [
                'ros2',
                'launch',
                'fc01_bringup',
                'robot.launch.py',
                'start_camera:=true',
                'vision_config:=' + str(camera_fixture),
                'control_enabled:=false',
                'velocity_topic:=/step1/remapped_velocity',
                'navigation_topic:=/step1/remapped_navigation',
                'target_topic:=/step1/remapped_target',
                'platform_state_topic:=/step1/remapped_platform',
            ],
            'production-launch',
        )
        wait(
            lambda: node.count_publishers('/step1/remapped_velocity') == 1
            and node.count_subscribers('/step1/remapped_velocity') == 2,
            20,
        )
        assert (
            node.count_publishers('/visioncontrol') == 0
            and node.count_publishers('/navigation_command') == 0
        )
        wait(lambda: node.count_subscribers('/step1/remapped_target') == 2)
        wait(lambda: node.count_publishers('/step1/remapped_target') == 1)
        assert node.count_publishers('/fc01/vision/state') == 0
        # 缺实际反馈仍须明确发布invalid撤销；仅收到桥的invalid快照不能算ready。
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        wait(lambda: production_velocity and production_navigation)
        assert all(not m.valid for m in production_velocity + production_navigation)
        assert all(m.vx == m.vy == m.vz == m.wz == 0.0 for m in production_velocity)
        wait(lambda: node.count_publishers('/step1/remapped_platform') == 1
             and node.count_subscribers('/step1/remapped_platform') == 1)
        assert node.count_publishers('/fc01/platform/state') == 0
        assert all(node.count_subscribers(t) == 0 for t in (
            '/sim/platform_pose', '/sim/platform_velocity', '/sim/platform_angular_velocity'))
        report['production_px4_bridge_started'] = True
        fmu_publishers = {
            name: node.count_publishers(name)
            for name, _ in node.get_topic_names_and_types() if name.startswith('/fmu/')
        }
        assert not any(fmu_publishers.values()), fmu_publishers
        report['fmu_publisher_counts'] = fmu_publishers
        report['disabled_production_commands_valid'] = False
        report['disabled_production_received'] = {
            'velocity': len(production_velocity),
            'navigation': len(production_navigation),
        }
        for topic in (
            '/step1/remapped_velocity',
            '/step1/remapped_navigation',
            '/step1/remapped_target',
            '/step1/remapped_platform',
        ):
            cli(['ros2', 'topic', 'info', '-v', topic], topic.rsplit('/', 1)[1] + '.txt')
        graph = node.get_node_names()
        assert 'fc01_system' not in graph and 'fc01_system_node' not in graph
        stop(launch)
        for log in logs:
            log.flush()
        launch_log = (args.output / 'production-launch.log').read_text()
        assert '[ERROR]' not in launch_log and 'Traceback' not in launch_log, launch_log
        report['production_launch_clean_shutdown'] = True
        # 双跳remap只应用一次；禁止把已解析名字再次传给实体创建而落入/fmu。
        remapped = child(
            [
                str(binary),
                '--ros-args',
                '-r',
                '/fc01/interface/status:=/step1/status',
                '-r',
                '/step1/status:=/fmu/in/vehicle_command',
            ],
            'double-remap',
        )
        wait(lambda: node.count_publishers('/step1/status') == 1)
        assert node.count_publishers('/fmu/in/vehicle_command') == 0
        stop(remapped)
        for rules in (
            ['-r', '/fc01/interface/status:=/fmu/in/vehicle_command'],
            ['-r', '/fc01/interface/status:=/fc01/vision/velocity_command_v2'],
        ):
            rejected = child([str(binary), '--ros-args'] + rules, 'rejected-' + str(len(children)))
            assert rejected.wait(timeout=10) == 1
        report['remap_boundary'] = (
            'double-hop applies once; direct PX4 and resolved collision rejected'
        )
        report.update(
            production_launch_remap=True,
            flight_controller_started=False,
            ros_domain=os.environ['ROS_DOMAIN_ID'],
            result='PASS',
        )
        (args.output / 'RESULT.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n'
        )
        print('FC01_STEP1_ROS_INTERFACE_PASS')
    finally:
        for p in reversed(children):
            stop(p)
        if mission is not None:
            mission.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()

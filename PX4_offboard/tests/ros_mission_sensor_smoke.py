#!/usr/bin/env python3
"""隔离Humble完整传感器链：合成PX4 DDS/图像→实际C++检测/桥→实际Mission。

只启动传感器和制导，不启动原生飞控节点；默认control_enabled=false。
几何夹具的数值不是现场相机标定，测试结果不代表实物跟踪性能。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('必须设置隔离ROS_DOMAIN_ID和ROS_LOCALHOST_ONLY=1')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleAngularVelocity
    from jikl_fc01_system.msg import TargetObservation, VelocityCommandV2
    from fc01_mission.mission_node import MissionNode
    from fc01_mission.sensor_contract import rotation_xyzw

    args.output.mkdir(parents=True, exist_ok=True)
    children, logs, checks = [], [], []
    rclpy.init(args=['--ros-args', '-p', 'camera_extrinsics_confirmed:=true',
                    '-p', 'camera_extrinsics_id:=synthetic-lever-arm',
                    '-p', 'camera_pos_x:=0.1', '-p', 'camera_pos_y:=0.2',
                    '-p', 'camera_pos_z:=0.3',
                    '-p', 'vision_control_topic:=/fc01/vision/velocity_command_v2',
                    '-p', 'navigation_command_topic:=/fc01/navigation/command'])
    node = Node('step2_sensor_fixture')
    mission = MissionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(mission)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    local_pub = node.create_publisher(VehicleLocalPosition, '/step2/local', qos)
    att_pub = node.create_publisher(VehicleAttitude, '/step2/attitude', qos)
    omega_pub = node.create_publisher(VehicleAngularVelocity, '/step2/omega', qos)
    image_pub = node.create_publisher(Image, '/step2/image', 1)
    status, observations, commands = [], [], []
    node.create_subscription(String, '/fc01/mission/sensors/status',
                             lambda m: status.append(json.loads(m.data)), 10)
    node.create_subscription(TargetObservation, '/fc01/vision/target_observation',
                             observations.append, qos)
    node.create_subscription(VelocityCommandV2, '/fc01/vision/velocity_command_v2',
                             commands.append, 10)
    config = dict(omega=True, velocity_valid=True, z_reset=0, quat_reset=0,
                  q=[1., 0., 0., 0.], local=True)
    last_send = 0.

    def child(binary, params, label):
        log = (args.output/(label+'.log')).open('w')
        logs.append(log)
        argv = [str(binary), '--ros-args']
        for key, value in params.items():
            argv += ['-p', key+':='+str(value).lower() if isinstance(value, bool)
                     else key+':='+str(value)]
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p)
        return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            assert p.wait(timeout=8) == 0

    def send_platform():
        nonlocal last_send
        now = time.monotonic()
        if now-last_send < .02:
            return
        last_send = now
        stamp = node.get_clock().now().nanoseconds//1000
        l = VehicleLocalPosition()
        l.timestamp = l.timestamp_sample = stamp
        l.xy_valid = l.z_valid = l.v_z_valid = True
        l.v_xy_valid = config['velocity_valid']
        l.x, l.y, l.z = 10., 20., -3.
        l.vx, l.vy, l.vz = .2, .3, -.1
        l.ref_timestamp = 123
        l.z_reset_counter = config['z_reset']
        a = VehicleAttitude()
        a.timestamp = a.timestamp_sample = stamp
        a.q = config['q']
        a.quat_reset_counter = config['quat_reset']
        w = VehicleAngularVelocity()
        w.timestamp = w.timestamp_sample = stamp
        w.xyz = [.1, .2, .3]
        if config['local']:
            local_pub.publish(l)
        att_pub.publish(a)
        if config['omega']:
            omega_pub.publish(w)

    def spin(seconds=.1):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            send_platform()
            executor.spin_once(timeout_sec=.003)

    def wait(predicate, timeout=8):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            spin(.02)
            if predicate():
                return
        raise AssertionError('condition timeout; '+repr({
            'platform':mission.sensor_reason,'target':mission.target_reason,
            'lock':mission.target_consecutive,'status':status[-1:] }))

    def image(data=None, stamp=None):
        msg = Image()
        msg.width, msg.height, msg.encoding, msg.step = 160, 120, 'bgr8', 480
        msg.header.stamp = stamp or node.get_clock().now().to_msg()
        if data is None:
            pixels = np.zeros((120,160,3), np.uint8)
            yy, xx = np.ogrid[:120,:160]
            pixels[(xx-80)**2+(yy-60)**2<=10**2, 2] = 255
            msg.data = pixels.tobytes()
        else:
            msg.data = data
        image_pub.publish(msg)
        return msg

    target_pub = None
    seq = 0
    target_reset = 0
    target_source = 'exact-geometry-fixture'

    def target(position=(2.,0.,0.), **changes):
        nonlocal seq
        seq += 1
        m = TargetObservation()
        m.sequence = seq
        m.reset_counter = target_reset
        m.source_id = target_source
        m.frame_id = m.body_frame_id = 'body_frd'
        m.stamp = node.get_clock().now().to_msg()
        m.initialized = m.tracking = m.calibration_valid = True
        m.valid_fields = m.POSITION
        m.position = list(position)
        for k, v in changes.items():
            setattr(m, k, v)
        target_pub.publish(m)
        return m

    def lock(position=(2.,0.,0.)):
        for _ in range(5):
            target(position)
            spin(.045)
            if mission.target_locked:
                break
        assert mission.target_locked, mission.target_reason

    try:
        bridge = child(args.prefix/'fc01_mission/lib/fc01_mission/px4_state_bridge', {
            'local_topic':'/step2/local','attitude_topic':'/step2/attitude',
            'angular_velocity_topic':'/step2/omega'}, 'bridge')
        camera = child(args.prefix/'fc01_vision/lib/fc01_vision/vision_image_node', {
            'use_camera_info':False,'image_topic':'/step2/image',
            'frame_width':160,'frame_height':120,'camera_fx':100.,'camera_fy':100.,
            'camera_cx':80.,'camera_cy':60.,'target_shape':'circle','target_diameter_m':.2,
            'calibration_confirmed':True,'calibration_id':'synthetic-intrinsic',
            'extrinsics_confirmed':True,'extrinsics_id':'synthetic-rotation',
            'target_model_confirmed':True,'target_model_id':'synthetic-circle',
            'capture_latency_confirmed':True,'capture_latency_bound_s':.01,
            'image_rectified_confirmed':True}, 'camera')
        wait(lambda: image_pub.get_subscription_count()==1 and mission.sensor_reason=='OK')
        spin(.08)
        np.testing.assert_allclose(mission.platform_pos,[20.,10.,3.])
        assert mission.platform_omega is not None
        checks.append('real_dds_bridge_absolute_origin_and_independent_fields')
        for count in range(1,4):
            previous = mission.target_consecutive
            image()
            wait(lambda:mission.target_consecutive>previous)
            assert mission.target_locked == (count>=3)
        assert mission.est_pos is not None
        checks.append('real_cpp_image_to_actual_mission_three_new_frames_lock')
        last = image()
        spin(.04)
        image_pub.publish(last)
        wait(lambda:not mission.target_locked)
        assert mission.vision_rel is None
        checks.append('repeated_raw_image_revokes_not_renews')
        old = node.get_clock().now().to_msg()
        old.sec -= 2
        image(stamp=old)
        spin(.08)
        assert not mission.target_locked
        image(data=b'')
        spin(.08)
        assert not mission.target_locked
        checks.append('old_and_empty_images_explicit_invalid')
        stop(camera)
        wait(lambda:node.count_publishers('/fc01/vision/target_observation')==0)
        target_pub = node.create_publisher(TargetObservation,'/fc01/vision/target_observation',qos)
        wait(lambda:target_pub.get_subscription_count()>=2)
        spin(.1)
        # PX4 identity FRD/NED corresponds to +90deg body-FLU in ENU.
        lock()
        expected = np.array([20.,10.,3.])+rotation_xyzw([0.,0.,math.sqrt(.5),math.sqrt(.5)]) @ np.array([2.1,.2,.3])
        np.testing.assert_allclose(mission.est_pos,expected,atol=1e-6)
        for side in (-.5,.5):
            sample=target((2.,side,0.))
            wait(lambda:mission.last_target_sample==sample.stamp.sec*10**9+sample.stamp.nanosec)
            relative_flu=np.array([2.,-side,0.])+mission.camera_pos
            np.testing.assert_allclose(mission.est_pos,mission.platform_pos+mission.platform_rot@relative_flu,atol=1e-6)
        checks.append('yaw90_left_right_lever_arm_once_and_absolute_waypoint_origin')
        # NED yaw90 produces identity in ENU; compose body roll30 for tilted test.
        qz=math.sqrt(.5)
        config['q']=[qz*math.cos(math.pi/12),qz*math.sin(math.pi/12),qz*math.sin(math.pi/12),qz*math.cos(math.pi/12)]
        spin(.12)
        sample=target((2.,.4,-.3))
        wait(lambda:mission.last_target_sample==sample.stamp.sec*10**9+sample.stamp.nanosec)
        np.testing.assert_allclose(mission.est_pos,mission.platform_pos+mission.platform_rot@np.array([2.1,-.2,.6]),atol=1e-6)
        checks.append('tilted_quaternion_wxyz_to_xyzw_geometry')
        previous=mission.vision_time
        target_pub.publish(sample)
        spin(.06)
        assert mission.vision_time==previous
        spin(.32)
        assert not mission.target_locked and mission.est_pos is None
        checks.append('duplicate_observation_and_disconnect_do_not_renew')
        lock()
        target(tracking=False)
        wait(lambda:not mission.target_locked)
        lock()
        target(position=(float('nan'),0.,0.))
        wait(lambda:not mission.target_locked)
        lock()
        target(calibration_valid=False)
        wait(lambda:not mission.target_locked)
        checks.append('tracking_nan_and_calibration_invalid_revoke')
        lock()
        config['omega']=False
        wait(lambda:mission.platform_pos is None and not mission.target_locked)
        assert mission.platform_omega is None
        config['omega']=True
        wait(lambda:mission.platform_pos is not None)
        lock()
        config['velocity_valid']=False
        wait(lambda:mission.platform_vel is None and not mission.target_locked)
        config['velocity_valid']=True
        wait(lambda:mission.platform_pos is not None)
        config['local']=False
        wait(lambda:mission.platform_pos is None)
        config['local']=True
        wait(lambda:mission.platform_pos is not None)
        checks.append('omega_velocity_and_local_alone_invalid_revoke_target')
        for name in ('z_reset','quat_reset'):
            lock()
            generation=mission.platform_history.generation
            config[name]+=1
            wait(lambda:mission.platform_history.generation>generation)
            assert not mission.target_locked
            wait(lambda:mission.platform_pos is not None)
        checks.append('local_and_attitude_coordinate_resets_clear_lock')
        lock()
        target_source='restarted-camera'
        target()
        wait(lambda:not mission.target_locked)
        lock()
        checks.append('camera_session_restart_requires_new_lock')
        mission.terminal_start=time.monotonic()
        mission.terminal_cmd=np.array([1.2,-.3,.1])
        target(tracking=False)
        wait(lambda:mission.guidance_invalidated)
        assert mission.terminal_start is None and np.all(mission.terminal_cmd==0.)
        lock()
        assert mission.guidance_invalidated
        demand,valid=mission._update_mission(time.monotonic(),.01)
        assert not valid and np.all(demand==0.)
        checks.append('cached_terminal_demand_latched_off_after_invalid_target_and_recovery')
        spin(.25)
        assert commands and all(not m.valid and m.vx==m.vy==m.vz==m.wz==0. for m in commands)
        assert status and 'WAYPOINT_ORIGIN_UNCONFIRMED' in status[-1]['missing']
        assert not any(t.startswith('/fmu/in/') and node.count_publishers(t)
                       for t,_ in node.get_topic_names_and_types())
        assert 'fc01_system' not in node.get_node_names()
        checks.append('default_disabled_zero_invalid_commands_and_unconfirmed_origin_status')
        result={'result':'PASS','checks':checks,'environment':'Ubuntu22.04 x86_64 Humble synthetic DDS and images',
                'hardware_flight_test':False,'source_stamp_preserved':True,
                'final_sensor_status':status[-1]}
        (args.output/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('FC01_STEP2_MISSION_SENSOR_PASS '+str(len(checks))+' named checks')
    finally:
        for p in reversed(children):
            stop(p)
        executor.remove_node(mission)
        executor.remove_node(node)
        mission.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
        for log in logs:
            log.close()


if __name__=='__main__':
    main()

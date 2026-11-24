#!/usr/bin/env python3
"""隔离Humble：合成Image→实际C++检测→实际Mission→真实DDS水平意图。

不启动原生控制器，不发布/fmu/in。NativeFlight/Platform均显式为夹具，
标定和相机模型仅用于合成几何。资源数据只属于x86图像ROS入口，不是USB/ARM64实测。
"""
import argparse
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--benchmark-seconds', type=float, default=4.)
    args = parser.parse_args()
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not 1 <= domain <= 232:
        raise RuntimeError('use explicit isolated localhost ROS domain1..232')
    if not 1. <= args.benchmark_seconds <= 30.:
        raise ValueError('benchmark duration must be1..30s')
    import numpy as np
    import rclpy
    import yaml
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from jikl_fc01_system.msg import (PlatformState, NativeFlightState, NativeFlightSnapshot, TargetObservation,
                                      HorizontalVelocityCommand, NavigationCommand)
    from fc01_mission.mission_node import MissionNode

    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    config_path = root/'target/src/fc01_vision/config/vision.yaml'
    vision_params = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
    width, height, rate = (int(vision_params[k]) for k in ('frame_width', 'frame_height', 'frame_fps'))
    assert (width, height, rate) == (640, 480, 120), 'recheck fixture if production image profile changed'
    camera_binary = args.prefix/'fc01_vision/lib/fc01_vision/vision_image_node'
    if not camera_binary.is_file():
        raise FileNotFoundError(camera_binary)
    # raw BGR8图像约0.88MiB，大于默认FastDDS约0.5MiB共享段。仅本隔离夹具
    # 启用有界16MiB图像段；生产USB CameraSource进程内检测不需要此文件。
    profile = root/'tests/fixtures/step5_camera_local_shm.xml'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in ('ROS_DISCOVERY_SERVER','CYCLONEDDS_URI','FASTDDS_DEFAULT_PROFILES_FILE',
                'RMW_FASTRTPS_USE_QOS_FROM_XML','ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    camera = camera_log = None
    children = []
    checks, events, samples, observations, envelopes, navigation, statuses = [], [], [], [], [], [], []
    source_stamps = set()
    seq = 0
    task_epoch = 1
    ready = False
    source_mode = 'new'
    image_side = 0
    last_image = None
    last_platform = last_image_send = last_resource = 0.
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    params = ['control_enabled:=true', 'engage_mode:=track',
              'camera_extrinsics_confirmed:=true', 'camera_extrinsics_id:=synthetic-mount',
              'camera_pos_x:=0.0', 'camera_pos_y:=0.0', 'camera_pos_z:=0.0',
              'standoff_calibration_confirmed:=true', 'standoff_calibration_id:=synthetic-standoff',
              'standoff_m:=1.0', 'horizontal_navigation_mode:=hold',
              'vision_control_topic:=/step5/camera/v2',
              'horizontal_command_topic:=/step5/camera/horizontal',
              'navigation_command_topic:=/step5/camera/navigation']
    init_args = ['--ros-args']
    for value in params:
        init_args += ['-p', value]
    rclpy.init(args=init_args)
    source = Node('step5_camera_fixture')
    mission = MissionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(mission)
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    platform = source.create_publisher(PlatformState, '/fc01/platform/state', sensor)
    native = source.create_publisher(NativeFlightSnapshot, '/fc01/native/flight_snapshot', sensor)
    image_pub = source.create_publisher(Image, '/step5/camera/image', sensor)
    extra_target = None

    def event(name, **kw):
        events.append(dict(event=name, monotonic=time.monotonic(), **kw))

    def receive_observation(msg):
        stamp = msg.stamp.sec*10**9+msg.stamp.nanosec
        observations.append(dict(stamp_ns=stamp, sequence=int(msg.sequence), tracking=msg.tracking,
            valid_fields=int(msg.valid_fields), source=msg.source_id, reset=int(msg.reset_counter),
            position=list(msg.position), receive_monotonic=time.monotonic(),
            source_to_callback_s=(time.time_ns()-stamp)*1e-9))

    def receive_envelope(msg):
        c = msg.command
        envelopes.append(dict(valid=c.valid, sequence=int(c.sequence), vx=c.vx, vy=c.vy, vz=c.vz,
            wz=c.wz, source=c.source_id, stamp_ns=c.stamp.sec*10**9+c.stamp.nanosec,
            attitude_stamp_ns=int(msg.attitude_sample_stamp_ns), origin=msg.origin_id,
            receive_monotonic=time.monotonic()))

    source.create_subscription(TargetObservation, '/fc01/vision/target_observation', receive_observation, sensor)
    source.create_subscription(HorizontalVelocityCommand, '/step5/camera/horizontal', receive_envelope, 1)
    source.create_subscription(NavigationCommand, '/step5/camera/navigation', navigation.append, 1)
    source.create_subscription(String, '/fc01/vision/camera_status', lambda m: statuses.append(json.loads(m.data)), 1)
    yy, xx = np.ogrid[:height, :width]
    pixels = {}
    for side in (-70, 0, 70):
        data = np.zeros((height, width, 3), np.uint8)
        data[(xx-(width//2+side))**2+(yy-height//2)**2 <= 35**2, 2] = 255
        pixels[side] = array('B', data.tobytes())

    def spawn_camera(label):
        nonlocal camera_log
        camera_log = (args.output/(label+'.log')).open('w')
        # 模型/标定确认只属于这个生成的圆形图像，绝不覆盖仓库或现场配置。
        overrides = dict(use_camera_info=False, image_topic='/step5/camera/image',
            source_id='synthetic-image-step5', target_shape='circle',
            calibration_confirmed=True, calibration_id='synthetic-pinhole',
            extrinsics_confirmed=True, extrinsics_id='synthetic-identity',
            target_model_confirmed=True, target_model_id='synthetic-circle',
            image_rectified_confirmed=True, capture_latency_confirmed=True,
            capture_latency_bound_s=.01, publish_debug_image=False, show_tuning_window=False)
        argv = [str(camera_binary), '--ros-args', '--params-file', str(config_path)]
        for key, value in overrides.items():
            argv += ['-p', key+':='+(str(value).lower() if isinstance(value, bool) else str(value))]
        (args.output/(label+'-command.json')).write_text(json.dumps(argv, indent=2)+'\n')
        p = subprocess.Popen(argv, stdout=camera_log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p)
        return p

    def stop_camera():
        nonlocal camera
        if camera is not None and camera.poll() is None:
            os.killpg(camera.pid, signal.SIGINT)
            code = camera.wait(timeout=8)
            if code != 0:
                raise RuntimeError('camera shutdown exit='+str(code))
        if camera_log is not None:
            camera_log.flush()

    def stats(pid):
        text = Path('/proc/'+str(pid)+'/stat').read_text()
        fields = text[text.rfind(')')+2:].split()
        # stat字段3开始：utime14/stime15/RSS24；CPU秒只属于该进程。
        return dict(cpu_seconds=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                    rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE'), threads=int(fields[17]))

    def send_platform():
        nonlocal seq, last_platform
        now = time.monotonic()
        if now-last_platform < .01:
            return
        last_platform = now
        seq += 1
        stamp = source.get_clock().now().to_msg()
        ns = (stamp.sec*10**9+stamp.nanosec)//1000*1000
        mono = time.monotonic_ns()
        p = PlatformState()
        p.sequence = seq
        p.stamp = stamp
        p.source_id = 'synthetic-platform-step5'
        p.origin_id = 'synthetic-origin-step5'
        p.frame_id, p.body_frame_id = 'px4_local_enu', 'base_link_flu'
        p.ready, p.require_angular_velocity, p.valid_fields = True, True, 15
        p.reason = 'SYNTHETIC'
        p.position, p.velocity, p.angular_velocity, p.attitude_xyzw = [0.,0.,1.5], [0.,0.,0.], [0.,0.,0.], [0.,0.,0.,1.]
        p.local_receive_monotonic_ns = p.attitude_receive_monotonic_ns = p.angular_velocity_receive_monotonic_ns = mono
        p.local_timestamp_sample_us = p.attitude_timestamp_sample_us = p.angular_velocity_timestamp_sample_us = ns//1000
        p.position_stamp_ns = p.velocity_stamp_ns = p.attitude_stamp_ns = p.angular_velocity_stamp_ns = ns
        platform.publish(p)
        n = NativeFlightState()
        n.sequence, n.stamp, n.task_epoch = seq, stamp, task_epoch
        n.boot_id, n.controller_pid, n.source_id, n.profile = boot, 17171, 'synthetic-native-step5', 'attitude_altitude'
        n.native_monotonic_ns = n.local_receive_monotonic_ns = mono
        n.local_sample_stamp_us = n.local_stamp_us = n.status_stamp_us = n.land_stamp_us = ns//1000
        n.attitude_stamp_us = n.attitude_sample_stamp_us = ns//1000
        n.feedback_current = n.publication_current = True
        n.rc_navigation_enabled = n.rc_manual_arm_proven = n.ground_reference_valid = n.ground_reference_qualified = True
        n.armed = n.offboard = n.liftoff_confirmed = n.target_height_settled = n.navigation_allowed = ready
        n.landed = not ready
        n.phase, n.reason = ('HEIGHT_HOLD' if ready else 'RC_READY_WAIT_ARM'), 'SYNTHETIC'
        n.rc_mode_slot = n.rc_navigation_mode_slot = 1
        n.reference_z_ned, n.z_ned, n.vz_ned, n.relative_height_m = 0., -1.5, 0., 1.5
        n.target_height_m, n.height_tolerance_m, n.speed_tolerance_mps, n.settle_seconds = 1.5, .15, .15, 2.
        # 与原生生产者相同，授权、原始NED/FRD姿态和原点在同一DDS消息内。
        # 此处平台ENU/FLU为单位旋转，对应原始PX4的NED航向+90度。
        snapshot = NativeFlightSnapshot()
        snapshot.state = n
        snapshot.attitude_wxyz = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
        snapshot.origin_id = p.origin_id
        snapshot.attitude_receive_monotonic_ns = mono
        native.publish(snapshot)

    def send_image():
        nonlocal last_image_send, last_image
        now = time.monotonic()
        if source_mode == 'silent' or now-last_image_send < 1./rate:
            return
        last_image_send = now
        if source_mode == 'duplicate':
            if last_image:
                image_pub.publish(last_image)
            return
        m = Image()
        m.header.stamp = source.get_clock().now().to_msg()
        if source_mode == 'old':
            m.header.stamp.sec -= 2
        elif source_mode == 'regressed':
            ns = int(time.time_ns()-100_000_000)
            m.header.stamp.sec, m.header.stamp.nanosec = ns//10**9, ns%10**9
        m.width, m.height, m.encoding, m.step = width, height, 'bgr8', width*3
        m.data = array('B') if source_mode == 'empty' else pixels[image_side]
        image_pub.publish(m)
        stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if source_mode == 'new':
            source_stamps.add(stamp)
        if source_mode == 'new':
            last_image = m

    def spin(seconds):
        nonlocal last_resource
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            send_platform()
            send_image()
            executor.spin_once(timeout_sec=.0005)
            now = time.monotonic()
            if camera is not None and camera.poll() is None and now-last_resource > .1:
                last_resource = now
                samples.append(dict(monotonic=now, camera=stats(camera.pid),
                    harness_and_mission=stats(os.getpid()), target_consecutive=mission.target_consecutive,
                    target_locked=mission.target_locked, native=mission.native_gate.reason,
                    mission_phase=mission.flight_state, fault=mission.horizontal_fault_reason))

    def wait(predicate, timeout=10.):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            spin(.02)
            if predicate():
                return
            if camera is not None and camera.poll() is not None:
                raise AssertionError('camera exited '+str(camera.returncode))
        raise AssertionError(repr(dict(phase=mission.flight_state, target=mission.target_reason,
            fault=mission.horizontal_fault_reason, native=mission.native_gate.reason,
            platform=mission.sensor_reason, observations=observations[-1:], envelopes=envelopes[-1:])))

    def new_task():
        nonlocal task_epoch, source_mode, ready, image_side
        ready = False
        source_mode = 'silent'
        spin(.12)
        task_epoch += 1
        spin(.12)
        source_mode = 'new'
        image_side = 0
        ready = True
        wait(lambda: envelopes and envelopes[-1]['valid'] and not mission.horizontal_fault_reason)
        spin(.08)
        event('synthetic_native_new_task', task_epoch=task_epoch)

    def fault_check(mode, label):
        nonlocal source_mode
        new_task()
        source_mode = mode
        injected = time.monotonic()
        wait(lambda: mission.horizontal_fault_reason and envelopes and not envelopes[-1]['valid'])
        assert not navigation[-1].valid
        assert all(envelopes[-1][key] == 0. for key in ('vx','vy','vz','wz'))
        event(label, reason=mission.horizontal_fault_reason, revoke_latency_s=time.monotonic()-injected)
        checks.append(label)

    result = None
    try:
        camera = spawn_camera('camera')
        wait(lambda: image_pub.get_subscription_count() == 1 and mission.target_locked and envelopes)
        assert not envelopes[-1]['valid'] and not navigation[-1].valid
        checks.append('images_cannot_bypass_native_measured_height_gate')
        ready = True
        wait(lambda: envelopes[-1]['valid'] and not navigation[-1].valid)
        assert mission.flight_state == 'track' and mission.target_consecutive >= 3
        checks.append('actual_cpp_detection_to_actual_mission_continuous_new_frame_track')
        # 以任务真正消费后的世界实际方向检查左右目标，避免只看识别框。
        for side in (70, -70):
            image_side = side
            spin(1.)
            wait(lambda: envelopes[-1]['valid'] and envelopes[-1]['vy']*side > .001)
            assert abs(envelopes[-1]['wz']) <= .350001
        checks.append('left_right_image_target_changes_attributable_real_dds_velocity_direction')
        image_side = 0
        spin(.5)
        start = time.monotonic()
        before_obs = len(observations)
        before_frames = len(source_stamps)
        before_envelopes = len(envelopes)
        before_resources = dict(camera=stats(camera.pid), harness_and_mission=stats(os.getpid()))
        spin(args.benchmark_seconds)
        elapsed = time.monotonic()-start
        after_resources = dict(camera=stats(camera.pid), harness_and_mission=stats(os.getpid()))
        window = observations[before_obs:]
        valid = [m for m in window if m['tracking'] and m['valid_fields'] & TargetObservation.POSITION]
        unique = {m['stamp_ns'] for m in valid}
        assert len(unique) >= 5 and all(m['stamp_ns'] in source_stamps for m in valid)
        assert envelopes[-1]['valid'] and not mission.horizontal_fault_reason
        latencies = sorted(m['source_to_callback_s'] for m in valid)
        metrics = dict(profile=dict(width=width,height=height,requested_source_hz=rate,
                 debug_image=False,visualization=False,image_transport='raw bgr8 ROS DDS; synthetic source'),
            observation_window_s=elapsed, source_new_frames=len(source_stamps)-before_frames,
            target_messages=len(window), target_valid_unique_frames=len(unique),
            effective_valid_new_frame_hz=len(unique)/elapsed,
            source_effective_new_frame_hz=(len(source_stamps)-before_frames)/elapsed,
            source_stamp_to_target_callback_s=dict(median=statistics.median(latencies),
                p95=latencies[min(len(latencies)-1, int(.95*len(latencies)))], max=max(latencies)),
            mission_horizontal_messages=len(envelopes)-before_envelopes,
            process_metrics={key:dict(cpu_percent_one_core=(after_resources[key]['cpu_seconds']-before_resources[key]['cpu_seconds'])/elapsed*100.,
                rss_bytes=after_resources[key]['rss_bytes'],rss_growth_bytes=after_resources[key]['rss_bytes']-before_resources[key]['rss_bytes'],
                threads=after_resources[key]['threads']) for key in before_resources})
        checks.append('640x480_120hz_requested_profile_effective_new_frame_and_resource_measurement')
        fault_check('duplicate', 'duplicate_raw_image_revokes_without_renewing')
        fault_check('old', 'old_raw_image_explicit_invalid_and_withdrawal')
        fault_check('empty', 'empty_raw_image_explicit_invalid_and_withdrawal')
        fault_check('silent', 'image_disconnect_timer_and_mission_deadlines_withdraw')
        fault_check('regressed', 'image_clock_regression_resets_and_withdraws')
        new_task()
        old_sources = {m['source'] for m in observations}
        stop_camera()
        # 正常停止后才重开，严格保持单一发布者；源会话改变不得清除已开始跟踪的故障。
        source_mode = 'silent'
        spin(.12)
        camera = spawn_camera('camera-restart')
        source_mode = 'new'
        wait(lambda: any(m['source'] not in old_sources for m in observations))
        wait(lambda: mission.horizontal_fault_reason and not envelopes[-1]['valid'])
        checks.append('camera_process_restart_new_source_cannot_restore_old_active_task')
        new_task()
        extra_target = source.create_publisher(TargetObservation, '/fc01/vision/target_observation', sensor)
        wait(lambda: mission.horizontal_fault_reason and not envelopes[-1]['valid'])
        assert 'PUBLISHER' in mission.horizontal_fault_reason
        checks.append('second_target_publisher_revokes_active_tracking')
        source.destroy_publisher(extra_target)
        extra_target = None
        assert not any(name.startswith('/fmu/in/') and source.count_publishers(name)
                       for name,_ in source.get_topic_names_and_types())
        checks.append('no_native_controller_or_fmu_input_publisher')
        result = dict(result='PASS', checks=checks, environment='Ubuntu22.04 x86_64 ROS2 Humble localhost',
            synthetic=True, hardware=False, native_controller_started=False,
            image_source='synthetic sensor_msgs/Image through real production vision_image_node',
            scope='Image→C++ detector/estimator→TargetObservation DDS→actual Mission→HorizontalVelocityCommand DDS',
            exclusions=['V4L2 CameraSource/exposure/USB latency','ARM64 FPS/resources','PX4 actual receive/adoption','physical target recognition or flight'],
            transport_profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),
            transport_profile=str(profile.relative_to(root)),
            profile_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
            camera_binary_sha256=hashlib.sha256(camera_binary.read_bytes()).hexdigest(),
            harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            metrics=metrics, events=events)
        print('FC01_CAMERA_PIPELINE_ROS_PASS', len(checks), 'named checks')
    except Exception as error:
        result = dict(result='FAIL', error=repr(error), checks=checks, events=events)
        raise
    finally:
        source_mode = 'silent'
        for p in children:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=3)
        if camera_log:
            camera_log.close()
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        (args.output/'observations.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in observations))
        (args.output/'envelopes.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in envelopes))
        (args.output/'resources.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in samples))
        (args.output/'camera-status.json').write_text(json.dumps(statuses,ensure_ascii=False,indent=2)+'\n')
        executor.remove_node(mission)
        executor.remove_node(source)
        mission.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

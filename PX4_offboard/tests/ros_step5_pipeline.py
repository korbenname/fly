#!/usr/bin/env python3
"""真实图像检测→PX4桥/mission→监督器/原生→DDS接收边界的隔离软件回归。

全域localhost、合成相机/手动RC/PX4端；不是PX4固件SITL或硬件采纳证明。
保持生产1.5m参数，垂向简化模型，无横向动力学。所有输出仅由本机模型消费。
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
from array import array
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True, type=Path)
    parser.add_argument('--build-evidence', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=181)
    parser.add_argument(
        '--scenario', choices=('normal', 'target_loss', 'no_vision', 'dds_loss'), default='normal'
    )
    parser.add_argument(
        "--attitude-hz",
        type=float,
        default=200.0,
        help="independent raw PX4 attitude rate; platform bridge stays50Hz",
    )
    args = parser.parse_args()
    if not 50.0 <= args.attitude_hz <= 400.0:
        parser.error("attitude-hz must be50..400")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    if not 1 <= args.domain <= 232:
        parser.error('isolated nonzero domain required')
    os.environ.update(
        ROS_DOMAIN_ID=str(args.domain),
        ROS_LOCALHOST_ONLY='1',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
    )
    for key in (
        'ROS_DISCOVERY_SERVER',
        'CYCLONEDDS_URI',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'FASTDDS_DEFAULT_PROFILES_FILE',
        'ROS_STATIC_PEERS',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
    ):
        os.environ.pop(key, None)
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
        ROOT / 'tests/fixtures/step5_camera_local_shm.xml'
    )
    node_path = args.prefix / 'jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
    vision_path = args.prefix / 'fc01_vision/lib/fc01_vision/vision_image_node'
    proof = json.loads(args.build_evidence.read_text())
    if sha(node_path) != proof['binary_sha256'] or any(
        sha(ROOT / p) != h for p, h in proof['production_sha256'].items()
    ):
        raise RuntimeError('current source/ELF mismatch')
    installed_mission = (
        args.prefix / 'fc01_mission/lib/python3.10/site-packages/fc01_mission/mission_node.py'
    )
    if sha(installed_mission) != sha(ROOT / 'target/src/fc01_mission/fc01_mission/mission_node.py'):
        raise RuntimeError('installed mission/source mismatch: rebuild unified overlay')
    import yaml
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from px4_msgs.msg import (
        BatteryStatus,
        FailsafeFlags,
        ManualControlSetpoint,
        ManualControlSwitches,
        VehicleAttitude,
        VehicleLocalPosition,
        VehicleAngularVelocity,
        VehicleStatus,
        VehicleLandDetected,
        VehicleCommandAck,
        VehicleCommand,
        VehicleAttitudeSetpoint,
        OffboardControlMode,
    )
    from jikl_fc01_system.msg import (
        TargetObservation,
        HorizontalVelocityCommand,
        NavigationCommand,
        NativeFlightState,
        PlatformState,
    )

    rclpy.init(args=[])
    peer = Node('fc01_step5_synthetic_px4_camera')
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    classes = {
        'vehicle_status_v1': VehicleStatus,
        'vehicle_land_detected': VehicleLandDetected,
        'vehicle_local_position': VehicleLocalPosition,
        'vehicle_attitude': VehicleAttitude,
        'vehicle_angular_velocity': VehicleAngularVelocity,
        'failsafe_flags': FailsafeFlags,
        'battery_status': BatteryStatus,
        'manual_control_setpoint': ManualControlSetpoint,
        'manual_control_switches': ManualControlSwitches,
        'vehicle_command_ack': VehicleCommandAck,
    }
    pubs = {
        name: peer.create_publisher(cls, '/fmu/out/' + name, sensor)
        for name, cls in classes.items()
    }
    image_pub = peer.create_publisher(Image, '/step5/camera/image_raw', sensor)
    targets = []
    heartbeats = []
    commands = []
    observations = []
    envelopes = []
    navs = []
    native = []
    platform = []
    mission_status = []
    events = []
    states = []
    samples = []
    resources = []
    children = []
    handles = []
    ipc = None
    sequence = 0
    state = {}
    thrust = 0.0
    h = 0.0
    speed = 0.0
    armed = False
    offboard = False
    arm_switch = 3
    mode_slot = 6
    feedback_on = True
    target_side = 0
    camera_on = False
    have_target = False
    last_frame = 0.0
    image_count = 0
    last_feedback = 0.0
    last_status = 0.0
    last_resource = 0.0
    started = time.monotonic()
    last_step = started
    ack_at = None
    release_at = None
    ack_sent = False
    release_sent = False
    phase_previous = None
    native_pid = None
    error = None
    result = 'FAIL'
    checks = []
    hold_at = None
    ramp_at = None
    nav_at = None
    fault_at = None
    cpu_baseline = {}
    proc_start = {}
    private = None
    supervisor = None
    temporary_home = None
    frames_log = (output / 'wire.jsonl').open('w')
    state_log = (output / 'states.jsonl').open('w')

    def finite_or_none(value):
        return float(value) if math.isfinite(float(value)) else None

    def event(name, **values):
        events.append(dict(event=name, monotonic=time.monotonic(), **values))

    def target_callback(message):
        nonlocal thrust
        q = [float(x) for x in message.q_d]
        u = [float(x) for x in message.thrust_body]
        if not all(math.isfinite(x) for x in q + u) or not -0.800001 <= u[2] <= 0 or u[0] or u[1]:
            raise RuntimeError('invalid actual target wire')
        if not armed and any(u):
            raise RuntimeError('power before synthetic manual arm')
        thrust = -u[2]
        row = dict(
            kind='target', received=time.monotonic(), timestamp=message.timestamp, q=q, thrust=u
        )
        targets.append(row)
        frames_log.write(json.dumps(row) + '\n')

    def heartbeat_callback(message):
        if not message.attitude or any(
            (
                message.position,
                message.velocity,
                message.acceleration,
                message.body_rate,
                message.thrust_and_torque,
                message.direct_actuator,
            )
        ):
            raise RuntimeError('Offboard architecture changed')
        row = dict(kind='heartbeat', received=time.monotonic(), timestamp=message.timestamp)
        heartbeats.append(row)
        frames_log.write(json.dumps(row) + '\n')

    def command_callback(message):
        nonlocal ack_at, release_at
        if not (
            message.command == 400
            and message.param1 == 0
            or message.command == 176
            and message.param1 == 1
            and message.param2 == 7
            and not armed
        ):
            raise RuntimeError('unexpected automatic command ' + str(message.command))
        if h > 0.15 or abs(speed) > 0.15:
            raise RuntimeError('unsafe disarm/release height')
        row = dict(
            command=message.command,
            param1=message.param1,
            param2=message.param2,
            received=time.monotonic(),
            height=h,
            speed=speed,
        )
        commands.append(row)
        event('actual_px4_topic_command_received', **row)
        if message.command == 400:
            ack_at = time.monotonic() + 0.15
        else:
            release_at = time.monotonic() + 0.15

    def observation_callback(message):
        observations.append(
            dict(
                received=time.monotonic(),
                receive_unix_ns=time.time_ns(),
                stamp_ns=message.stamp.sec * 10**9 + message.stamp.nanosec,
                sequence=message.sequence,
                tracking=message.tracking,
                valid_fields=message.valid_fields,
                position=[float(x) if math.isfinite(x) else None for x in message.position],
                source=message.source_id,
                reset=message.reset_counter,
            )
        )

    def envelope_callback(message):
        c = message.command
        envelopes.append(
            dict(
                received=time.monotonic(),
                sequence=c.sequence,
                valid=c.valid,
                v=[c.vx, c.vy, c.vz],
                wz=c.wz,
                attitude_stamp=message.attitude_sample_stamp_ns,
                origin=message.origin_id,
            )
        )

    def navigation_callback(message):
        navs.append(
            dict(
                received=time.monotonic(),
                sequence=message.sequence,
                valid=message.valid,
                mode=message.mode,
                position=[float(x) for x in message.position],
            )
        )

    subscriptions = [
        peer.create_subscription(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', target_callback, reliable
        ),
        peer.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', heartbeat_callback, reliable
        ),
        peer.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command', command_callback, reliable
        ),
        peer.create_subscription(
            TargetObservation, '/fc01/vision/target_observation', observation_callback, sensor
        ),
        peer.create_subscription(
            HorizontalVelocityCommand,
            '/fc01/vision/horizontal_command',
            envelope_callback,
            reliable,
        ),
        peer.create_subscription(
            NavigationCommand, '/fc01/navigation/command', navigation_callback, reliable
        ),
        peer.create_subscription(
            NativeFlightState, '/fc01/native/flight_state', native.append, sensor
        ),
        peer.create_subscription(PlatformState, '/fc01/platform/state', platform.append, sensor),
        peer.create_subscription(
            String, '/fc01/mission/sensors/status', lambda m: mission_status.append(m.data), sensor
        ),
    ]

    attitude_stop = threading.Event()
    attitude_stamps = []
    attitude_errors = []

    def attitude_source():
        # 独立高频raw姿态；桥50Hz/原生20Hz/原生状态10Hz不能假定取到相同帧。
        while not attitude_stop.is_set():
            try:
                if feedback_on:
                    stamp = peer.get_clock().now().nanoseconds // 1000
                    pubs['vehicle_attitude'].publish(
                        VehicleAttitude(
                            timestamp=stamp, timestamp_sample=stamp - 2000, q=[1.0, 0.0, 0.0, 0.0]
                        )
                    )
                    attitude_stamps.append((time.monotonic(), stamp))
            except Exception as exc:
                attitude_errors.append(repr(exc))
                return
            attitude_stop.wait(1.0 / args.attitude_hz)

    attitude_thread = threading.Thread(target=attitude_source, name='synthetic-px4-attitude')
    attitude_thread.start()

    def publish_feedback():
        if not feedback_on:
            return
        stamp = peer.get_clock().now().nanoseconds // 1000
        ground = h <= 0.015 and abs(speed) <= 0.08
        payload = dict(
            vehicle_status_v1=VehicleStatus(
                timestamp=stamp,
                system_id=1,
                component_id=1,
                arming_state=2 if armed else 1,
                nav_state=14 if offboard else 15,
                armed_time=10 if armed else 0,
                latest_arming_reason=2 if armed else 0,
                pre_flight_checks_pass=True,
                power_input_valid=True,
                can_set_nav_states_mask=1 << 15,
            ),
            vehicle_land_detected=VehicleLandDetected(
                timestamp=stamp,
                landed=ground,
                at_rest=ground,
                ground_contact=ground,
                maybe_landed=ground,
            ),
            vehicle_local_position=VehicleLocalPosition(
                timestamp=stamp,
                timestamp_sample=stamp - 2000,
                x=0.0,
                y=0.0,
                z=2.0 - h,
                vx=0.0,
                vy=0.0,
                vz=-speed,
                z_deriv=-speed,
                xy_valid=True,
                v_xy_valid=True,
                z_valid=True,
                v_z_valid=True,
                heading_good_for_control=True,
                dist_bottom_valid=True,
                dist_bottom=0.1 + h,
                dist_bottom_var=0.0025,
                dist_bottom_sensor_bitfield=VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE,
            ),
            vehicle_angular_velocity=VehicleAngularVelocity(
                timestamp=stamp, timestamp_sample=stamp - 2000, xyz=[0.0, 0.0, 0.0]
            ),
            failsafe_flags=FailsafeFlags(timestamp=stamp),
            battery_status=BatteryStatus(
                timestamp=stamp, id=1, connected=True, voltage_v=7.6, remaining=1.0
            ),
            manual_control_setpoint=ManualControlSetpoint(
                timestamp=stamp,
                timestamp_sample=stamp - 2000,
                valid=True,
                data_source=ManualControlSetpoint.SOURCE_RC,
                throttle=-1.0,
            ),
            manual_control_switches=ManualControlSwitches(
                timestamp=stamp,
                timestamp_sample=stamp - 2000,
                mode_slot=mode_slot,
                arm_switch=arm_switch,
                kill_switch=3,
                return_switch=3,
                loiter_switch=3,
                offboard_switch=3,
            ),
        )
        for name, msg in payload.items():
            pubs[name].publish(msg)
        samples.append(
            dict(received=time.monotonic(), height=h, speed=speed, armed=armed, offboard=offboard)
        )

    def publish_image(now):
        nonlocal last_frame, image_count
        if not camera_on or now - last_frame < 1.0 / 120:
            return
        last_frame = now
        image_count += 1
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        if have_target:
            # 合成红圆只创建已知像素；真实OpenCV检测/估计仍在C++ vision_image_node内运行。
            yy, xx = np.ogrid[:480, :640]
            image[(xx - (320 + target_side)) ** 2 + (yy - 240) ** 2 <= 23**2] = (0, 0, 255)
        msg = Image()
        msg.header.stamp = peer.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic_optical'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'bgr8'
        msg.step = 640 * 3
        msg.data = array('B', image.tobytes())
        image_pub.publish(msg)

    def collect_resources(now):
        nonlocal native_pid
        pids = [(name, p.pid) for name, p in children]
        if native_pid:
            pids.append(('native', native_pid))
        for name, pid in pids:
            try:
                text = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
                ticks = int(text[11]) + int(text[12])
                rss = int(text[21]) * os.sysconf('SC_PAGE_SIZE')
                cpu_baseline.setdefault((name, pid), ticks)
                proc_start.setdefault((name, pid), now)
                resources.append(
                    dict(name=name, pid=pid, received=now, cpu_ticks=ticks, rss_bytes=rss)
                )
            except (FileNotFoundError, ProcessLookupError):
                pass

    def send(op):
        nonlocal sequence
        sequence += 1
        words = op.split()
        with (output / 'service/test-requests.jsonl').open('a') as requests:
            requests.write(
                json.dumps(dict(id=sequence, op=words[0], args=[float(x) for x in words[1:]]))
                + '\n'
            )
        return sequence

    def step():
        nonlocal h, speed, last_step, last_feedback, last_status, last_resource, ipc, state, phase_previous, native_pid
        nonlocal ack_at, release_at, ack_sent, release_sent
        if attitude_errors:
            raise RuntimeError(attitude_errors[-1])
        now = time.monotonic()
        dt = min(0.03, now - last_step)
        last_step = now
        if armed:
            speed += (9.80665 * (thrust / 0.5 - 1.0) - 1.4 * speed) * dt
            h += speed * dt
            if h < 0:
                h = speed = 0.0
        else:
            h = speed = 0.0
        if now - last_feedback >= 0.02:
            publish_feedback()
            last_feedback = now
        publish_image(now)
        if ack_at and now >= ack_at:
            pubs['vehicle_command_ack'].publish(
                VehicleCommandAck(
                    timestamp=peer.get_clock().now().nanoseconds // 1000,
                    command=400,
                    result=0,
                    target_system=1,
                    target_component=191,
                )
            )
            ack_at = None
            ack_sent = True
            event('disarm_ack')
        if release_at and now >= release_at:
            pubs['vehicle_command_ack'].publish(
                VehicleCommandAck(
                    timestamp=peer.get_clock().now().nanoseconds // 1000,
                    command=176,
                    result=0,
                    target_system=1,
                    target_component=191,
                )
            )
            release_at = None
            release_sent = True
        for _ in range(3):
            rclpy.spin_once(peer, timeout_sec=0.0001)
        for name, process in children:
            if process.poll() is not None:
                raise RuntimeError(name + ' exited ' + str(process.returncode))
        if ipc is None and (output / 'service/test-ipc.jsonl').exists():
            ipc = (output / 'service/test-ipc.jsonl').open()
        if ipc:
            while True:
                line = ipc.readline()
                if not line:
                    break
                state = json.loads(line)
                state['_received'] = state.pop('_test_received')
                states.append(state)
                state_log.write(json.dumps(state) + '\n')
                native_pid = state.get('_test_controller_pid', native_pid)
                if state.get('phase') != phase_previous:
                    event(
                        'native_phase',
                        phase=state.get('phase'),
                        reason=state.get('reason'),
                        height=h,
                    )
                    print(
                        args.scenario,
                        state.get('phase'),
                        state.get('reason'),
                        'h=',
                        round(h, 3),
                        flush=True,
                    )
                    phase_previous = state.get('phase')
        if now - last_resource >= 0.25:
            collect_resources(now)
            last_resource = now
        time.sleep(0.0005)

    def until(predicate, seconds, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            step()
            if predicate():
                return
        raise RuntimeError(
            label
            + ' timeout '
            + str(
                {
                    k: state.get(k)
                    for k in (
                        'phase',
                        'reason',
                        'visual_reason',
                        'navigation_allowed',
                        'horizontal_matched_attitude_valid',
                    )
                }
            )
            + ' mission='
            + str(mission_status[-1:])
        )

    def spawn(name, argv, env=None):
        log = (output / (name + '.log')).open('w')
        handles.append(log)
        p = subprocess.Popen(
            argv, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env
        )
        children.append((name, p))
        event('spawn', process_name=name, pid=p.pid, argv=[str(x) for x in argv])
        return p

    try:
        rclpy.spin_once(peer, timeout_sec=0.1)
        if any(n != peer.get_name() for n in peer.get_node_names()):
            raise RuntimeError('domain not empty')
        params = yaml.safe_load((ROOT / 'tests/fixtures/stage3_runtime_20261114.yaml').read_text())
        p = params['fc01_system']['ros__parameters']
        p.update(
            visual_navigation_enabled=args.scenario != 'no_vision',
            px4_horizontal_enabled=args.scenario != 'no_vision',
            fc01_entry_version='step5-synthetic',
            fc01_config_path=str(output / 'runtime.yaml'),
            fc01_config_source_path=str(output / 'runtime.yaml'),
        )
        (output / 'runtime.yaml').write_text(yaml.safe_dump(params))
        if args.scenario != 'no_vision':
            vp = yaml.safe_load((ROOT / 'target/src/fc01_vision/config/vision.yaml').read_text())[
                '/**'
            ]['ros__parameters']
            vp.update(
                use_camera_info=False,
                image_topic='/step5/camera/image_raw',
                target_shape='circle',
                use_pnp=False,
                calibration_confirmed=True,
                calibration_id='synthetic-pinhole',
                image_rectified_confirmed=True,
                extrinsics_confirmed=True,
                extrinsics_id='synthetic-identity',
                target_model_confirmed=True,
                target_model_id='synthetic-circle-0.2m',
                capture_latency_confirmed=True,
                capture_latency_bound_s=0.01,
                show_tuning_window=False,
                publish_debug_image=False,
            )
            (output / 'vision.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': vp}}))
            spawn(
                'vision',
                [str(vision_path), '--ros-args', '--params-file', str(output / 'vision.yaml')],
            )
            spawn(
                'bridge',
                [
                    sys.executable,
                    '-m',
                    'fc01_mission.px4_state_bridge',
                    '--ros-args',
                    '--params-file',
                    str(ROOT / 'target/src/fc01_mission/config/platform.yaml'),
                ],
            )
            mp = yaml.safe_load((ROOT / 'target/src/fc01_mission/config/mission.yaml').read_text())
            mp['fc01_mission']['ros__parameters'].update(
                control_enabled=True,
                engage_mode='track',
                horizontal_navigation_mode='waypoint',
                waypoint_alignment_confirmed=True,
                waypoint_alignment_origin_id='px4_local:0:0:0:0:0:0',
                navigation_waypoint_x=0.0,
                navigation_waypoint_y=0.3,
                camera_extrinsics_confirmed=True,
                camera_extrinsics_id='synthetic-mount',
                camera_pos_x=0.0,
                camera_pos_y=0.0,
                camera_pos_z=0.0,
                standoff_calibration_confirmed=True,
                standoff_calibration_id='synthetic-standoff',
                standoff_m=1.0,
                yaw_rate_gain=0.0,
                vision_control_topic='/fc01/vision/velocity_command_v2',
                navigation_command_topic='/fc01/navigation/command',
            )
            (output / 'mission.yaml').write_text(yaml.safe_dump(mp))
            spawn(
                'mission',
                [
                    sys.executable,
                    '-m',
                    'fc01_mission.mission_node',
                    '--ros-args',
                    '--params-file',
                    str(output / 'mission.yaml'),
                ],
            )
        temporary_home = tempfile.TemporaryDirectory(prefix='fc01-step5-home-')
        tmp = temporary_home.name
        private = Path(tmp)
        env = dict(os.environ, HOME=tmp, FC01_STEP5_PRIVATE_HOME=tmp)
        supervisor = spawn(
            'supervisor',
            [
                sys.executable,
                str(ROOT / 'tests/fixtures/step5_supervisor.py'),
                '--node',
                str(node_path),
                '--config',
                str(output / 'runtime.yaml'),
                '--output',
                str(output / 'service'),
            ],
            env,
        )
        until(lambda: state.get('phase') == 'RC_READY_WAIT_ARM', 15, 'service actual READY')
        until(lambda: native and native[-1].ready_wait_arm, 3, 'current native READY')
        if any(e['valid'] for e in envelopes) or any(n['valid'] for n in navs):
            raise RuntimeError('mission bypassed preheight gate')
        checks.append('service_supervisor_native_current_ready_no_auto_arm')
        seq = send('TAKEOFF 1.5')
        until(lambda: any(r.get('_test_request_id') == seq for r in states), 2, 'console refusal')
        if next(r for r in states if r.get('_test_request_id') == seq).get('accepted'):
            raise RuntimeError('console auto takeoff')
        arm_switch = 1
        event('manual_rc_arm_edge')
        until(lambda: state.get('phase') == 'RC_ARMED_WAIT_NAVIGATION', 2, 'RC edge')
        armed = True
        event('synthetic_px4_manual_arm_confirmed')
        until(lambda: state.get('reason') == 'RC_ARMED_WAIT_CH6_NAVIGATION', 2, 'actual arm')
        mode_slot = 1
        event('rc_navigation_slot_1')
        edge = time.monotonic() + 0.15
        until(lambda: time.monotonic() > edge, 0.5, 'separate mode feedback')
        offboard = True
        nav_at = time.monotonic()
        event('synthetic_px4_offboard_confirmed')
        until(lambda: state.get('phase') == 'THRUST_RAMP', 3, 'ramp')
        ramp_at = time.monotonic()
        until(lambda: state.get('phase') == 'HEIGHT_HOLD', 16, 'actual native liftoff and settling')
        until(lambda: native and native[-1].navigation_allowed, 3, 'measured native authorization')
        hold_at = time.monotonic()
        if any(
            r.get('visual_motion') for r in states if r.get('phase') in ('THRUST_RAMP', 'ASCEND')
        ):
            raise RuntimeError('preheight horizontal output')
        checks.append('manual_rc_px4_offboard_liftoff_settle_1_5m')
        if args.scenario != 'no_vision':
            until(
                lambda: state.get('visual_reason') == 'HORIZONTAL_WAYPOINT'
                and targets[-1]['q'][2] < -0.001,
                3,
                'mission actual waypoint',
            )
            checks.append('real_bridge_mission_waypoint_native_wire')
            camera_on = have_target = True
            target_side = 75
            event('image_target_right', pixel_x=395)
            until(
                lambda: state.get('visual_reason') == 'HORIZONTAL_TRACK_PX4_VELOCITY_CLOSED_LOOP'
                and any(t['received'] > hold_at and t['q'][1] > 0.001 for t in targets),
                5,
                'camera detection mission right tilt',
            )
            if not any(o['tracking'] for o in observations) or not any(
                e['valid'] for e in envelopes
            ):
                raise RuntimeError('missing camera command evidence')
            checks.append('image_detection_lock_nav_withdraw_tracking_right_wire')
            before = time.monotonic()
            target_side = -75
            event('image_target_left', pixel_x=245)
            until(
                lambda: any(t['received'] > before and t['q'][1] < -0.001 for t in targets),
                4,
                'target change causes opposite tilt',
            )
            checks.append('image_target_change_attributable_left_wire')
            stable = time.monotonic() + 2.0
            until(lambda: time.monotonic() > stable, 3, 'active camera resources')
            if args.scenario == 'dds_loss':
                feedback_on = False
                fault_at = time.monotonic()
                event('all_px4_dds_feedback_stopped')
                until(
                    lambda: state.get('phase') == 'SAFETY', 3, 'all DDS missing native safety latch'
                )
                # 先排空在途DDS；没有飞控反馈时不能继续证明受控下降或飞行状态。
                drained = time.monotonic() + 0.2
                until(lambda: time.monotonic() > drained, 0.6, 'drain in-flight wire')
                cut = (len(targets), len(heartbeats))
                silence = time.monotonic() + 0.6
                until(lambda: time.monotonic() > silence, 1.0, 'all DDS safety stops native wire')
                if cut != (len(targets), len(heartbeats)):
                    raise RuntimeError('native continued wire without feedback')
                checks.append('all_dds_missing_actual_native_latch_and_wire_silence')
                # 此分支没有软件可验证的自动降落；明确模拟外部人员已地面解除，
                # 仅用于安全结束本机夹具，不计作故障受控落地证据。
                armed = False
                offboard = False
                arm_switch = 3
                mode_slot = 6
                feedback_on = True
                h = speed = 0.0
                event('test_only_external_ground_disarm_after_feedback_loss')
            if args.scenario == 'target_loss':
                camera_on = False
                fault_at = time.monotonic()
                event('camera_stream_stopped')
                until(
                    lambda: state.get('phase') in ('DESCENT', 'RECOVERY_DESCENT'),
                    3,
                    'real camera loss native descent',
                )
                if state.get('visual_motion'):
                    raise RuntimeError('fault retained visual control')
                checks.append('camera_loss_actual_mission_revoke_native_descent')
        if args.scenario == 'no_vision':
            stable = time.monotonic() + 2.0
            until(lambda: time.monotonic() > stable, 3, 'no task measured height hold')
            if state.get('phase') != 'HEIGHT_HOLD' or abs(h - 1.5) > 0.15:
                raise RuntimeError('no-vision height hold failed')
            checks.append('no_task_native_height_hold_observation')
        if args.scenario not in ('target_loss', 'dds_loss'):
            seq = send('LAND')
            event('normal_land_request')
            until(lambda: any(r.get('_test_request_id') == seq for r in states), 2, 'LAND reply')
            if not next(r for r in states if r.get('_test_request_id') == seq).get('accepted'):
                raise RuntimeError('land refused')
        if args.scenario != 'dds_loss':
            until(
                lambda: any(c['command'] == 400 for c in commands),
                28,
                'continuous descent to Disarm',
            )
            until(lambda: ack_sent and state.get('m1_400_ack_accepted'), 2, 'attributable ACK')
            ack_only = time.monotonic() + 0.5
            until(lambda: time.monotonic() > ack_only, 1, 'ACK alone not disarmed')
            if not state.get('armed') or state.get('m1_400_state_verified'):
                raise RuntimeError('ACK replaced actual state')
            armed = False
            event('synthetic_px4_new_disarmed_state')
            until(lambda: release_sent, 3, 'native state-confirmed Disarm and release')
            offboard = False
            arm_switch = 3
            mode_slot = 6
            event('manual_ground_release')
            until(
                lambda: state.get('phase') == 'RC_READY_WAIT_ARM', 12, 'second fresh manual READY'
            )
            if any(state.get('thrust_body', [])):
                raise RuntimeError('power on reprepare')
            checks.append('continuous_descent_ground_ack_and_disarm_release_reprepare')
            if args.scenario == 'target_loss' and any(
                r.get('visual_motion')
                for r in states
                if fault_at <= r['_received'] and r.get('phase') in ('DESCENT', 'RECOVERY_DESCENT')
            ):
                raise RuntimeError('fault descent resumed visual')
        else:
            until(
                lambda: state.get('phase') == 'RC_READY_WAIT_ARM',
                12,
                'external ground recovery for test cleanup',
            )
        for topic in ('vehicle_attitude_setpoint', 'offboard_control_mode', 'vehicle_command'):
            writers = peer.get_publishers_info_by_topic('/fmu/in/' + topic)
            if len(writers) != 1 or writers[0].node_name != 'fc01_system':
                raise RuntimeError('native publisher not unique ' + topic)
        checks.append('single_native_control_publisher')
        supervisor.send_signal(signal.SIGTERM)
        event('service_graceful_stop_requested')
        # 原生要保持DDS地面反馈直到自己退出；监督器poll后不再走一般step早退出检查。
        stopping = time.monotonic() + 12
        while supervisor.poll() is None and time.monotonic() < stopping:
            publish_feedback()
            rclpy.spin_once(peer, timeout_sec=0.005)
            time.sleep(0.01)
        if supervisor.poll() != 0:
            raise RuntimeError('supervisor did not exit cleanly')
        checks.append('production_supervisor_graceful_stop_and_no_orphan')
        if native_pid and Path('/proc', str(native_pid)).exists():
            raise RuntimeError('orphan native')
        # 所有实际/fmu/in控制端点的唯一writer必须仍是原生控制器。
        for topic in ('vehicle_attitude_setpoint', 'offboard_control_mode', 'vehicle_command'):
            writers = peer.get_publishers_info_by_topic('/fmu/in/' + topic)
            # 此时监督器已停止，控制writer应该随原生节点退出。
            if len(writers) > 1:
                raise RuntimeError('multiple native control publishers')
        result = 'PASS'
    except Exception as exc:
        error = repr(exc)
        event('failure', error=error)
        print(error, flush=True)
    finally:
        # 测试进程拥有所有合成进程。失败清理显式单列，绝不作为连续故障落地证据。
        if supervisor and supervisor.poll() is None:
            supervisor.send_signal(signal.SIGTERM)
            cleanup = time.monotonic() + 3
            while supervisor.poll() is None and time.monotonic() < cleanup:
                try:
                    publish_feedback()
                    rclpy.spin_once(peer, timeout_sec=0.005)
                except Exception:
                    pass
                time.sleep(0.01)
        if ipc:
            ipc.close()
        for name, process in reversed(children):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        if native_pid and Path('/proc', str(native_pid)).exists():
            event('cleanup_orphan_detected', pid=native_pid)
            try:
                if Path('/proc', str(native_pid), 'exe').resolve() == node_path.resolve():
                    os.kill(native_pid, signal.SIGKILL)
            except FileNotFoundError:
                pass
            result = 'FAIL'
            error = error or 'orphan native required test cleanup'
        if temporary_home:
            temporary_home.cleanup()
        for stream in handles:
            stream.close()
        frames_log.close()
        state_log.close()
        attitude_stop.set()
        attitude_thread.join(timeout=2)
        if attitude_thread.is_alive():
            raise RuntimeError("synthetic attitude source did not stop")
        peer.destroy_node()
        rclpy.try_shutdown()
    # 此处仅是该主机、该合成图像参数的测量，绝不当成OrangePi/真实相机性能。
    distinct = {(o['source'], o['reset'], o['sequence']): o for o in observations if o['tracking']}
    unique = list(distinct.values())
    span = max((x['received'] for x in unique), default=0) - min(
        (x['received'] for x in unique), default=0
    )
    latencies = [(o['receive_unix_ns'] - o['stamp_ns']) / 1e9 for o in unique]
    resource_summary = {}
    for name, pid in cpu_baseline:
        rows = [r for r in resources if r['name'] == name and r['pid'] == pid]
        if len(rows) > 1:
            duration = rows[-1]['received'] - rows[0]['received']
            resource_summary[name] = dict(
                pid=pid,
                max_rss_bytes=max(r['rss_bytes'] for r in rows),
                cpu_percent_one_core=100
                * (rows[-1]['cpu_ticks'] - rows[0]['cpu_ticks'])
                / os.sysconf('SC_CLK_TCK')
                / duration,
            )
    hb = [
        r
        for r in heartbeats
        if nav_at
        and nav_at
        <= r['received']
        <= next((c['received'] for c in commands if c['command'] == 400), time.monotonic())
    ]
    flight_samples = [s for s in samples if ramp_at and ramp_at <= s['received'] and s['armed']]
    hold_samples = [
        s
        for s in samples
        if hold_at
        and hold_at
        <= s['received']
        <= next(
            (
                e['monotonic']
                for e in events
                if e['event'] in ('normal_land_request', 'camera_stream_stopped')
            ),
            0,
        )
    ]
    metrics = dict(
        requested_raw_attitude_hz=args.attitude_hz,
        raw_attitude_samples=len(attitude_stamps),
        effective_raw_attitude_hz=(
            (len(attitude_stamps) - 1) / (attitude_stamps[-1][0] - attitude_stamps[0][0])
            if len(attitude_stamps) > 1
            else None
        ),
        altitude_model=dict(
            max_overshoot_m=max((s['height'] - 1.5 for s in flight_samples), default=None),
            native_height_gate_time_from_ramp_s=hold_at - ramp_at if hold_at and ramp_at else None,
            hold_max_abs_error_m=max((abs(s['height'] - 1.5) for s in hold_samples), default=None),
            hold_mean_abs_error_m=(
                sum(abs(s['height'] - 1.5) for s in hold_samples) / len(hold_samples)
                if hold_samples
                else None
            ),
        ),
        image_to_observation_latency_s=dict(
            max=max(latencies, default=None),
            mean=sum(latencies) / len(latencies) if latencies else None,
        ),
        p95_image_to_observation_latency_s=(
            sorted(latencies)[min(len(latencies) - 1, math.floor(0.95 * len(latencies)))]
            if latencies
            else None
        ),
        requested_synthetic_resolution=[640, 480],
        requested_synthetic_fps=120,
        images_published=image_count,
        unique_tracking_frames=len(unique),
        effective_new_tracking_fps=(len(unique) - 1) / span if span > 0 else None,
        max_heartbeat_receive_gap_s=max(
            (b['received'] - a['received'] for a, b in zip(hb, hb[1:])), default=None
        ),
        max_control_period_s=max(
            (
                r['max_cycle_period_s']
                for r in states
                if isinstance(r.get('max_cycle_period_s'), (float, int))
            ),
            default=None,
        ),
        resources=resource_summary,
        real_camera_metrics='UNMEASURED',
        horizontal_dynamics='fixed XY: no drift/stability performance claim',
    )
    report = dict(
        result=result,
        error=error,
        scenario=args.scenario,
        checks=checks,
        events=events,
        metrics=metrics,
        environment='Ubuntu22.04 x86_64 Humble real DDS localhost; synthetic Image and PX4; actual standby.supervise with hardware-only boundaries substituted',
        px4_firmware=False,
        px4_dds_received_by_synthetic_peer=bool(heartbeats and targets),
        physical_flight=False,
        node_sha256=sha(node_path),
        vision_sha256=sha(vision_path),
        mission_sha256=sha(installed_mission),
        source_head=subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True
        ).strip(),
    )
    for name, data in [
        ('camera-observations', observations),
        ('mission-envelopes', envelopes),
        ('navigation', navs),
        ('resources', resources),
        ('events', events),
        ('mission-status', mission_status),
        (
            'native-gate',
            [
                dict(
                    sequence=m.sequence,
                    stamp=m.stamp.sec * 10**9 + m.stamp.nanosec,
                    phase=m.phase,
                    reason=m.reason,
                    feedback=m.feedback_current,
                    publication=m.publication_current,
                    allowed=m.navigation_allowed,
                    height=finite_or_none(m.relative_height_m),
                    vz=finite_or_none(m.vz_ned),
                    settled=m.target_height_settled,
                    ground_qualified=m.ground_reference_qualified,
                    received_native=m.native_monotonic_ns,
                )
                for m in native
            ],
        ),
    ]:
        (output / (name + '.json')).write_text(
            json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        )
    report['harness_sha256'] = sha(__file__)
    report['supervisor_fixture_sha256'] = sha(ROOT / 'tests/fixtures/step5_supervisor.py')
    (output / 'result.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if result == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Isolated Humble regression: synthetic PX4 and images, production FC01 nodes.

The plant reuses the pinned PX4 PositionControl/Takeoff library from step 1.
The hardware camera case opens a deliberately absent device. Other cases feed
real vision_image_node over DDS without a USB device. No Agent, UART or PX4
firmware process is used, and every control endpoint is localhost-only.
"""
import argparse
from array import array
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from ros_step1_takeoff_position import (ALTITUDE, ANCHOR, FIXTURE, ROOT, Plant, digest,
                                        packet_values, write_jsonl)
from px4_takeoff_position_model import Px4PositionControl


TRACK_CASES = ('old_frame', 'duplicate_frame', 'stream_loss', 'lock_loss', 'xy_reset',
               'rc_takeover', 'px4_local_stale')
CASES = ('unplugged', 'hold', 'waypoint', 'wrong_origin', 'waypoint_silence', 'handoff', *TRACK_CASES)
DISTURBANCE_CASES = ('unplugged', 'hold', 'handoff', *TRACK_CASES)
WAYPOINT = (ANCHOR[0] + .38, ANCHOR[1] + .20)
FINAL_Z = ANCHOR[2] - ALTITUDE
ORIGIN = 'px4_local:1000:0:0:0:0:0'
IMAGE_TOPIC = '/position_step2_fixture/image'


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--build-evidence', type=Path, required=True)
    parser.add_argument('--px4-model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=CASES, required=True)
    parser.add_argument('--domain', type=int, default=218)
    args = parser.parse_args()
    if not 101 <= args.domain <= 232:
        parser.error('use an isolated domain between 101 and 232')
    args.prefix = args.prefix.resolve()
    args.output = args.output.resolve()
    args.px4_model = args.px4_model.resolve()
    return args


def isolated_environment(domain):
    for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_STATIC_PEERS'):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT / 'tests/fixtures/step5_camera_local_shm.xml'))


def check_build(args):
    executable = args.prefix / 'jikl_fc01_system/lib/jikl_fc01_system/fc01_system_node'
    proof = json.loads(args.build_evidence.read_text())
    sources = {ROOT / path: sha for path, sha in proof['production_sha256'].items()}
    if ROOT / 'native/src/jikl_fc01_system/src/fc01_system_node.cpp' not in sources:
        raise ValueError('build receipt does not bind native node source')
    sources[executable] = proof['binary_sha256']
    for package, names in {
        'fc01_mission': ('fc01_mission/mission_node.py', 'fc01_mission/px4_state_bridge.py'),
        'fc01_bringup': ('launch/robot.launch.py', 'launch/horizontal_first_test.launch.py'),
    }.items():
        for name in names:
            source = ROOT / 'target/src' / package / name
            dest = args.prefix / package
            dest /= ('lib/python3.10/site-packages' if package == 'fc01_mission'
                     else 'share/' + package)
            for path in (source, dest / name):
                sources[path] = digest(source)
    for package, name in (
        ('fc01_vision', 'config/vision.yaml'),
        ('fc01_mission', 'config/platform.yaml'),
        ('fc01_mission', 'config/mission.yaml'),
        ('fc01_mission', 'launch/mission.launch.py'),
    ):
        source = ROOT / 'target/src' / package / name
        sources[source] = digest(source)
        sources[args.prefix / package / 'share' / package / name] = sources[source]
    for path, sha in sources.items():
        if digest(path) != sha:
            raise ValueError(f'build/source identity mismatch: {path}')
    camera = args.prefix / 'fc01_vision/lib/fc01_vision/vision_image_node'
    v4l2 = args.prefix / 'fc01_vision/lib/fc01_vision/fc01_vision_node'
    for path in (camera, v4l2, args.px4_model, args.px4_model.with_suffix('.json'), Path(__file__), FIXTURE,
                 ROOT / 'tests/fixtures/step5_camera_local_shm.xml'):
        sources[path.resolve()] = digest(path)
    return executable, camera, sources


class Scenario:
    def __init__(self, args, executable, camera, private):
        import numpy as np
        import rclpy
        import yaml
        from builtin_interfaces.msg import Time
        from jikl_fc01_system.msg import (HorizontalVelocityCommand, NativeFlightState,
                                          NavigationCommand, TargetObservation)
        from px4_msgs.msg import (BatteryStatus, FailsafeFlags, ManualControlSetpoint,
                                  ManualControlSwitches, OffboardControlMode, TrajectorySetpoint,
                                  VehicleAngularVelocity, VehicleAttitude, VehicleAttitudeSetpoint,
                                  VehicleCommand, VehicleCommandAck, VehicleLandDetected,
                                  VehicleLocalPosition, VehicleStatus)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        self.args, self.executable, self.camera, self.private = args, executable, camera, private
        self.yaml, self.rclpy, self.Time, self.Image = yaml, rclpy, Time, Image
        self.socket_path = private / 'control.sock'
        self.children, self.files = {}, []
        self.model = Px4PositionControl(args.px4_model)
        self.plant = Plant(self.model)
        self.last_step = time.monotonic()
        self.next_feedback = self.next_ipc = self.next_image = 0.
        self.image_state = 'off' if args.scenario == 'unplugged' else 'blank'
        if args.scenario in TRACK_CASES:
            self.image_state = 'target'
        self.image_stamp, self.image_count = 0, 0
        self.first_old_replay_stamp = None
        self.local_position_enabled = True
        shape = (480, 640, 3)
        blank = np.zeros(shape, dtype=np.uint8)
        red_circle = blank.copy()
        y, x = np.ogrid[:shape[0], :shape[1]]
        red_circle[(x - 395) ** 2 + (y - 240) ** 2 <= 23 ** 2] = (0, 0, 255)
        self.blank = array('B', blank.tobytes())
        self.red_circle = array('B', red_circle.tobytes())
        self.feedback, self.targets, self.heartbeats, self.events, self.native_states = [], [], [], [], []
        self.observations, self.envelopes, self.navs, self.commands = [], [], [], []
        self.camera_reports, self.mission_reports = [], []
        self.state, self.native = {}, None
        self.sequence, self.buffer, self.ipc = 0, b'', None
        self.authorized_at = self.fault_at = self.landing_at = self.ack = None
        self.cleaning = False
        rclpy.init(args=[])
        self.peer = rclpy.create_node('fc01_position_step2_fixture')
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.feedback_types = dict(
            vehicle_status_v1=VehicleStatus, vehicle_land_detected=VehicleLandDetected,
            vehicle_local_position=VehicleLocalPosition, vehicle_attitude=VehicleAttitude,
            vehicle_angular_velocity=VehicleAngularVelocity, failsafe_flags=FailsafeFlags,
            battery_status=BatteryStatus, manual_control_setpoint=ManualControlSetpoint,
            manual_control_switches=ManualControlSwitches, vehicle_command_ack=VehicleCommandAck)
        self.feedback_pubs = {name: self.peer.create_publisher(kind, '/fmu/out/' + name, sensor)
                              for name, kind in self.feedback_types.items()}
        self.synthetic_nav = None
        if args.scenario == 'waypoint_silence':
            self.synthetic_nav = self.peer.create_publisher(NavigationCommand, '/fc01/navigation/command', reliable)
        self.NavigationCommand = NavigationCommand
        self.synthetic_nav_on = False
        self.synthetic_nav_sequence = 0
        self.image_pub = self.peer.create_publisher(Image, IMAGE_TOPIC, image_qos)
        readers = ((TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.receive_target, reliable),
                   (OffboardControlMode, '/fmu/in/offboard_control_mode', self.receive_mode, reliable),
                   (VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', self.reject_attitude, reliable),
                   (VehicleCommand, '/fmu/in/vehicle_command', self.receive_command, reliable),
                   (NativeFlightState, '/fc01/native/flight_state', self.receive_native, sensor),
                   (TargetObservation, '/fc01/vision/target_observation', self.receive_observation, sensor),
                   (NavigationCommand, '/fc01/navigation/command', self.receive_navigation, reliable),
                   (HorizontalVelocityCommand, '/fc01/vision/horizontal_command', self.receive_envelope, reliable),
                   (String, '/fc01/vision/camera_status', self.receive_camera, sensor),
                   (String, '/fc01/mission/sensors/status', self.receive_mission, sensor))
        self.readers = [self.peer.create_subscription(*items) for items in readers]

    def event(self, name, **details):
        now = time.monotonic()
        item = dict(event=name, received_at=now, height_m=self.plant.height,
                    position_ned_m=list(self.plant.position), velocity_ned_mps=list(self.plant.velocity),
                    **details)
        self.events.append(item)
        print(json.dumps(item, ensure_ascii=False, allow_nan=False), flush=True)
        return now

    @staticmethod
    def horizontal_velocity(wire):
        return any(value is not None and abs(value) > 1e-6 for value in wire['velocity'][:2])

    def receive_target(self, msg):
        item = dict(received_at=time.monotonic(), timestamp_us=msg.timestamp,
                    position=packet_values(msg.position), velocity=packet_values(msg.velocity),
                    acceleration=packet_values(msg.acceleration), yaw=packet_values([msg.yaw])[0],
                    yaw_rate=packet_values([msg.yawspeed])[0], height_m=self.plant.height)
        self.targets.append(item)
        if self.cleaning:
            return
        if self.fault_at is None and (not self.plant.armed or not self.plant.offboard):
            raise RuntimeError('trajectory reached DDS before independent manual arm and Offboard')
        if self.plant.height < 1.30 and self.authorized_at is None and self.fault_at is None:
            if item['position'][:2] not in ([None, None], list(ANCHOR[:2])):
                raise RuntimeError('takeoff left the immutable ground anchor')
            if item['velocity'][:2] != [None, None]:
                raise RuntimeError('takeoff sent a mission horizontal velocity')

    def receive_mode(self, msg):
        if not msg.position or any((msg.velocity, msg.acceleration, msg.attitude, msg.body_rate,
                                   msg.thrust_and_torque, msg.direct_actuator)):
            raise RuntimeError('new mode changed Offboard heartbeat control type')
        self.heartbeats.append(dict(received_at=time.monotonic(), timestamp_us=msg.timestamp))
        if self.plant.xy_valid and not self.plant.armed and not self.plant.offboard:
            self.plant.offboard_lost = False

    @staticmethod
    def reject_attitude(_msg):
        raise RuntimeError('position mission emitted a competing attitude setpoint')

    def receive_command(self, msg):
        if self.cleaning:
            return
        plant = self.plant
        ground_disarm = msg.command == 400 and msg.param1 == 0. and plant.armed and plant.landed
        ground_release = msg.command == 176 and msg.param1 == 1. and msg.param2 == 7. and not plant.armed and plant.landed
        if not (ground_disarm or ground_release) or (self.landing_at is None and self.fault_at is None):
            raise RuntimeError(f'unexpected VehicleCommand {msg.command} ({msg.param1}, {msg.param2})')
        if any(item['command'] == msg.command for item in self.commands):
            raise RuntimeError('duplicate ground command')
        self.commands.append(dict(command=msg.command, p1=msg.param1, p2=msg.param2,
                                  timestamp_us=msg.timestamp, received_at=time.monotonic()))
        self.ack = (time.monotonic() + .10, msg.command)

    def receive_native(self, msg):
        self.native = msg
        self.native_states.append(dict(received_at=time.monotonic(), stamp_ns=self.stamp_ns(msg.stamp),
                                       phase=msg.phase, reason=msg.reason, liftoff=msg.liftoff_confirmed,
                                       settled=msg.target_height_settled, allowed=msg.navigation_allowed,
                                       relative_height_m=packet_values([msg.relative_height_m])[0]))
        if msg.navigation_allowed and self.authorized_at is None:
            if not msg.liftoff_confirmed or not msg.target_height_settled or msg.phase != 'HEIGHT_HOLD':
                raise RuntimeError('native granted mission before real liftoff and settling')
            self.authorized_at = self.event('native_stable_1_5m_authorization', stamp_ns=self.stamp_ns(msg.stamp))

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 10**9 + stamp.nanosec

    def receive_camera(self, msg):
        self.camera_reports.append(dict(received_at=time.monotonic(), report=json.loads(msg.data)))

    def receive_mission(self, msg):
        self.mission_reports.append(dict(received_at=time.monotonic(), report=json.loads(msg.data)))

    def receive_observation(self, msg):
        self.observations.append(dict(received_at=time.monotonic(), stamp_ns=self.stamp_ns(msg.stamp),
                                      sequence=msg.sequence, tracking=msg.tracking, reset=msg.reset_counter,
                                      calibrated=msg.calibration_valid, fields=msg.valid_fields,
                                      position=packet_values(msg.position)))

    def receive_envelope(self, msg):
        command = msg.command
        self.envelopes.append(dict(received_at=time.monotonic(), stamp_ns=self.stamp_ns(command.stamp),
                                   valid=command.valid, v=packet_values([command.vx, command.vy, command.vz]),
                                   attitude_sample_stamp_ns=msg.attitude_sample_stamp_ns, origin=msg.origin_id))

    def receive_navigation(self, msg):
        self.navs.append(dict(received_at=time.monotonic(), stamp_ns=self.stamp_ns(msg.stamp),
                              valid=msg.valid, mode=msg.mode, position=packet_values(msg.position),
                              origin=msg.frame_id))

    def send_feedback(self):
        stamp = self.peer.get_clock().now().nanoseconds // 1000
        plant = self.plant
        x, y, z = plant.position
        vx, vy, vz = plant.velocity
        fields = dict(
            vehicle_status_v1=dict(system_id=1, component_id=1, arming_state=2 if plant.armed else 1,
                nav_state=14 if plant.offboard else 15, armed_time=10 if plant.armed else 0,
                latest_arming_reason=2 if plant.armed else 0, pre_flight_checks_pass=True,
                power_input_valid=True, can_set_nav_states_mask=1 << 15),
            vehicle_land_detected=dict(landed=plant.landed, at_rest=plant.landed,
                ground_contact=plant.landed, maybe_landed=plant.landed),
            vehicle_local_position=dict(timestamp_sample=stamp-2000, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz,
                z_deriv=vz, heading=plant.heading, heading_good_for_control=True,
                xy_valid=plant.xy_valid, v_xy_valid=plant.xy_valid, z_valid=True, v_z_valid=True,
                xy_reset_counter=plant.xy_reset, ref_timestamp=1000, dist_bottom_valid=True,
                dist_bottom=plant.height + .10, dist_bottom_var=.0025, dist_bottom_sensor_bitfield=1),
            vehicle_attitude=dict(timestamp_sample=stamp-2000, q=[1., 0., 0., 0.]),
            vehicle_angular_velocity=dict(timestamp_sample=stamp-2000, xyz=[0., 0., 0.]),
            failsafe_flags=dict(local_position_invalid=not plant.xy_valid,
                local_velocity_invalid=not plant.xy_valid, offboard_control_signal_lost=plant.offboard_lost),
            battery_status=dict(id=1, connected=True, voltage_v=7.6, remaining=1.),
            manual_control_setpoint=dict(timestamp_sample=stamp-2000, valid=True,
                data_source=self.feedback_types['manual_control_setpoint'].SOURCE_RC, throttle=-1.,
                sticks_moving=plant.sticks_moving, roll=.2 if plant.sticks_moving else 0.),
            manual_control_switches=dict(timestamp_sample=stamp-2000, mode_slot=plant.mode_slot,
                arm_switch=plant.arm_switch, kill_switch=3, return_switch=3, loiter_switch=3, offboard_switch=3),
        )
        for name, payload in fields.items():
            if name == 'vehicle_local_position' and not self.local_position_enabled:
                continue
            self.feedback_pubs[name].publish(self.feedback_types[name](timestamp=stamp, **payload))
        if self.synthetic_nav_on:
            self.synthetic_nav_sequence += 1
            self.synthetic_nav.publish(self.NavigationCommand(
                stamp=self.peer.get_clock().now().to_msg(), sequence=self.synthetic_nav_sequence,
                source_id='test-only-waypoint-source', frame_id='px4_local_ned:' + ORIGIN,
                mode=self.NavigationCommand.CRUISE, valid=True, position_valid=True, position=[*WAYPOINT, 0.]))
        self.feedback.append(dict(received_at=time.monotonic(), stamp_us=stamp,
                                  position_ned_m=list(plant.position), velocity_ned_mps=list(plant.velocity),
                                  altitude_m=plant.height, thrust_ned=plant.thrust, landed=plant.landed,
                                  offboard=plant.offboard, armed=plant.armed,
                                  local_position_published=self.local_position_enabled))

    def send_image(self):
        if self.image_state == 'off':
            return
        frame = self.Image(height=480, width=640, encoding='bgr8', step=640*3)
        frame.header.frame_id = 'synthetic_optical'
        if self.image_state != 'replay':
            self.image_stamp = self.peer.get_clock().now().nanoseconds
        elif self.first_old_replay_stamp is None:
            self.first_old_replay_stamp = self.image_stamp
        frame.header.stamp = self.Time(sec=self.image_stamp//10**9, nanosec=self.image_stamp%10**9)
        frame.data = self.red_circle if self.image_state in ('target', 'replay') else self.blank
        self.image_pub.publish(frame)
        self.image_count += 1

    def request(self, op):
        self.sequence += 1
        self.ipc.sendall(f'{self.sequence} {op}\n'.encode())

    def read_ipc(self, now):
        if self.ipc is None and self.socket_path.exists():
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                probe.connect(str(self.socket_path))
                self.ipc = probe.dup()
            self.ipc.setblocking(False)
        if self.ipc is None:
            return
        if now >= self.next_ipc:
            self.request('STATUS')
            self.next_ipc = now + .08
        while True:
            try:
                data = self.ipc.recv(262144)
            except BlockingIOError:
                break
            if not data:
                if self.cleaning:
                    break
                raise RuntimeError('native IPC closed prematurely')
            self.buffer += data
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                state = json.loads(line)
                if 'phase' in state:
                    if state['phase'] != self.state.get('phase'):
                        self.event('native_phase', phase=state['phase'], reason=state.get('reason'),
                                   local_fresh=state.get('local_fresh'))
                    self.state = state

    def step(self):
        now = time.monotonic()
        dt, self.last_step = min(.025, now - self.last_step), now
        if not self.cleaning:
            target = self.targets[-1] if self.targets and now-self.targets[-1]['received_at'] <= .25 else None
            self.plant.advance(dt, target)
            for name, process in self.children.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited during scenario ({process.returncode})')
        if now >= self.next_feedback:
            self.send_feedback()
            self.next_feedback = now + .02
        if now >= self.next_image and not self.cleaning:
            self.send_image()
            self.next_image = now + 1./45
        if self.ack and now >= self.ack[0]:
            stamp = self.peer.get_clock().now().nanoseconds//1000
            self.feedback_pubs['vehicle_command_ack'].publish(self.feedback_types['vehicle_command_ack'](
                timestamp=stamp, command=self.ack[1], result=0, target_system=1, target_component=191,
                from_external=False))
            self.ack = None
        self.rclpy.spin_once(self.peer, timeout_sec=.002)
        self.read_ipc(now)

    def until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.step()
            if predicate():
                return
        raise RuntimeError(f'{label}: phase={self.state.get("phase")}, reason={self.state.get("reason")}, '
                           f'h={self.plant.height:.3f}, mission={self.mission_reports[-1:]}')

    def observe(self, seconds):
        stop = time.monotonic() + seconds
        self.until(lambda: time.monotonic() >= stop, seconds + .5, 'observation interval')

    def spawn(self, name, command):
        stream = (self.args.output / (name + '.log')).open('w')
        self.files.append(stream)
        self.children[name] = subprocess.Popen([str(piece) for piece in command], stdout=stream,
                                               stderr=subprocess.STDOUT, start_new_session=True)

    def configure(self):
        out = self.args.output
        has_waypoint = self.args.scenario not in ('unplugged', 'hold')
        origin = ('px4_local:999:0:0:0:0:0' if self.args.scenario == 'wrong_origin' else ORIGIN)
        config = self.yaml.safe_load(FIXTURE.read_text())
        params = config['fc01_system']['ros__parameters']
        params.update(px4_takeoff_position_hold=True, px4_position_mission_enabled=True,
                      visual_navigation_enabled=True, px4_horizontal_enabled=True,
                      px4_position_waypoint_enabled=has_waypoint,
                      px4_position_waypoint_origin_id=origin if has_waypoint else '',
                      px4_position_waypoint_north_m=WAYPOINT[0], px4_position_waypoint_east_m=WAYPOINT[1],
                      visual_max_radius=2., visual_max_speed=.5, visual_max_acceleration=.6,
                      visual_max_yaw_rate=.35,
                      socket_path=str(self.socket_path), fc01_entry_version='position-step2-synthetic')
        (out / 'runtime.yaml').write_text(self.yaml.safe_dump(config))
        camera_cfg = self.yaml.safe_load((ROOT / 'target/src/fc01_vision/config/vision.yaml').read_text())
        camera_cfg['/**']['ros__parameters'].update(
            use_camera_info=False, image_topic=IMAGE_TOPIC, target_shape='circle', use_pnp=False,
            device_path=str(self.private / 'nonexistent-usb-camera'), camera_fx=466.7, camera_fy=466.7,
            calibration_confirmed=True, calibration_id='FIXTURE_ONLY_PINHOLE', image_rectified_confirmed=True,
            extrinsics_confirmed=True, extrinsics_id='FIXTURE_ONLY_IDENTITY', target_model_confirmed=True,
            target_model_id='FIXTURE_ONLY_CIRCLE', capture_latency_confirmed=True, capture_latency_bound_s=.01,
            visualization=False, show_tuning_window=False, publish_debug_image=False)
        (out / 'vision.yaml').write_text(self.yaml.safe_dump(camera_cfg))
        mission = self.yaml.safe_load((ROOT / 'target/src/fc01_mission/config/mission.yaml').read_text())
        mission['fc01_mission']['ros__parameters'].update(
            control_enabled=True, engage_mode='track', horizontal_navigation_mode='waypoint' if has_waypoint else 'hold',
            position_takeoff_anchor_authority=True,
            waypoint_alignment_confirmed=has_waypoint, waypoint_alignment_origin_id=origin if has_waypoint else '',
            navigation_waypoint_x=WAYPOINT[1], navigation_waypoint_y=WAYPOINT[0],
            camera_extrinsics_confirmed=True, camera_extrinsics_id='FIXTURE_ONLY_IDENTITY',
            camera_pos_x=0., camera_pos_y=0., camera_pos_z=0., standoff_calibration_confirmed=True,
            standoff_calibration_id='FIXTURE_ONLY_STANDOFF', standoff_m=1., yaw_rate_gain=0.,
            navigation_command_topic='/fc01/navigation/command', vision_control_topic='/fc01/vision/velocity_command_v2')
        (out / 'mission.yaml').write_text(self.yaml.safe_dump(mission))

    def run(self):
        self.observe(.60)
        if self.peer.get_node_names() != [self.peer.get_name()]:
            raise RuntimeError('isolated ROS domain is occupied')
        self.configure()
        out, plant = self.args.output, self.plant
        self.spawn('native', [self.executable, '--ros-args', '--disable-external-lib-logs',
                             '--params-file', out/'runtime.yaml'])
        production_camera = self.args.scenario == 'unplugged'
        arguments = ['control_enabled:=true', f'vision_config:={out / "vision.yaml"}']
        # The silence case uses the sole synthetic waypoint writer to isolate absence
        # of any withdrawal packet. The remaining cases use the actual mission.
        if production_camera:
            self.spawn('bringup', ['ros2', 'launch', 'fc01_bringup', 'horizontal_first_test.launch.py',
                                   f'site_mission_config:={out / "mission.yaml"}', *arguments])
        elif self.synthetic_nav is None:
            self.spawn('bringup', ['ros2', 'launch', 'fc01_bringup', 'robot.launch.py',
                                   'engage_mode:=track', 'start_camera:=false',
                                   f'mission_config:={out / "mission.yaml"}', *arguments])
            self.spawn('camera', [self.camera, '--ros-args', '--params-file', out/'vision.yaml'])
        self.until(lambda: self.native and self.native.ready_wait_arm, 18., 'fresh READY')
        if self.synthetic_nav is None:
            self.until(lambda: self.mission_reports and self.camera_reports, 8., 'actual auxiliary node receipt')
        if production_camera and (self.image_count != 0 or
                                  self.camera_reports[-1]['report'].get('reason') !=
                                  'CAMERA_FRAME_STALE_OR_DISCONNECTED'):
            raise RuntimeError('absent camera was not independently reported by the V4L2 source')
        if self.args.scenario in TRACK_CASES:
            self.until(lambda: any(row['tracking'] and row['calibrated'] for row in self.observations) and
                       any(row['report']['target_locked'] for row in self.mission_reports),
                       4., 'ground image detection and continuous target lock')
            self.event('target_detected_on_ground_before_arm')
        else:
            self.observe(.30)
            if any(row['tracking'] for row in self.observations):
                raise RuntimeError('blank image/absent camera fabricated a visible target')
        if self.targets or any(row['valid'] for row in self.envelopes + self.navs):
            raise RuntimeError('mission or trajectory output before arm')
        if self.state.get('takeoff_position_anchor_ned') != list(ANCHOR):
            raise RuntimeError('step 1 ground anchor not observed')
        self.event('current_ready_wait_arm', anchor_ned_m=list(ANCHOR))
        self.observe(.22)
        plant.arm_switch = 1
        self.until(lambda: self.state.get('phase') == 'RC_ARMED_WAIT_NAVIGATION', 3., 'physical RC edge')
        self.event('synthetic_manual_rc_arm_edge')
        plant.armed = True
        self.until(lambda: self.state.get('reason') == 'RC_ARMED_WAIT_CH6_NAVIGATION', 3., 'independent arm')
        plant.mode_slot = 1
        self.event('synthetic_rc_selects_navigation_slot', slot=plant.mode_slot)
        self.observe(.20)
        if self.targets:
            raise RuntimeError('trajectory emitted during delayed Offboard transition')
        plant.offboard = True
        self.event('synthetic_px4_new_offboard')
        if self.args.scenario in DISTURBANCE_CASES:
            self.until(lambda: plant.height >= .03, 7., 'early rise')
            plant.velocity[0] += .22
            plant.velocity[1] -= .14
            self.event('early_horizontal_disturbance')
        self.until(lambda: self.authorized_at is not None, 21., 'measured liftoff and stable 1.5m')
        self.check_preheight()
        self.check_ownership()
        if self.args.scenario in ('unplugged', 'hold', 'wrong_origin'):
            self.check_hold()
        elif self.args.scenario == 'waypoint':
            self.check_waypoint()
        elif self.args.scenario == 'waypoint_silence':
            self.synthetic_nav_on = True
            self.check_waypoint()
            self.check_waypoint_silence()
        elif self.args.scenario == 'handoff':
            self.check_waypoint()
            self.image_state = 'target'
            self.event('camera_target_appears_after_valid_waypoint')
            self.check_track()
        else:
            self.check_track()
            self.inject_fault()
        if self.state.get('phase') == 'SAFETY':
            return
        self.complete_landing()

    def check_preheight(self):
        # Use the native source stamp, not cross-topic callback arrival order.
        grant_stamp = next(row['stamp_ns'] for row in self.native_states if row['allowed'])
        early = [row for row in self.targets if row['timestamp_us']*1000 < grant_stamp]
        if not any(row['position'][:2] == list(ANCHOR[:2]) for row in early):
            raise RuntimeError('no anchored ascent setpoints before mission authorization')
        if any(row['position'][:2] not in ([None, None], list(ANCHOR[:2])) or
               row['velocity'][:2] != [None, None] for row in early):
            raise RuntimeError('native motion left the ground anchor before the grant')
        if any(row['valid'] and row['stamp_ns'] < grant_stamp for row in self.navs + self.envelopes):
            raise RuntimeError('mission produced a valid horizontal request before the grant')
        if self.args.scenario in TRACK_CASES and not any(row['tracking'] and row['calibrated'] and
                                                        row['stamp_ns'] < grant_stamp
                                                        for row in self.observations):
            raise RuntimeError('preheight visual test never received a valid target')
        if self.args.scenario in DISTURBANCE_CASES:
            impulse_at = next(row['received_at'] for row in self.events
                              if row['event'] == 'early_horizontal_disturbance')
            corrected = next((row for row in self.feedback if row['received_at'] > impulse_at and
                              row['altitude_m'] < 1.30 and row['thrust_ned'] and
                              sum((row['position_ned_m'][axis]-ANCHOR[axis])*row['thrust_ned'][axis]
                                  for axis in (0, 1)) < -1e-4), None)
            if corrected is None:
                raise RuntimeError('position hold never counteracted the early lateral disturbance')
            self.event('px4_model_counteracts_preheight_drift', sample=corrected)
        self.event('preheight_motion_limited_to_ground_anchor', native_targets=len(early))

    def check_ownership(self):
        publishers = {topic: self.peer.count_publishers('/fmu/in/' + topic) for topic in
                      ('trajectory_setpoint', 'offboard_control_mode', 'vehicle_attitude_setpoint')}
        if publishers != dict(trajectory_setpoint=1, offboard_control_mode=1, vehicle_attitude_setpoint=0):
            raise RuntimeError(f'unexpected Offboard ownership {publishers}')
        self.event('exclusive_px4_trajectory_and_heartbeat', publishers=publishers)

    def check_height(self, start, end=None):
        end = time.monotonic() if end is None else end
        active = [row for row in self.targets if start <= row['received_at'] <= end]
        if not active or any(row['position'][2] is None or abs(row['position'][2]-FINAL_Z) > 1e-5
                             for row in active):
            raise RuntimeError('post-height mission overrode the native 1.5m position target')
        if abs(self.plant.height-ALTITUDE) > .15:
            raise RuntimeError('plant left 1.5m flight envelope')
        return active

    def check_hold(self):
        # A blank image or a physically unavailable camera gives no navigation waypoint by itself.
        hold_start = time.monotonic()
        self.observe(3.)
        active = self.check_height(hold_start)
        if any(row['position'][:2] != list(ANCHOR[:2]) or row['velocity'][:2] != [None, None]
               for row in active):
            raise RuntimeError('unconfigured search moved the hold point')
        radius = math.dist(self.plant.position[:2], ANCHOR[:2])
        if radius > .07 or any(item['valid'] and item['mode'] == 2 for item in self.navs):
            raise RuntimeError('hold drifted or fabricated a waypoint')
        if any(row['valid'] for row in self.envelopes):
            raise RuntimeError('no-target hold produced an active visual command')
        self.event('no_target_hold', lateral_error_m=radius, camera=self.camera_reports[-1]['report'])

    def check_waypoint(self):
        started = time.monotonic()
        self.until(lambda: any(item['valid'] and item['mode'] == 2 and
                               item['position'][:2] == list(WAYPOINT) for item in self.navs),
                   4., 'configured waypoint authorized')
        self.until(lambda: math.dist(self.plant.position[:2], WAYPOINT) < .13, 8., 'plant follows waypoint')
        self.check_height(started)
        if any(item['valid'] for item in self.envelopes) or any(item['tracking'] for item in self.observations):
            raise RuntimeError('waypoint was attributed to a fabricated visual lock')
        self.event('configured_waypoint_moves_without_target', goal_ned_m=list(WAYPOINT))

    def check_waypoint_silence(self):
        self.fault_at = self.event('sole_waypoint_writer_stops_messages')
        self.synthetic_nav_on = False
        self.until(lambda: self.state.get('phase') == 'DESCENT' and self.targets and
                   (self.targets[-1]['velocity'][2] or 0.) > 0., 1.25, 'expired waypoint descent')
        if self.state.get('reason') != 'VISUAL_RECOVERY_HORIZONTAL_WAYPOINT_INPUT_INVALID':
            raise RuntimeError('descent was not attributed to the expired waypoint')
        descent = next(row for row in self.targets if row['received_at'] >= self.fault_at and
                       (row['velocity'][2] or 0.) > 0.)
        self.observe(.40)
        for row in self.targets:
            if row['timestamp_us'] < descent['timestamp_us']:
                continue
            if row['position'][2] is not None or row['velocity'][:2] != [None, None]:
                raise RuntimeError('expired waypoint target returned after the first descent packet')
            if all(component is not None for component in row['position'][:2]) and math.dist(
                row['position'][:2], descent['position'][:2]
            ) > 1e-6:
                raise RuntimeError('downward recovery resumed progression of the old waypoint')
        self.event('waypoint_expiry_withdraws_and_descends', first_descent_wire=descent,
                   first_descent_wire_s=descent['received_at']-self.fault_at,
                   reason=self.state.get('reason'))

    def check_track(self):
        # A valid envelope by itself is not movement evidence. Require a changed native wire and actual XY response.
        started = time.monotonic()
        self.until(lambda: any(row['valid'] for row in self.envelopes), 4., 'real image-to-mission lock')
        self.until(lambda: any(not row['valid'] for row in self.navs if row['received_at'] >= started),
                   3., 'waypoint channel withdrawn by lock')
        position = tuple(self.plant.position[:2])
        self.until(lambda: self.targets and self.targets[-1]['velocity'][0] is not None and
                   self.targets[-1]['velocity'][1] is not None and
                   math.hypot(*self.targets[-1]['velocity'][:2]) > .04 and
                   self.targets[-1]['position'][:2] == [None, None] and
                   math.dist(position, self.plant.position[:2]) > .06,
                   5., 'motion from track')
        active = self.check_height(started)
        valid_envelope = next(row for row in reversed(self.envelopes) if row['valid'])
        settled_nav = [row for row in self.navs if row['received_at'] >= started]
        if not settled_nav or settled_nav[-1]['valid']:
            raise RuntimeError('waypoint channel stayed valid after stable tracking began')
        self.event('track_withdraws_waypoint_and_moves', intent=valid_envelope, wire=active[-1])

    def inject_fault(self):
        fault = self.args.scenario
        old_stamp = self.image_stamp
        self.fault_at = self.event('inject_' + fault, last_image_stamp_ns=old_stamp)
        frames = self.image_count
        no_resume = fault in ('xy_reset', 'rc_takeover', 'px4_local_stale')
        safety_at = None
        if fault in ('old_frame', 'duplicate_frame'):
            self.image_state = 'replay'
        if fault == 'old_frame':
            cutoff_ns = self.peer.get_clock().now().nanoseconds - 600_000_000
            self.image_stamp = next(row['stamp_ns'] for row in reversed(self.observations)
                                    if row['tracking'] and row['stamp_ns'] <= cutoff_ns)
        replay_stamp = self.image_stamp
        if fault == 'stream_loss':
            self.image_state = 'off'
        elif fault == 'lock_loss':
            self.image_state = 'blank'
        elif fault == 'xy_reset':
            self.plant.xy_reset += 1
        elif fault == 'px4_local_stale':
            self.local_position_enabled = False
        elif fault == 'rc_takeover':
            self.plant.sticks_moving = True
            self.until(lambda: self.state.get('phase') == 'DESCENT' and self.targets and
                       (self.targets[-1]['velocity'][2] or 0.) > 0.,
                       2., 'controlled descent before pilot Offboard exit')
            exit_stamp_us = self.peer.get_clock().now().nanoseconds // 1000
            self.event('synthetic_px4_confirms_pilot_left_offboard', stamp_us=exit_stamp_us)
            self.plant.offboard = False
        if no_resume:
            self.until(lambda: self.state.get('phase') == 'SAFETY', 2., 'fault latches Safety')
            safety_at = time.monotonic()
            expected = {
                'xy_reset': ('PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID',),
                'rc_takeover': ('OPERATOR_TAKEOVER_LATCHED',),
                'px4_local_stale': ('DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE',
                                    'Z_VZ_STALE_OR_INVALID',
                                    'PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID'),
            }[fault]
            reason = self.state.get('reason', '').removeprefix('PUBLISHER_BOUNDARY_')
            if reason not in expected or (fault == 'px4_local_stale' and self.state.get('local_fresh') is not False):
                raise RuntimeError(f'expected {fault} diagnosis {expected}, got {self.state.get("reason")}')
            self.observe(.20)
            sent = (len(self.targets), len(self.heartbeats))
            if fault == 'rc_takeover' and any(row['timestamp_us'] > exit_stamp_us + 100_000 for row in
                                              self.targets[:sent[0]] + self.heartbeats[:sent[1]]):
                raise RuntimeError('position targets or heartbeats continued after PX4 left Offboard')
            self.event('restore_inputs_while_armed', fault=fault)
            self.local_position_enabled = True
            self.plant.sticks_moving = False
            self.plant.offboard = True
            self.observe(.55)
            if sent != (len(self.targets), len(self.heartbeats)):
                raise RuntimeError('target or heartbeat continued during armed Safety')
        else:
            self.until(lambda: bool(self.envelopes) and not self.envelopes[-1]['valid'],
                       1.25, 'mission invalidates old track command')
            self.until(lambda: self.state.get('phase') in ('DESCENT', 'RECOVERY_DESCENT', 'SAFETY'),
                       2., 'native protection after losing active tracking')
            if self.state['phase'] != 'SAFETY':
                self.until(lambda: self.targets and (self.targets[-1]['velocity'][2] or 0.) > 0.,
                           1., 'actual native downward wire')
                for row in self.targets:
                    if (row['received_at'] >= self.fault_at and (row['velocity'][2] or 0.) > 0.
                            and self.horizontal_velocity(row)):
                        raise RuntimeError('native downward protection retained an old horizontal velocity')
            self.observe(.25)
            if fault in ('old_frame', 'duplicate_frame') and (
                    self.image_count <= frames or self.image_stamp != replay_stamp):
                raise RuntimeError('expected repeated publication with the original acquisition stamp')
            if fault == 'old_frame' and not any(row['received_at'] >= self.fault_at and
                                                row['report'].get('reason') == 'SOURCE_TIME_STALE_OR_INVALID'
                                                for row in self.camera_reports):
                raise RuntimeError('delayed image was not rejected as stale')
            if fault == 'duplicate_frame':
                duplicate = next((row for row in self.camera_reports if row['received_at'] >= self.fault_at and
                                  row['report'].get('reason') == 'DUPLICATE_FRAME'), None)
                if duplicate is None or any(row['received_at'] >= duplicate['received_at'] and row['tracking']
                                            for row in self.observations):
                    raise RuntimeError('repeated acquisition stamp continued to qualify as fresh tracking')
            if fault == 'stream_loss' and self.image_count != frames:
                raise RuntimeError('camera stream did not stop')
            if fault == 'lock_loss' and not any(not row['tracking'] and row['stamp_ns'] > old_stamp
                                                for row in self.observations):
                raise RuntimeError('fresh blank frames did not revoke target observations')
            last = self.targets[-1]
            if self.state['phase'] != 'SAFETY' and (last['velocity'][2] or 0.) <= 0.:
                raise RuntimeError('native retained old track output in visible protection state')
        safety = next(row for row in self.events if row['event'] == 'native_phase' and
                      row['received_at'] >= self.fault_at and
                      row['phase'] in ('DESCENT', 'RECOVERY_DESCENT', 'SAFETY'))
        descent_wire = next((row for row in self.targets if row['received_at'] >= self.fault_at and
                             (row['velocity'][2] or 0.) > 0.), None)
        if descent_wire and any(self.horizontal_velocity(row) for row in self.targets
                                if row['timestamp_us'] >= descent_wire['timestamp_us']):
            raise RuntimeError('native trajectory resumed active horizontal intent after descent began')
        invalid_intent = next((row for row in self.envelopes if row['received_at'] >= self.fault_at and
                               not row['valid']), None)
        last_motion = next((row for row in reversed(self.targets)
                            if (row['velocity'][2] or 0.) <= 0. and self.horizontal_velocity(row)), None)
        self.event('fault_revokes_active_motion', phase=self.state['phase'], reason=self.state.get('reason'),
                   phase_transition_s=safety['received_at'] - self.fault_at,
                   safety_latched_s=safety_at - self.fault_at if safety_at is not None else None,
                   first_descent_wire_s=descent_wire['received_at'] - self.fault_at if descent_wire else None,
                   first_invalid_intent_s=invalid_intent['received_at'] - self.fault_at if invalid_intent else None,
                   last_active_motion_wire_s=last_motion['received_at'] - self.fault_at if last_motion else None,
                   synthetic_frames_after_fault=self.image_count-frames,
                   replayed_capture_stamp_ns=self.first_old_replay_stamp)
        if no_resume:
            return
        restored = self.event('restore_fresh_synthetic_images')
        self.image_state = 'target'
        self.observe(.55)
        revived = [row for row in self.targets if row['received_at'] >= restored and
                   (row['velocity'][2] or 0.) <= 0. and self.horizontal_velocity(row)]
        if revived or self.state.get('phase') in ('ASCEND', 'HEIGHT_HOLD'):
            raise RuntimeError('fresh imagery revived revoked mission motion')

    def complete_landing(self):
        self.landing_at = self.event('existing_automatic_protection_or_normal_landing')
        if self.state.get('phase') == 'HEIGHT_HOLD':
            self.request('LAND')
        self.until(lambda: self.commands and self.commands[0]['command'] == 400, 20.,
                   'current landing evidence before Disarm')
        self.until(lambda: self.ack is None, 2., 'Disarm ACK sent')
        self.observe(.20)
        if len(self.commands) != 1:
            raise RuntimeError('ACK replaced independent disarmed status')
        self.plant.armed = False
        self.until(lambda: len(self.commands) == 2 and self.commands[-1]['command'] == 176,
                   4., 'release after disarmed evidence')
        self.until(lambda: self.ack is None, 2., 'release ACK sent')
        self.plant.offboard = False
        self.plant.arm_switch, self.plant.mode_slot = 3, 6
        self.event('disarmed_landed_exit')

    def close(self):
        self.cleaning = True
        forced = []
        if self.children:
            self.plant.armed = self.plant.offboard = False
            self.plant.arm_switch, self.plant.mode_slot = 3, 6
            self.plant.position[:] = ANCHOR
            self.plant.velocity[:] = [0., 0., 0.]
            for process in self.children.values():
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            until = time.monotonic() + 7.
            while time.monotonic() < until and any(p.poll() is None for p in self.children.values()):
                self.send_feedback()
                self.rclpy.spin_once(self.peer, timeout_sec=.015)
            for name, process in self.children.items():
                if process.poll() is None:
                    forced.append(name)
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
        for stream in self.files:
            stream.close()
        if self.ipc:
            self.ipc.close()
        self.model.close()
        self.peer.destroy_node()
        self.rclpy.shutdown()
        if forced:
            raise RuntimeError(f'forced child cleanup: {forced}')
        if 'native' in self.children and self.children['native'].returncode != 0:
            raise RuntimeError(f'native shutdown returned {self.children["native"].returncode}')

    def evidence(self):
        for key in ('feedback', 'targets', 'heartbeats', 'events', 'native_states', 'observations',
                    'envelopes', 'navs', 'commands', 'camera_reports', 'mission_reports'):
            write_jsonl(self.args.output / (key + '.jsonl'), getattr(self, key))


def main():
    args = arguments()
    isolated_environment(args.domain)
    executable, camera, inputs = check_build(args)
    args.output.mkdir(parents=True, exist_ok=False)
    error = None
    with tempfile.TemporaryDirectory(prefix='fc01-pos-step2-') as path:
        run = Scenario(args, executable, camera, Path(path))
        try:
            run.run()
        except Exception as exc:
            error = repr(exc)
        finally:
            try:
                run.close()
            except Exception as exc:
                error = f'{error or ""} cleanup={exc!r}'
        run.evidence()
    if any(digest(path) != value for path, value in inputs.items()):
        error = f'{error or ""} source/binary changed during run'
    report = dict(result='PASS' if error is None else 'FAIL', error=error, scenario=args.scenario,
                  scope='x86_64 Humble localhost DDS; synthetic PX4 and image with pinned controller/plant; no real aircraft',
                  camera=('production V4L2 node, deliberately absent device' if args.scenario == 'unplugged'
                          else 'none, sole synthetic waypoint writer' if args.scenario == 'waypoint_silence'
                          else 'synthetic ROS Image into real vision_image_node'),
                  artifact_sha256={str(path): digest for path, digest in inputs.items()},
                  node_sha256=inputs[executable], domain=args.domain,
                  frames_injected=run.image_count, controls={name: process.returncode
                      for name, process in run.children.items()}, events=run.events)
    (args.output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2)+'\n')
    print(json.dumps(dict(result=report['result'], error=error, scenario=args.scenario), ensure_ascii=False))
    return bool(error)


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Exercise the actual FC01 ROS node with a synthetic three-axis PX4 peer.

Only typed trajectory packets drive the point-mass model. An optional
shared library executes the actual PX4 PositionControl and takeoff ramp.
The peer does not run the full firmware, Agent, serial devices or camera.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / 'tests/fixtures/stage3_runtime_20261114.yaml'
ANCHOR = (2.25, -1.75, 2.)
ALTITUDE = 1.5
FAULTS = ('kill', 'offboard_loss', 'dds_loss', 'rc_takeover', 'xy_invalid', 'xy_reset',
          'no_confirm', 'late_slot')


def clamp(value, limit):
    return min(limit, max(-limit, value))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Plant:
    """State and integration in NED metres and m/s, with a rigid floor."""

    def __init__(self, controller=None):
        self.controller = controller
        self.position = list(ANCHOR)
        self.velocity = [0., 0., 0.]
        self.acceleration = [0., 0., 0.]
        self.thrust = None
        self.heading = 0.
        self.armed = False
        self.offboard = False
        self.arm_switch = 3
        self.mode_slot = 6
        self.kill_switch = 3
        self.sticks_moving = False
        self.xy_valid = True
        self.xy_reset = 0
        self.offboard_lost = True

    @property
    def height(self):
        return ANCHOR[2] - self.position[2]

    @property
    def landed(self):
        return self.height <= .012 and abs(self.velocity[2]) <= .08

    def advance(self, dt, target):
        if dt <= 0.:
            return
        self.thrust = None
        if self.controller and (target or not self.offboard or not self.armed):
            target_fields = target or dict(position=[None]*3, velocity=[None]*3,
                                           acceleration=[None]*3, yaw=None, yaw_rate=None)
            self.thrust = self.controller.update(
                dt, self.position, self.velocity, self.acceleration, self.heading,
                target_position=target_fields['position'], target_velocity=target_fields['velocity'],
                target_acceleration=target_fields['acceleration'], target_yaw=target_fields['yaw'],
                target_yaw_speed=target_fields['yaw_rate'], armed=self.armed, offboard=self.offboard,
                airborne=not self.landed)
        self.acceleration = [0., 0., 0.]
        if not (self.armed and self.offboard):
            return
        if self.thrust is not None:
            # PX4 thrust is a normalized NED vector. The point mass scales it
            # by gravity/hover_thrust and adds gravity and simple viscous drag.
            gravity = 9.80665
            acceleration = [gravity*(self.thrust[i]/self.controller.hover_thrust+(1. if i==2 else 0.))
                            - .18*self.velocity[i] for i in range(3)]
            self.integrate(dt, acceleration)
            return
        for axis in range(3):
            position = target['position'][axis] if target else None
            velocity = target['velocity'][axis] if target else None
            feedforward = target['acceleration'][axis] if target else None
            if position is not None:
                limit = .6 if axis == 2 else .7
                velocity_sp = clamp(2.0 * (position - self.position[axis]), limit)
                if velocity is not None:
                    velocity_sp += velocity
                demand = 3.6 * (velocity_sp - self.velocity[axis])
            elif velocity is not None:
                demand = 3.6 * (velocity - self.velocity[axis])
            else:
                demand = feedforward if feedforward is not None else 0.
            self.acceleration[axis] = clamp(demand, 2.5)
        self.integrate(dt, self.acceleration)

    def integrate(self, dt, acceleration):
        self.acceleration = acceleration
        for axis in range(3):
            self.velocity[axis] += self.acceleration[axis] * dt
            self.position[axis] += self.velocity[axis] * dt
        if self.position[2] >= ANCHOR[2]:
            self.position[2] = ANCHOR[2]
            self.velocity[2] = 0.
            self.acceleration[2] = 0.


def packet_values(values):
    return [float(value) if math.isfinite(value) else None for value in values]


def write_jsonl(path, rows):
    path.write_text(''.join(json.dumps(row, allow_nan=False) + '\n' for row in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-evidence', type=Path, required=True,
                        help='JSON with actual binary_sha256 and production_sha256')
    parser.add_argument('--scenario', choices=('disturbance', *FAULTS), default='disturbance')
    parser.add_argument('--domain', type=int, default=193)
    parser.add_argument('--px4-model', type=Path, help='optional test PX4 position-control shared library')
    args = parser.parse_args()
    if not 101 <= args.domain <= 232:
        parser.error('use an isolated domain in 101..232')
    binary = args.node.resolve()
    build = json.loads(args.build_evidence.read_text())
    inputs = {ROOT / name: sha for name, sha in build['production_sha256'].items()}
    if ROOT / 'native/src/jikl_fc01_system/src/fc01_system_node.cpp' not in inputs:
        parser.error('build evidence must bind the actual node source')
    inputs.update({binary: build['binary_sha256'], FIXTURE: digest(FIXTURE),
                   Path(__file__).resolve(): digest(Path(__file__))})
    if args.px4_model:
        args.px4_model = args.px4_model.resolve()
        for path in (args.px4_model, args.px4_model.with_suffix('.json'),
                     Path(__file__).resolve().with_name('px4_takeoff_position_model.py')):
            inputs[path] = digest(path)
    mismatches = [str(path) for path, sha in inputs.items() if digest(path) != sha]
    if mismatches:
        parser.error('build identity mismatch: ' + repr(mismatches))

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for key in ('ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
                'ROS_STATIC_PEERS', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from jikl_fc01_system.msg import NativeFlightState, NavigationCommand, HorizontalVelocityCommand
    from px4_msgs.msg import (BatteryStatus, FailsafeFlags, ManualControlSetpoint, ManualControlSwitches,
        OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleAttitudeSetpoint,
        VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    import yaml

    rclpy.init(args=[])
    peer = Node('fc01_takeoff_position_synthetic_peer')
    sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    sink = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    topics = {'vehicle_status_v1': VehicleStatus, 'vehicle_local_position': VehicleLocalPosition,
              'vehicle_land_detected': VehicleLandDetected, 'vehicle_attitude': VehicleAttitude,
              'failsafe_flags': FailsafeFlags, 'battery_status': BatteryStatus,
              'manual_control_setpoint': ManualControlSetpoint,
              'manual_control_switches': ManualControlSwitches, 'vehicle_command_ack': VehicleCommandAck}
    sources = {name: peer.create_publisher(kind, '/fmu/out/' + name, sensor)
               for name, kind in topics.items()}
    waypoint_source = peer.create_publisher(NavigationCommand, '/fc01/navigation/command', 1)
    velocity_source = peer.create_publisher(HorizontalVelocityCommand, '/fc01/vision/horizontal_command', 1)
    model = None
    if args.px4_model:
        from px4_takeoff_position_model import Px4PositionControl
        model = Px4PositionControl(args.px4_model)
    plant = Plant(model)
    last = next_feedback = next_status = time.monotonic()
    telemetry, targets, heartbeats, events = [], [], [], []
    native = None
    state = {}
    phase = None
    process = ipc = None
    buffer = b''
    request_sequence = feedback_sequence = 0
    cleaning = False
    feedback_enabled = True
    offboard_at = liftoff_at = disturbance_at = yaw_at = metric_end = fault_at = None
    landing_at = None
    command_records = []
    due_ack = None
    error = None

    def event(label, **fields):
        item = dict(event=label, monotonic_s=time.monotonic(), height_m=plant.height,
                    position_ned_m=list(plant.position), velocity_ned_mps=list(plant.velocity),
                    **fields)
        events.append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)
        return item['monotonic_s']

    def receive_target(msg):
        item = dict(received_at=time.monotonic(), timestamp_us=msg.timestamp,
                    position=packet_values(msg.position), velocity=packet_values(msg.velocity),
                    acceleration=packet_values(msg.acceleration), jerk=packet_values(msg.jerk),
                    yaw=packet_values([msg.yaw])[0], yaw_rate=packet_values([msg.yawspeed])[0])
        targets.append(item)
        if cleaning:
            return
        if fault_at is None and (not plant.armed or not plant.offboard):
            raise RuntimeError('trajectory before independent armed and Offboard facts')
        xy = item['position'][:2]
        if xy != [None, None]:
            if xy != list(ANCHOR[:2]):
                raise RuntimeError('trajectory moved the NED origin: ' + repr(item))
            if landing_at is None and fault_at is None and (
                    plant.landed or plant.height < params['aa_liftoff_height_m']):
                raise RuntimeError('anchored XY before independent airborne evidence: ' + repr(item))
        if xy == [None, None] and (item['velocity'][:2] != [None, None] or
                                   item['acceleration'][:2] != [0., 0.]):
            raise RuntimeError('horizontal correction before the liftoff gate')

    def receive_heartbeat(msg):
        if not msg.position or any((msg.velocity, msg.acceleration, msg.attitude, msg.body_rate,
                                   msg.thrust_and_torque, msg.direct_actuator)):
            raise RuntimeError('wrong heartbeat control mode')
        heartbeats.append(dict(received_at=time.monotonic(), timestamp_us=msg.timestamp))
        if plant.xy_valid and not plant.armed and not plant.offboard:
            plant.offboard_lost = False

    def receive_attitude(_msg):
        raise RuntimeError('position mode published an attitude setpoint')

    def receive_command(msg):
        nonlocal due_ack
        if cleaning:
            return
        disarm = msg.command == 400 and msg.param1 == 0 and plant.armed and plant.landed
        release = (msg.command == 176 and msg.param1 == 1 and msg.param2 == 7 and
                   not plant.armed and plant.landed)
        if landing_at is None or not (disarm or release):
            raise RuntimeError('unexpected VehicleCommand ' + str(msg))
        if any(row['command'] == msg.command for row in command_records):
            raise RuntimeError('repeated landing/release command ' + str(msg))
        row = dict(command=msg.command, p1=msg.param1, p2=msg.param2, stamp_us=msg.timestamp)
        command_records.append(row)
        event('wire_command', **row)
        due_ack = (time.monotonic()+.15, msg.command)

    def receive_native(msg):
        nonlocal native, liftoff_at
        native = msg
        if msg.liftoff_confirmed and liftoff_at is None:
            liftoff_at = event('native_liftoff_evidenced', controller_pid=msg.controller_pid,
                               observed_source_monotonic_ns=msg.native_monotonic_ns,
                               observed_vertical_sample_us=msg.local_sample_stamp_us)

    peer.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', receive_target, sink)
    peer.create_subscription(OffboardControlMode, '/fmu/in/offboard_control_mode', receive_heartbeat, sink)
    peer.create_subscription(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint',
                             receive_attitude, sink)
    peer.create_subscription(VehicleCommand, '/fmu/in/vehicle_command', receive_command, sink)
    peer.create_subscription(NativeFlightState, '/fc01/native/flight_state', receive_native, sensor)

    def publish():
        nonlocal feedback_sequence
        feedback_sequence += 1
        stamp = peer.get_clock().now().nanoseconds // 1000
        x, y, z = plant.position
        vx, vy, vz = plant.velocity
        feedback = dict(
            vehicle_status_v1=VehicleStatus(
                timestamp=stamp, system_id=1, component_id=1,
                arming_state=2 if plant.armed else 1, nav_state=14 if plant.offboard else 15,
                armed_time=10 if plant.armed else 0, latest_arming_reason=2 if plant.armed else 0,
                pre_flight_checks_pass=True, power_input_valid=True, can_set_nav_states_mask=1 << 15),
            vehicle_land_detected=VehicleLandDetected(timestamp=stamp, landed=plant.landed,
                at_rest=plant.landed, ground_contact=plant.landed, maybe_landed=plant.landed),
            vehicle_local_position=VehicleLocalPosition(
                timestamp=stamp, timestamp_sample=stamp-2000, x=x, y=y, z=z,
                vx=vx, vy=vy, vz=vz, z_deriv=vz, heading=plant.heading, heading_good_for_control=True,
                xy_valid=plant.xy_valid, v_xy_valid=plant.xy_valid, z_valid=True, v_z_valid=True,
                xy_reset_counter=plant.xy_reset, ref_timestamp=1000,
                dist_bottom_valid=True, dist_bottom=plant.height+.10, dist_bottom_var=.0025,
                dist_bottom_sensor_bitfield=VehicleLocalPosition.DIST_BOTTOM_SENSOR_RANGE),
            vehicle_attitude=VehicleAttitude(timestamp=stamp, timestamp_sample=stamp-2000,
                q=[math.cos(plant.heading/2),0.,0.,math.sin(plant.heading/2)]),
            failsafe_flags=FailsafeFlags(timestamp=stamp, local_position_invalid=not plant.xy_valid,
                local_velocity_invalid=not plant.xy_valid, offboard_control_signal_lost=plant.offboard_lost),
            battery_status=BatteryStatus(timestamp=stamp,id=1,connected=True,voltage_v=7.6,remaining=1.),
            manual_control_setpoint=ManualControlSetpoint(timestamp=stamp,timestamp_sample=stamp-2000,
                valid=True, data_source=ManualControlSetpoint.SOURCE_RC, throttle=-1.,
                sticks_moving=plant.sticks_moving, roll=.2 if plant.sticks_moving else 0.),
            manual_control_switches=ManualControlSwitches(timestamp=stamp,timestamp_sample=stamp-2000,
                mode_slot=plant.mode_slot, arm_switch=plant.arm_switch, kill_switch=plant.kill_switch,
                return_switch=3, loiter_switch=3, offboard_switch=3))
        for name, message in feedback.items():
            sources[name].publish(message)
        if not cleaning:
            waypoint = NavigationCommand()
            waypoint.sequence = feedback_sequence
            waypoint.stamp = peer.get_clock().now().to_msg()
            waypoint.source_id = 'takeoff-isolated-waypoint-peer'
            waypoint.frame_id = 'px4_local_ned:px4_local:1000:0:0:0:0:0'
            waypoint.valid = True
            waypoint.mode = 2
            waypoint.position_valid = True
            waypoint.position = [ANCHOR[0]+1., ANCHOR[1]+.5, ANCHOR[2]-ALTITUDE]
            waypoint_source.publish(waypoint)
            envelope = HorizontalVelocityCommand()
            envelope.origin_id = 'px4_local:1000:0:0:0:0:0'
            envelope.attitude_sample_stamp_ns = (stamp-2000)*1000
            command = envelope.command
            command.stamp = waypoint.stamp
            command.sequence = feedback_sequence
            command.source_id = waypoint.source_id
            command.frame_id = 'body_frd'
            command.valid = True
            command.vx, command.vy, command.wz = .3, .1, .2
            velocity_source.publish(envelope)
        telemetry.append(dict(monotonic_s=time.monotonic(), stamp_us=stamp,
            position_ned_m=list(plant.position), velocity_ned_mps=list(plant.velocity),
            acceleration_ned_mps2=list(plant.acceleration), height_m=plant.height,
            xy_error_m=math.hypot(x-ANCHOR[0], y-ANCHOR[1]), measured_heading_rad=plant.heading,
            stage=phase, target=targets[-1] if targets else None, px4_thrust_ned=plant.thrust,
            liftoff_confirmed=native.liftoff_confirmed if native else False,
            armed=plant.armed, offboard=plant.offboard, landed=plant.landed, cleanup=cleaning))

    def step():
        nonlocal last, next_feedback, next_status, ipc, buffer, request_sequence, state, phase, due_ack
        now = time.monotonic()
        dt = min(.03, now-last)
        last = now
        target = targets[-1] if targets and now-targets[-1]['received_at'] <= .25 else None
        if not cleaning:
            plant.advance(dt, target)
        if feedback_enabled and now >= next_feedback:
            publish()
            next_feedback = now+.05
        if due_ack and now >= due_ack[0]:
            stamp = peer.get_clock().now().nanoseconds//1000
            sources['vehicle_command_ack'].publish(VehicleCommandAck(timestamp=stamp,
                command=due_ack[1], result=0, target_system=1, target_component=191,
                from_external=False))
            event('ack_independent_of_arming_state', command=due_ack[1], armed=plant.armed)
            due_ack = None
        rclpy.spin_once(peer, timeout_sec=.002)
        if process and process.poll() is not None and not cleaning:
            raise RuntimeError('native process exited ' + str(process.returncode))
        if ipc is None and socket_path.exists():
            ipc = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            ipc.connect(str(socket_path))
            ipc.setblocking(False)
        if ipc:
            if now >= next_status:
                request_sequence += 1
                ipc.sendall(f'{request_sequence} STATUS\n'.encode())
                next_status = now+.05
            while True:
                try:
                    response = ipc.recv(262144)
                except BlockingIOError:
                    break
                if not response:
                    if cleaning:
                        break
                    raise RuntimeError('native IPC closed unexpectedly')
                buffer += response
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    state = json.loads(line)
                    if state.get('phase') != phase:
                        phase = state.get('phase')
                        event('native_phase', phase=phase, reason=state.get('reason'))

    def until(condition, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            step()
            if condition():
                return
        raise RuntimeError(label+' timed out: '+repr((phase,state.get('reason'),
                                                      plant.height, targets[-1:] or None)))

    def observe_for(seconds, label):
        deadline = time.monotonic()+seconds
        until(lambda: time.monotonic() >= deadline, seconds+1., label)

    def cleanup():
        nonlocal feedback_enabled
        if process is None or process.poll() is not None:
            return
        plant.armed = plant.offboard = False
        plant.arm_switch, plant.mode_slot, plant.kill_switch = 3, 6, 3
        plant.xy_valid = True
        feedback_enabled = True
        plant.position[:] = ANCHOR
        plant.velocity[:] = [0., 0., 0.]
        observe_for(.5, 'simulated cleanup ground')
        process.send_signal(signal.SIGINT)
        deadline = time.monotonic()+5.
        while process.poll() is None and time.monotonic()<deadline:
            publish()
            rclpy.spin_once(peer, timeout_sec=.01)
            time.sleep(.02)
        if process.poll() is None:
            raise RuntimeError('native process did not exit after SIGINT')

    with tempfile.TemporaryDirectory(prefix='fc01-s1-hold-') as tmp, (output/'native.log').open('w') as log:
        socket_path = Path(tmp)/'control.sock'
        cfg = yaml.safe_load(FIXTURE.read_text())
        params = cfg['fc01_system']['ros__parameters']
        params.update(px4_takeoff_position_hold=True, visual_navigation_enabled=False,
                      px4_horizontal_enabled=False, socket_path=str(socket_path))
        config = output/'runtime.yaml'
        config.write_text(yaml.safe_dump(cfg, sort_keys=False))
        argv = [str(binary),'--ros-args','--disable-external-lib-logs','--params-file',str(config)]
        try:
            deadline = time.monotonic()+.7
            while time.monotonic()<deadline:
                rclpy.spin_once(peer, timeout_sec=.01)
            if peer.get_node_names() != [peer.get_name()]:
                raise RuntimeError('test domain is occupied: '+repr(peer.get_node_names()))
            process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
            until(lambda: native and native.ready_wait_arm, 14., 'fresh ground READY')
            if not (native.feedback_current and native.publication_current and not native.armed and
                    native.ground_reference_valid and native.ground_reference_qualified and
                    not native.navigation_allowed and native.reference_z_ned == ANCHOR[2] and
                    native.target_height_m == ALTITUDE and not targets and heartbeats):
                raise RuntimeError('invalid READY or early trajectory publication')
            if peer.count_publishers('/fmu/in/trajectory_setpoint') != 1 or peer.count_publishers(
                    '/fmu/in/vehicle_attitude_setpoint'):
                raise RuntimeError('nonexclusive trajectory publisher')
            if peer.count_subscribers('/fc01/navigation/command') or peer.count_subscribers(
                    '/fc01/vision/horizontal_command'):
                raise RuntimeError('takeoff mode subscribed to external motion intent')
            until(lambda: state.get('takeoff_position_anchor_ned') == list(ANCHOR),
                  .5, 'reported ground NED anchor agrees with telemetry')
            event('ready_ground_heartbeat_only', z0=native.reference_z_ned)
            # The production READY source watermark includes 100 ms of
            # clock-skew protection. Use a clearly later physical switch edge.
            observe_for(.2, 'arm after READY source watermark')
            plant.arm_switch = 1
            until(lambda: phase == 'RC_ARMED_WAIT_NAVIGATION', 2., 'human RC arm edge')
            plant.armed = True
            until(lambda: state.get('reason') == 'RC_ARMED_WAIT_CH6_NAVIGATION', 2., 'manual armed fact')
            if args.scenario == 'late_slot':
                observe_for(8.2, '8 second authorization window expiry')
                if phase != 'SAFETY' or state.get('reason') != 'RC_NAVIGATION_SELECTION_TIMEOUT':
                    raise RuntimeError('late navigation slot was not rejected by the eight-second gate')
            plant.mode_slot = 1
            observe_for(.18, 'navigation slot before Offboard feedback')
            if targets:
                raise RuntimeError('trajectory without independent Offboard confirmation')
            if args.scenario in ('no_confirm','late_slot'):
                if args.scenario == 'late_slot':
                    plant.offboard = True
                    event('late_offboard_after_expired_slot')
                    observe_for(.4, 'late Offboard feedback cannot revive expired authorization')
                until(lambda:phase == 'SAFETY', 4., 'expired RC authorization')
                if targets:
                    raise RuntimeError('expired authorization produced a trajectory')
                if args.scenario == 'no_confirm' and state.get('reason') != 'RC_NAVIGATION_CONFIRMATION_TIMEOUT':
                    raise RuntimeError('missing Offboard confirmation did not expire by its own gate')
                event('admission_denied', reason=state.get('reason'))
            else:
                plant.offboard = True
                offboard_at = event('independent_armed_offboard_status')
                until(lambda: plant.height >= .025, 5., 'vertical motion before liftoff proof')
                if liftoff_at is not None or any(row['position'][0] is not None for row in targets):
                    raise RuntimeError('horizontal loop started before the independent liftoff gate')
                # Displace the plant before liftoff is confirmed. Reanchoring
                # to the first airborne sample must fail the fixed wire XY check.
                plant.velocity[0] += .35
                plant.velocity[1] -= .2
                disturbance_at = event('external_horizontal_velocity_impulse')
                until(lambda: liftoff_at is not None, 9., 'liftoff proof')
                until(lambda: targets and targets[-1]['position'][:2] == list(ANCHOR[:2]),
                      1., 'anchor in actual DDS trajectory')
                if plant.height >= ALTITUDE-.25:
                    raise RuntimeError('no early anchored correction at observed height')
                if math.hypot(plant.position[0]-ANCHOR[0], plant.position[1]-ANCHOR[1]) < .03:
                    raise RuntimeError('fixture did not displace the plant before liftoff proof')
                if args.scenario in FAULTS:
                    observe_for(.35, 'active horizontal correction before fault')
                    fault_at = event('inject_'+args.scenario)
                    if args.scenario == 'kill':
                        plant.kill_switch = 1
                    elif args.scenario == 'offboard_loss':
                        plant.offboard = False
                    elif args.scenario == 'dds_loss':
                        feedback_enabled = False
                    elif args.scenario == 'rc_takeover':
                        plant.sticks_moving = True
                        until(lambda:phase == 'DESCENT', 2., 'controlled RC handover before mode exit')
                        plant.offboard = False
                    elif args.scenario == 'xy_reset':
                        plant.xy_reset += 1
                    elif args.scenario == 'xy_invalid':
                        plant.xy_valid = False
                    until(lambda:phase == 'SAFETY', 2., 'fault latch')
                    reason = state.get('reason')
                    expected = {
                        'xy_invalid': ('PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID',),
                        'xy_reset': ('PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID',),
                        'dds_loss': ('DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE',
                                     'PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID',
                                     'PUBLISHER_BOUNDARY_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE',
                                     'PUBLISHER_BOUNDARY_PX4_TAKEOFF_POSITION_FEEDBACK_OR_FRAME_INVALID'),
                    }.get(args.scenario, ('OPERATOR_TAKEOVER_LATCHED',))
                    if reason not in expected:
                        raise RuntimeError(f'fault reason {reason!r} is outside {expected!r}')
                    event('safety_latched', reason=reason)
                    observe_for(.25, 'drain in-flight setpoint packets')
                    before = (len(targets), len(heartbeats))
                    # A later healthy input must not resume this armed task.
                    plant.xy_valid = True
                    feedback_enabled = True
                    plant.offboard = True
                    plant.sticks_moving = False
                    plant.kill_switch = 3
                    observe_for(.6, 'latched output inhibition')
                    if phase != 'SAFETY' or before != (len(targets), len(heartbeats)):
                        raise RuntimeError('armed fault resumed output on a later healthy input')
                else:
                    yaw_at = event('heading_change_90deg_while_displaced', yaw_rad=math.pi/2)
                    plant.heading = math.pi/2
                    until(lambda:native and native.target_height_settled and phase=='HEIGHT_HOLD',
                          14., 'stable 1.5 m from current feedback')
                    event('native_1_5m_height_hold', navigation_allowed=native.navigation_allowed)
                    if native.navigation_allowed:
                        raise RuntimeError('external navigation obtained authority in position hold mode')
                    observe_for(3., 'steady XYZ observation')
                    metric_end = time.monotonic()
                    landing_at = event('request_native_landing')
                    request_sequence += 1
                    ipc.sendall(f'{request_sequence} LAND\n'.encode())
                    until(lambda: command_records and command_records[0]['command']==400,
                          19., 'actual ground-only disarm command after position descent')
                    until(lambda: due_ack is None, 1., 'disarm ACK while still armed')
                    observe_for(.20, 'ACK alone cannot complete disarm')
                    if len(command_records) != 1:
                        raise RuntimeError('mode release before independent disarmed status')
                    plant.armed = False
                    event('independent_disarmed_status')
                    until(lambda: len(command_records)==2 and command_records[-1]['command']==176,
                          2., 'actual ground-only PX4 mode release command')
                    until(lambda: due_ack is None, 1., 'mode release ACK')
                    plant.offboard = False
                    plant.arm_switch, plant.mode_slot = 3, 6
                    until(lambda: native and native.ready_wait_arm, 12., 'fresh READY after landing')
                    event('new_ready_after_landing')
        except Exception as exc:
            error = repr(exc)
        finally:
            cleaning = True
            try:
                cleanup()
            except Exception as exc:
                error = (error or '') + ' cleanup=' + repr(exc)
                if process and process.poll() is None:
                    process.kill()
                    process.wait(timeout=3.)
            if ipc:
                ipc.close()
            if model:
                model.close()
            peer.destroy_node()
            rclpy.shutdown()

    exit_code = process.returncode if process else None
    metrics = {}
    if args.scenario == 'disturbance' and metric_end is not None:
        metrics, passed = convergence(telemetry, targets, heartbeats, offboard_at, liftoff_at,
                                      disturbance_at, yaw_at, metric_end)
        if not passed:
            error = (error or '') + ' plant convergence assertion failed'
    if exit_code != 0:
        error = (error or '')+' native_exit='+str(exit_code)
    if any(digest(path) != sha for path, sha in inputs.items()):
        error = (error or '')+' input changed during run'
    report = dict(result='FAIL' if error else 'PASS', error=error, scenario=args.scenario,
        environment='ROS 2 Humble localhost, typed DDS synthetic peer, no firmware/Agent/UART/camera',
        disconnected_camera=True, domain=args.domain, node_sha256=inputs[binary], command=argv,
        source_git=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_sha256={str(path): sha for path, sha in inputs.items()}, native_exit=exit_code,
        anchor_ned_m=ANCHOR, target_height_m=ALTITUDE, metrics=metrics, events=events,
        setpoints=len(targets), heartbeats=len(heartbeats), commands=command_records,
        px4_position_model_sha256=inputs.get(args.px4_model),
        model_hover_thrust=model.hover_thrust if model else None,
        model=('PX4 PositionControl and TakeoffHandling with point-mass dynamics and rigid ground'
               if model else 'Synthetic NED double integrator with bounded PD acceleration and rigid ground'))
    write_jsonl(output/'feedback.jsonl', telemetry)
    write_jsonl(output/'wire_targets.jsonl', targets)
    write_jsonl(output/'wire_heartbeats.jsonl', heartbeats)
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2)+'\n')
    print(json.dumps(dict(result=report['result'], error=error, metrics=metrics), ensure_ascii=False))
    return 1 if error else 0


def convergence(telemetry, targets, heartbeats, offboard_at, liftoff_at, disturbance_at, yaw_at, end):
    disturbed = [row for row in telemetry if disturbance_at <= row['monotonic_s'] <= end]
    hold = [row for row in disturbed if row['monotonic_s'] >= end-2.]
    airborne = [row for row in telemetry if liftoff_at <= row['monotonic_s'] <= end]

    def with_anchor(row):
        return row['target'] and row['target']['position'][:2] == list(ANCHOR[:2])

    def inward(row, value):
        return sum((row['position_ned_m'][axis]-ANCHOR[axis])*row[value][axis] for axis in (0,1))

    correction = next((row for row in disturbed if with_anchor(row) and row['height_m'] < ALTITUDE
                       and inward(row, 'acceleration_ned_mps2') < -.01), None)
    after_yaw = next((row for row in disturbed if with_anchor(row) and row['monotonic_s'] >= yaw_at
                      and inward(row, 'velocity_ned_mps') < -.002), None)
    # Independent subscription callbacks can occur in either order. Pair the
    # target and heartbeat by their common wire stamp inside the active window.
    active = [item for item in targets if offboard_at <= item['received_at'] <= end]
    active_hb = [item for item in heartbeats if offboard_at <= item['received_at'] <= end]
    hb_stamps = {item['timestamp_us'] for item in active_hb}
    pairs = bool(active) and all(item['timestamp_us'] in hb_stamps for item in active[:-2])
    gaps = [b['received_at']-a['received_at'] for a, b in zip(active_hb, active_hb[1:])]
    max_heartbeat_gap = max(gaps, default=None)
    final_target = active[-1]['position'] if active else None
    metrics = dict(peak_horizontal_m=max(row['xy_error_m'] for row in disturbed),
        hold_max_horizontal_error_m=max(row['xy_error_m'] for row in hold),
        hold_rms_horizontal_error_m=math.sqrt(sum(row['xy_error_m']**2 for row in hold)/len(hold)),
        height_overshoot_m=max(0.,max(row['height_m'] for row in airborne)-ALTITUDE),
        height_hold_max_error_m=max(abs(row['height_m']-ALTITUDE) for row in hold),
        first_correction_before_target=correction, first_return_after_yaw_change=after_yaw,
        active_target_heartbeat_pairs=pairs, max_heartbeat_gap_s=max_heartbeat_gap)
    passed = (pairs and max_heartbeat_gap is not None and max_heartbeat_gap < .3
              and final_target == [*ANCHOR[:2], ANCHOR[2]-ALTITUDE]
              and correction and after_yaw and metrics['peak_horizontal_m'] > .05 and
              metrics['hold_max_horizontal_error_m'] < .05 and metrics['height_hold_max_error_m'] < .15)
    return metrics, bool(passed)


if __name__ == '__main__':
    raise SystemExit(main())

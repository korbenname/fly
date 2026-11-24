#!/usr/bin/env python3
"""真实Humble任务收发：合成PX4/原生授权/目标连续帧，验证水平封套和撤销。"""
import argparse
import json
import math
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--decimated-attitude',action='store_true',
                        help='原生承认的原始姿态不在低频平台桥历史中，且旋转值不同')
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('must use isolated localhost ROS domain')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from jikl_fc01_system.msg import (PlatformState,NativeFlightState,NativeFlightSnapshot,TargetObservation,
                                      VelocityCommandV2,HorizontalVelocityCommand,NavigationCommand)
    from fc01_mission.mission_node import MissionNode
    rclpy.init(args=['--ros-args','-p','control_enabled:=true','-p','engage_mode:=track',
                    '-p','horizontal_navigation_mode:=waypoint',
                    '-p','waypoint_alignment_confirmed:=true',
                    '-p','waypoint_alignment_origin_id:=fixture-origin',
                    '-p','navigation_waypoint_x:=20.5','-p','navigation_waypoint_y:=10.0',
                    '-p','camera_extrinsics_confirmed:=true','-p','camera_extrinsics_id:=synthetic-mount',
                    '-p','standoff_calibration_confirmed:=true','-p','standoff_calibration_id:=synthetic-standoff',
                    '-p','standoff_m:=1.0',
                    '-p','vision_control_topic:=/step4/mission/v2',
                    '-p','horizontal_command_topic:=/step4/mission/horizontal',
                    '-p','navigation_command_topic:=/step4/mission/navigation'])
    source=Node('step4_mission_fixture');mission=MissionNode()
    executor=SingleThreadedExecutor();executor.add_node(source);executor.add_node(mission)
    sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    platform=source.create_publisher(PlatformState,'/fc01/platform/state',sensor)
    native=source.create_publisher(NativeFlightSnapshot,'/fc01/native/flight_snapshot',sensor)
    target=source.create_publisher(TargetObservation,'/fc01/vision/target_observation',sensor)
    commands,envelopes,navigation=[],[],[]
    source.create_subscription(VelocityCommandV2,'/step4/mission/v2',commands.append,10)
    source.create_subscription(HorizontalVelocityCommand,'/step4/mission/horizontal',envelopes.append,10)
    source.create_subscription(NavigationCommand,'/step4/mission/navigation',navigation.append,10)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    mode={'native_ready':False,'target':False,'target_valid':True,'target_source':'target-fixture'}
    seq=0;last_send=0.;checks=[];attitudes={}

    def send():
        nonlocal seq,last_send
        mono=time.monotonic()
        if mono-last_send<.025:return
        last_send=mono;seq+=1
        stamp=source.get_clock().now().to_msg();ns=(stamp.sec*10**9+stamp.nanosec)//1000*1000
        p=PlatformState();p.sequence=seq;p.stamp=stamp
        p.source_id='platform-fixture';p.origin_id='fixture-origin'
        p.frame_id='px4_local_enu';p.body_frame_id='base_link_flu'
        p.ready=True;p.reason='OK';p.valid_fields=15
        p.position=[20.,10.,3.];p.velocity=[0.,0.,0.]
        p.attitude_xyzw=[0.,float(np.sin(.2)),0.,float(np.cos(.2))]
        p.angular_velocity=[0.,0.,0.]
        p.local_receive_monotonic_ns=p.attitude_receive_monotonic_ns=p.angular_velocity_receive_monotonic_ns=time.monotonic_ns()
        p.local_timestamp_sample_us=p.attitude_timestamp_sample_us=p.angular_velocity_timestamp_sample_us=ns//1000
        p.position_stamp_ns=p.velocity_stamp_ns=p.attitude_stamp_ns=p.angular_velocity_stamp_ns=ns
        platform.publish(p)
        native_sample_ns=ns-5_000_000 if args.decimated_attitude else ns
        native_yaw=.2 if args.decimated_attitude else 0.
        # 原生20Hz/200Hz原始姿态和50Hz桥快照可引用不同帧，不能要求两边历史重合。
        ry=np.array([[math.cos(.4),0.,math.sin(.4)],[0.,1.,0.],[-math.sin(.4),0.,math.cos(.4)]])
        rz=np.array([[math.cos(native_yaw),-math.sin(native_yaw),0.],
                     [math.sin(native_yaw),math.cos(native_yaw),0.],[0.,0.,1.]])
        attitudes[native_sample_ns]=rz@ry
        n=NativeFlightState();n.sequence=seq;n.stamp=stamp;n.boot_id=boot
        n.source_id='native-fixture';n.controller_pid=17171;n.profile='attitude_altitude'
        n.native_monotonic_ns=n.local_receive_monotonic_ns=time.monotonic_ns()
        n.local_sample_stamp_us=n.local_stamp_us=n.status_stamp_us=n.land_stamp_us=ns//1000
        n.attitude_stamp_us=n.attitude_sample_stamp_us=native_sample_ns//1000
        n.feedback_current=n.publication_current=True
        n.armed=n.offboard=mode['native_ready'];n.landed=not mode['native_ready']
        n.rc_navigation_enabled=n.rc_manual_arm_proven=n.ground_reference_valid=n.ground_reference_qualified=True
        n.liftoff_confirmed=n.target_height_settled=n.navigation_allowed=mode['native_ready']
        n.phase='HEIGHT_HOLD' if mode['native_ready'] else 'RC_WAIT_ARM';n.reason='SYNTHETIC'
        n.rc_mode_slot=n.rc_navigation_mode_slot=1
        n.reference_z_ned=23.;n.relative_height_m=1.5;n.z_ned=21.5;n.vz_ned=0.
        n.target_height_m=1.5;n.height_tolerance_m=.1;n.speed_tolerance_mps=.15;n.settle_seconds=.8
        snapshot=NativeFlightSnapshot();snapshot.state=n;snapshot.origin_id=p.origin_id
        snapshot.attitude_receive_monotonic_ns=p.attitude_receive_monotonic_ns
        # ENU/FLU绕Y倾斜0.4rad，经NED/FRD基变换得到原始PX4四元数。
        # Hamilton乘法 q(NED←ENU) * q(ENU←FLU) * q(FLU←FRD)。
        cp,sp=math.cos(.2),math.sin(.2)
        cy,sy=math.cos(native_yaw/2),math.sin(native_yaw/2)
        x,y,z,w=-sp*sy,sp*cy,cp*sy,cp*cy
        a=math.sqrt(.5)
        snapshot.attitude_wxyz=[a*(w+z),a*(x+y),a*(x-y),a*(w-z)]
        native.publish(snapshot)
        if mode['target']:
            t=TargetObservation();t.sequence=seq;t.stamp=stamp;t.source_id=mode['target_source']
            t.frame_id=t.body_frame_id='body_frd'
            t.initialized=t.calibration_valid=True;t.tracking=mode['target_valid']
            t.valid_fields=t.POSITION;t.position=[2.,.3,0.]
            target.publish(t)

    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            send();executor.spin_once(timeout_sec=.003)

    def wait(predicate,timeout=6.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            spin(.02)
            if predicate():return
        raise AssertionError(str({'state':mission.flight_state,'target':mission.target_reason,
                                  'policy':mission.horizontal_policy_reason,'fault':mission.horizontal_fault_reason,
                                  'native':mission.native_gate.reason,'platform':mission.sensor_reason}))

    try:
        wait(lambda:commands and navigation and mission.platform_pos is not None)
        spin(.15)
        assert not commands[-1].valid and not navigation[-1].valid
        checks.append('before_native_height_no_intent')
        mode['native_ready']=True
        wait(lambda:navigation[-1].valid and navigation[-1].position_valid)
        assert not commands[-1].valid
        assert navigation[-1].mode==NavigationCommand.CRUISE==2
        assert list(navigation[-1].position)==[10.,20.5,0.]
        assert navigation[-1].frame_id=='px4_local_ned:fixture-origin'
        checks.append('aligned_bounded_waypoint_only_after_native_height')
        mode['target']=True
        wait(lambda:envelopes and envelopes[-1].command.valid and not navigation[-1].valid)
        assert mission.flight_state=='track' and mission.target_consecutive>=3
        env=envelopes[-1];msg=env.command
        assert env.origin_id=='fixture-origin' and env.attitude_sample_stamp_ns in attitudes
        assert msg.stamp.sec*10**9+msg.stamp.nanosec>=env.attitude_sample_stamp_ns
        world=attitudes[env.attitude_sample_stamp_ns]@np.array([msg.vx,-msg.vy,-msg.vz])
        assert abs(world[2])<1e-10 and np.linalg.norm(world[:2])<=.5000001
        assert abs(msg.vz)>1e-6
        assert abs(msg.wz)<=.3500001
        checks.append('continuous_target_lock_switches_nav_withdraw_to_real_envelope')
        checks.append('full_tilt_projection_preserves_world_xy_without_vertical_authority')
        if args.decimated_attitude:
            assert env.attitude_sample_stamp_ns not in {
                stamp for stamp,_ in mission.platform_history.fields['attitude']}
            checks.append('atomic_native_attitude_works_when_decimated_bridge_never_received_sample')
        mode['target_valid']=False
        wait(lambda:mission.horizontal_fault_reason and not envelopes[-1].command.valid and not navigation[-1].valid)
        spin(.1)
        assert all(getattr(envelopes[-1].command,x)==0. for x in ('vx','vy','vz','wz'))
        mode['target_valid']=True
        spin(.4)
        assert not envelopes[-1].command.valid and not navigation[-1].valid
        checks.append('active_target_loss_clears_xy_yaw_and_latches_no_waypoint_fallback')
        mode['target_source']='target-restarted'
        spin(.4)
        assert not envelopes[-1].command.valid
        checks.append('target_restart_cannot_clear_active_fault')
        mode['native_ready']=False
        spin(.2)
        assert not envelopes[-1].command.valid and not navigation[-1].valid
        checks.append('native_authority_revocation_remains_stronger_than_mission')
        assert not any(name.startswith('/fmu/in/') and source.count_publishers(name)
                       for name,_ in source.get_topic_names_and_types())
        assert 'fc01_system' not in source.get_node_names()
        checks.append('no_controller_or_px4_command_publishers_started')
        report={'passed':True,'checks':checks,'environment':'Ubuntu22.04 x86_64 ROS2 Humble, synthetic Platform/NativeFlightSnapshot/TargetObservation',
                'hardware':False,'flight_controller_started':False,'envelopes_received':len(envelopes),
                'decimated_native_attitude':args.decimated_attitude,
                'native_state_contract':'/fc01/native/flight_snapshot (atomic state/raw q/origin/receive stamp)'}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print('FC01_MISSION_HORIZONTAL_ROS_PASS '+str(len(checks))+' named checks')
    finally:
        executor.remove_node(mission);executor.remove_node(source)
        mission.destroy_node();source.destroy_node();executor.shutdown();rclpy.try_shutdown()


if __name__=='__main__':main()

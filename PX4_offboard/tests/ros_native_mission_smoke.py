#!/usr/bin/env python3
"""实际Humble接续门：合成原生发布边界状态→实际Mission→原生定高后的水平定点意图；不启动飞控。"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('isolate ROS domain and set ROS_LOCALHOST_ONLY=1')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from jikl_fc01_system.msg import NativeFlightState,NativeFlightSnapshot,PlatformState,VelocityCommandV2,NavigationCommand
    from fc01_mission.mission_node import MissionNode
    rclpy.init(args=['--ros-args','-p','control_enabled:=true',
                    '-p','vision_control_topic:=/step3/velocity','-p','navigation_command_topic:=/step3/navigation',
                    '-p','waypoint_alignment_confirmed:=true','-p','waypoint_alignment_origin_id:=fixture-origin',
                    '-p','navigation_waypoint_x:=23.0','-p','navigation_waypoint_y:=10.0'])
    source=Node('native_mission_fixture')
    mission=MissionNode()
    executor=SingleThreadedExecutor()
    executor.add_node(source);executor.add_node(mission)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    native=source.create_publisher(NativeFlightSnapshot,'/fc01/native/flight_snapshot',qos)
    platform=source.create_publisher(PlatformState,'/fc01/platform/state',qos)
    commands,navigation=[],[]
    source.create_subscription(VelocityCommandV2,'/step3/velocity',commands.append,10)
    source.create_subscription(NavigationCommand,'/step3/navigation',navigation.append,10)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    mode=dict(native=True,old=False,feedback=True,offboard=True,airborne=True,settled=True,
              nav=True,vz=0.,height=1.5,phase='HEIGHT_HOLD',slot=1,reset=0,source='native-fixture-session',
              pid=10017,kill=False,failsafe=False,takeover=False,freeze=False,boot=boot)
    seq=0;last_send=0.;frozen=None;checks=[]

    def send():
        nonlocal seq,last_send,frozen
        now=time.monotonic()
        if now-last_send<.025:return
        last_send=now;seq+=1
        ns=source.get_clock().now().nanoseconds//1000*1000
        p=PlatformState();p.sequence=seq;p.stamp=source.get_clock().now().to_msg()
        p.z_reset_counter=mode['reset'];p.source_id='platform-fixture';p.origin_id='fixture-origin';p.frame_id='px4_local_enu';p.body_frame_id='base_link_flu'
        p.ready=True;p.valid_fields=15;p.reason='OK'
        p.position=[20.,10.,3.];p.velocity=[0.,0.,0.]
        # 倾斜平台测量；无有效目标时速度始终撤销。完整三维编码另由水平封套测试验证。
        p.attitude_xyzw=[0.,float(np.sin(.2)),0.,float(np.cos(.2))];p.angular_velocity=[0.,0.,0.]
        p.local_receive_monotonic_ns=p.attitude_receive_monotonic_ns=p.angular_velocity_receive_monotonic_ns=time.monotonic_ns()
        p.local_timestamp_sample_us=p.attitude_timestamp_sample_us=p.angular_velocity_timestamp_sample_us=ns//1000
        p.position_stamp_ns=p.velocity_stamp_ns=p.attitude_stamp_ns=p.angular_velocity_stamp_ns=ns
        platform.publish(p)
        if not mode['native']:return
        n=NativeFlightState();n.sequence=seq;n.stamp=source.get_clock().now().to_msg()
        n.source_id=mode['source'];n.boot_id=mode['boot'];n.controller_pid=mode['pid'];n.profile='attitude_altitude'
        n.native_monotonic_ns=n.local_receive_monotonic_ns=time.monotonic_ns()
        n.local_sample_stamp_us=ns//1000;n.local_stamp_us=n.status_stamp_us=n.land_stamp_us=ns//1000
        n.attitude_stamp_us=n.attitude_sample_stamp_us=ns//1000
        n.feedback_current=mode['feedback'];n.publication_current=True
        n.armed=True;n.offboard=mode['offboard'];n.landed=not mode['airborne']
        n.rc_navigation_enabled=n.rc_manual_arm_proven=n.ground_reference_valid=n.ground_reference_qualified=True
        n.liftoff_confirmed=mode['airborne'];n.target_height_settled=mode['settled'];n.navigation_allowed=mode['nav']
        n.phase=mode['phase'];n.reason='SYNTHETIC_NATIVE_MEASURED_STATE'
        n.rc_mode_slot=mode['slot'];n.rc_navigation_mode_slot=1;n.z_reset_counter=mode['reset']
        n.reference_z_ned=23.;n.relative_height_m=mode['height'];n.z_ned=23.-mode['height'];n.vz_ned=mode['vz']
        n.target_height_m=1.5;n.height_tolerance_m=.1;n.speed_tolerance_mps=.15;n.settle_seconds=.8
        n.kill=mode['kill'];n.failsafe=mode['failsafe'];n.operator_takeover=mode['takeover']
        if mode['old']:n.stamp.sec-=2
        if mode['freeze']:
            if frozen is None:frozen=n
            n=frozen
        else:frozen=None
        snapshot=NativeFlightSnapshot();snapshot.state=n;snapshot.origin_id=p.origin_id
        snapshot.attitude_receive_monotonic_ns=n.local_receive_monotonic_ns
        c,s=float(np.cos(.2))/math.sqrt(2.),float(np.sin(.2))/math.sqrt(2.)
        snapshot.attitude_wxyz=[c,s,-s,c]
        native.publish(snapshot)

    def spin(duration=.1):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            send();executor.spin_once(timeout_sec=.003)

    def wait(predicate,timeout=7):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            spin(.02)
            if predicate():return
        raise AssertionError('timeout '+repr({'gate':mission.native_gate.reason,'state':mission.flight_state,'commands':[(m.valid,m.vx,m.vy,m.vz) for m in commands[-2:]]}))

    # 本测试不提供已标定目标：到高后只允许原生定点NAV，不得无目标生成跟踪速度。
    # 真正TRACK封套由ros_mission_horizontal_smoke另行检查，不能放宽原生准入门。
    def valid():return navigation and navigation[-1].valid and navigation[-1].mode==NavigationCommand.HOLD
    def invalid():return (commands and not commands[-1].valid and
        all(getattr(commands[-1],k)==0. for k in ('vx','vy','vz','wz')) and
        navigation and not navigation[-1].valid)

    try:
        mode['native']=False
        wait(lambda:commands and mission.platform_pos is not None)
        spin(.15);assert invalid();assert mission.flight_state=='wait_native'
        checks.append('platform_alone_never_starts_takeoff')
        mode['native']=True;mode['old']=True
        spin(.2);assert invalid()
        checks.append('historical_ready_timestamp_rejected')
        mode['old']=False;mode['source']='current-session';mode['pid']=10018
        mode['settled']=False;mode['phase']='ASCEND'
        spin(.15);assert invalid()
        checks.append('climb_and_target_detection_do_not_grant_horizontal_motion')
        mode['settled']=True;mode['phase']='HEIGHT_HOLD';mode['offboard']=False
        spin(.15);assert invalid()
        mode['offboard']=True;mode['slot']=2
        spin(.15);assert invalid()
        checks.append('actual_offboard_and_slot_one_required')
        mode['slot']=1;mode['height']=0.;mode['vz']=0.
        spin(.15);assert invalid()
        mode['height']=1.5;mode['vz']=.6
        spin(.15);assert invalid()
        checks.append('fake_height_and_unsettled_vertical_speed_rejected')
        mode['vz']=0.
        wait(valid)
        assert mission.flight_state=='native_height_hold'
        assert not commands[-1].valid and all(getattr(commands[-1],k)==0. for k in ('vx','vy','vz','wz'))
        assert not navigation[-1].position_valid and navigation[-1].mode==NavigationCommand.HOLD
        checks.append('current_native_hold_allows_point_hold_without_unlocked_target_velocity')
        # 非正常DDS内容矛盾：沿用同seq/stamp却声称已退出Offboard，必须立即撤销，
        # 不能因被当成重复包而继续使用旧授权。后续真正新快照仍可恢复。
        mode['native']=False
        spin(.05)  # 排空之前真正递增的快照，再复制已接受的最后一条，隔离本反例。
        conflicting=copy.deepcopy(mission.native_gate.latest)
        conflicting.offboard=False
        snapshot=NativeFlightSnapshot();snapshot.state=conflicting;snapshot.origin_id='fixture-origin'
        snapshot.attitude_receive_monotonic_ns=conflicting.local_receive_monotonic_ns
        c,s=float(np.cos(.2))/math.sqrt(2.),float(np.sin(.2))/math.sqrt(2.)
        snapshot.attitude_wxyz=[c,s,-s,c]
        native.publish(snapshot)
        wait(lambda:invalid() and mission.native_gate.reason=='NATIVE_DUPLICATE_INCONSISTENT')
        mode['native']=True
        wait(valid)
        checks.append('same_identity_conflicting_safety_payload_revokes_without_renewal')
        mode['freeze']=True
        spin(.45);assert invalid()
        mode['freeze']=False
        wait(valid)
        checks.append('duplicate_snapshot_cannot_renew_native_or_local_ttl')
        mode['native']=False
        spin(.45);assert invalid()
        mode['native']=True
        wait(valid)
        checks.append('native_dds_loss_revokes_and_new_current_feedback_recovers')
        for key in ('kill','failsafe','takeover'):
            mode[key]=True;wait(invalid)
            mode[key]=False;wait(valid)
        checks.append('kill_failsafe_and_rc_takeover_always_revoke')
        mode['boot']='prior-boot'
        wait(invalid)
        mode['boot']=boot;mode['source']='restarted-session';mode['pid']=10019
        wait(valid)
        mode['reset']=1
        spin(.04)
        # 首个新reset快照撤销；下一新快照方可承认原生重新测量授权。
        assert mission.native_gate.identity[-3]==1
        wait(valid)
        checks.append('boot_process_and_coordinate_reset_provenance')
        assert not any(t.startswith('/fmu/in/') and source.count_publishers(t) for t,_ in source.get_topic_names_and_types())
        assert 'fc01_system' not in source.get_node_names()
        checks.append('no_flight_controller_or_fmu_input_publishers')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'passed':True,'checks':checks,'environment':'Ubuntu22.04 x86_64 Humble synthetic atomic-native/platform DDS','hardware':False,'flight_controller_started':False,'control_enabled_isolated_fixture':True,'native_state_contract':'/fc01/native/flight_snapshot (atomic state/raw q/origin/receive stamp)'},ensure_ascii=False,indent=2)+'\n')
        print('FC01_NATIVE_MISSION_GATE_PASS '+str(len(checks))+' named checks')
    finally:
        executor.remove_node(mission);executor.remove_node(source)
        mission.destroy_node();source.destroy_node();executor.shutdown();rclpy.try_shutdown()


if __name__=='__main__':main()

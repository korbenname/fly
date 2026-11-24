"""执行实际Mission方法：水平坐标、原点、锁定切换和发布封套，不启动动力。"""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'target/src/fc01_mission'))
from fc01_mission.sensor_contract import rotation_xyzw
from fc01_mission.sensor_contract import stamp_ns, finite
from fc01_mission.platform_feedback import quaternion_ned_frd_to_enu_flu

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'target/src/fc01_mission/fc01_mission/mission_node.py'


class Navigation(NS):
    HOLD, CRUISE = 1, 2


def fresh(stamp, now, timeout):
    return stamp > 0 and -20_000_000 <= now-stamp <= timeout*1e9


def implementation():
    tree = ast.parse(SOURCE.read_text())
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == 'MissionNode')
    selected = ('_clear_target','_blend_angle','_tracking_policy_current','_waypoint_policy_current',
                '_update_native_mission','_publish_command','_publish_navigation','_on_native_flight',
                '_native_attitude_current','_native_snapshot_binding','_on_native_snapshot')
    node = ast.ClassDef(name='Actual', bases=[], keywords=[], decorator_list=[],
                       body=[x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name in selected])
    module = ast.Module(body=[node], type_ignores=[])
    ast.fix_missing_locations(module)
    scope = {'np':np,'math':math,'time':time,'fresh':fresh,'finite':finite,'stamp_ns':stamp_ns,
             'quaternion_ned_frd_to_enu_flu':quaternion_ned_frd_to_enu_flu,'VelocityCommandMsg':NS,
             'HorizontalVelocityCommand':NS,'NavigationCommandMsg':Navigation,'rotation_xyzw':rotation_xyzw}
    exec(compile(module,str(SOURCE),'exec'),scope)
    return scope['Actual']


def fixture():
    n = implementation()()
    n.platform_input_mode = 'px4'
    n.control_enabled = True
    n.guidance_invalidated = False
    n._native_navigation_current = lambda now: True
    native = NS(source_id='native',boot_id='boot',controller_pid=17,task_epoch=1,
                z_reset_counter=0,vz_reset_counter=0,quat_reset_counter=0,sequence=1,
                stamp=NS(sec=100,nanosec=0),attitude_sample_stamp_us=99_990_000)
    n.native_gate = NS(reason='NATIVE_NOT_CURRENT',latest=native,timeout=.3)
    n.platform_pos = np.array([10.,20.,100.])
    n.platform_start = n.platform_pos.copy()
    n.platform_vel = np.zeros(3)
    n.platform_rot = np.eye(3)
    n.platform_attitude_stamp_ns = 99_990_000_000
    n.platform_history = NS(identity=('px4-writer',0,'px4_local:12:0:0:0:0:0'),fields={'attitude':[(99_990_000_000,[0.,0.,0.,1.])]})
    # ENU/FLU单位姿态对应原始PX4 NED/FRD航向+90度。
    n.native_attitude_snapshot=(n._native_snapshot_binding(native),
        (math.sqrt(.5),0.,0.,math.sqrt(.5)),n.platform_history.identity[2],9_990_000_000)
    n.get_clock = lambda: NS(now=lambda:NS(nanoseconds=100_000_000_000,to_msg=lambda:NS(sec=100,nanosec=0)))
    n.horizontal_fault_reason = ''
    n.horizontal_policy_reason = ''
    n.native_tracking_started = False
    n.horizontal_navigation_mode = 'hold'
    n.position_takeoff_anchor_authority = False
    n.horizontal_max_radius_m = 2.
    n.horizontal_max_speed_mps = .5
    n.horizontal_max_yaw_rate_rps = .35
    n.standoff_calibration_confirmed = n.camera_extrinsics_confirmed = True
    n.standoff_calibration_id = 'fixture-standoff-1m'
    n.camera_extrinsics_id = 'fixture-mount'
    n.standoff_m = 1.
    n.target_locked = True
    n.vision_rel = np.array([2.,0.,1.])
    n.est_pos = np.array([12.,20.,1000.])
    n.target_vel_est = np.array([0.,0.,50.])
    n.vision_time = n.vision_state_time = 10.
    n.sensor_timeout_s = .3
    n.last_target_sample = 100_000_000_000
    n.command_timeout_s = .5
    n.controller_gain = 2.
    n.track_max_speed = 2.
    n.yaw_cmd = 0.
    n.flight_state = 'native_height_hold'
    n.waypoint_alignment_confirmed = True
    n.waypoint_alignment_origin_id = n.platform_history.identity[2]
    n.navigation_waypoint_x,n.navigation_waypoint_y = 11.,20.
    n.navigation_waypoint_z = 999.
    n.terminal_start = None
    n.terminal_cmd = np.zeros(3)
    n.last_v_cmd = np.zeros(3)
    n.vision_control_hz = 50.
    n.last_vision_control_time = 0.
    n.yaw_rate_cmd = n.yaw_rate_ff = 0.
    n.yaw_rate_gain = 3.
    n.last_published_yaw = None
    n.last_published_yaw_time = 0.
    n.vision_control_seq = n.navigation_seq = 1
    n.reset_counter = 0
    n.vision_control_source_id = 'mission-fixture'
    n.last_navigation_time = 0.
    n.navigation_command_hz = 10.
    n.commands,n.envelopes,n.navigation = [],[],[]
    n.vision_control_pub = NS(publish=n.commands.append)
    n.horizontal_command_pub = NS(publish=n.envelopes.append)
    n.navigation_command_pub = NS(publish=n.navigation.append)
    return n


def bind_native_rotation(node, xyzw, *, stamp_us=99_990_000, received_ns=9_990_000_000):
    """由独立Hamilton基变换构造原始PX4样本，绑定到同一原生状态。"""
    x,y,z,w=xyzw
    a=math.sqrt(.5)
    raw=(a*(w+z),a*(x+y),a*(x-y),a*(w-z))
    node.native_gate.latest.attitude_sample_stamp_us=stamp_us
    node.native_attitude_snapshot=(node._native_snapshot_binding(node.native_gate.latest),
        raw,node.platform_history.identity[2],received_ns)


class HorizontalMissionTests(unittest.TestCase):
    def test_position_mission_defers_distance_to_the_native_ground_anchor(self):
        n = fixture()
        n.navigation_waypoint_x = 13.
        self.assertFalse(n._waypoint_policy_current())
        n.position_takeoff_anchor_authority = True
        self.assertTrue(n._waypoint_policy_current())
        n.waypoint_alignment_origin_id = 'older-origin'
        self.assertFalse(n._waypoint_policy_current())
    def test_before_native_height_gate_no_horizontal_or_yaw(self):
        n=fixture();n._native_navigation_current=lambda now:False
        velocity,valid=n._update_native_mission(10.,.01)
        self.assertFalse(valid);np.testing.assert_array_equal(velocity,[0.,0.,0.])
        n._publish_command([1.,1.,99.],10.,True);n._publish_navigation(10.)
        self.assertFalse(n.commands[-1].valid);self.assertFalse(n.navigation[-1].valid)
        self.assertEqual([n.commands[-1].vx,n.commands[-1].vy,n.commands[-1].vz,n.commands[-1].wz],[0.]*4)

    def test_hold_waypoint_and_tracking_are_single_intent(self):
        n=fixture();n.target_locked=False
        n._publish_navigation(10.);self.assertTrue(n.navigation[-1].valid)
        self.assertEqual(n.navigation[-1].mode,Navigation.HOLD)
        n.horizontal_navigation_mode='waypoint'
        n._publish_navigation(10.11)
        message=n.navigation[-1]
        self.assertTrue(message.valid);self.assertTrue(message.position_valid)
        self.assertEqual(message.mode,Navigation.CRUISE)
        self.assertEqual(message.frame_id,'px4_local_ned:'+n.platform_history.identity[2])
        self.assertEqual(message.position,[20.,11.,0.])
        n.target_locked=True
        velocity,valid=n._update_native_mission(10.12,.01)
        self.assertTrue(valid);n._publish_command(velocity,10.12,valid)
        n._publish_navigation(10.22)
        self.assertTrue(n.commands[-1].valid)
        self.assertFalse(n.navigation[-1].valid)
        self.assertFalse(n.navigation[-1].position_valid)
        self.assertTrue(n.native_tracking_started)

    def test_unconfirmed_or_outside_waypoint_falls_back_current_hold(self):
        for change in ({'waypoint_alignment_confirmed':False},{'waypoint_alignment_origin_id':'other-origin'},
                       {'navigation_waypoint_x':13.},{'navigation_waypoint_x':math.nan}):
            n=fixture();n.target_locked=False;n.horizontal_navigation_mode='waypoint'
            for k,v in change.items():setattr(n,k,v)
            n._publish_navigation(10.)
            self.assertTrue(n.navigation[-1].valid)
            self.assertFalse(n.navigation[-1].position_valid)
            self.assertEqual(n.navigation[-1].mode,Navigation.HOLD)

    def test_missing_calibration_or_old_image_blocks_direct_publish(self):
        for change in ({'standoff_calibration_confirmed':False},{'standoff_calibration_id':''},
                       {'camera_extrinsics_confirmed':False},{'camera_extrinsics_id':''},
                       {'standoff_m':0.},{'standoff_m':3.},{'standoff_m':math.nan},
                       {'last_target_sample':99_000_000_000},{'vision_state_time':9.},
                       {'vision_rel':np.array([math.nan,0.,0.])}):
            n=fixture()
            for k,v in change.items():setattr(n,k,v)
            self.assertFalse(n._tracking_policy_current(10.))
            n._publish_command([.4,0.,0.],10.,True)
            self.assertFalse(n.commands[-1].valid)
            self.assertFalse(n.native_tracking_started)

    def test_tilt_projection_retains_body_z_and_exact_attitude_stamp(self):
        # 完整ENU/FLU旋转，native收到FRD后完整旋转才能恢复NED XY。
        roll,pitch,yaw=.31,-.4,math.pi/2
        rx=np.array([[1.,0.,0.],[0.,math.cos(roll),-math.sin(roll)],[0.,math.sin(roll),math.cos(roll)]])
        ry=np.array([[math.cos(pitch),0.,math.sin(pitch)],[0.,1.,0.],[-math.sin(pitch),0.,math.cos(pitch)]])
        rz=np.array([[math.cos(yaw),-math.sin(yaw),0.],[math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
        n=fixture();n.platform_rot=rz@ry@rx;n.yaw_cmd=None
        cr,sr=math.cos(roll/2),math.sin(roll/2)
        cp,sp=math.cos(pitch/2),math.sin(pitch/2)
        cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        q=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
        n.platform_history.fields['attitude']=[(99_990_000_000,q)]
        bind_native_rotation(n,q)
        n._publish_command([.3,-.4,100.],10.,True)
        msg=n.commands[-1];env=n.envelopes[-1]
        body_flu=np.array([msg.vx,-msg.vy,-msg.vz])
        np.testing.assert_allclose(n.platform_rot@body_flu,[.3,-.4,0.],atol=1e-12)
        self.assertNotEqual(msg.vz,0.)
        self.assertIs(env.command,msg)
        self.assertEqual(env.attitude_sample_stamp_ns,99_990_000_000)
        self.assertEqual(msg.stamp.sec,100)
        self.assertEqual(env.origin_id,n.platform_history.identity[2])

    def test_bridge_latest_ahead_of_native_encodes_only_common_raw_attitude(self):
        n=fixture()
        # 桥先收到90度航向样本，而原生状态仍确认上一帧单位姿态。
        n.platform_rot=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        n.platform_attitude_stamp_ns=100_000_000_000
        n.platform_history.fields['attitude'].append((100_000_000_000,[0.,0.,math.sqrt(.5),math.sqrt(.5)]))
        n.yaw_cmd=None
        n._publish_command([.3,.1,0.],10.,True)
        self.assertTrue(n.envelopes[-1].command.valid)
        self.assertEqual(n.envelopes[-1].attitude_sample_stamp_ns,99_990_000_000)
        np.testing.assert_allclose([n.commands[-1].vx,n.commands[-1].vy,n.commands[-1].vz],[.3,-.1,0.])

    def test_native_confirmed_attitude_missing_or_stale_retracts(self):
        for missing in (True,False):
            n=fixture()
            if missing:n.native_attitude_snapshot=None
            else:bind_native_rotation(n,[0.,0.,0.,1.],stamp_us=99_000_000)
            n._publish_command([.3,0.,0.],10.,True)
            self.assertFalse(n.commands[-1].valid)
            np.testing.assert_array_equal([n.commands[-1].vx,n.commands[-1].vy,n.commands[-1].vz],[0.,0.,0.])

    def test_decimated_platform_does_not_need_native_raw_sample_in_history(self):
        n=fixture();n.yaw_cmd=None
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_050_000_000,
            to_msg=lambda:NS(sec=100,nanosec=50_000_000)))
        # 200Hz原始姿态的.035s帧被50Hz桥快照跳过，原子快照仍带有原生精确q。
        n.platform_history.fields['attitude']=[
            (100_000_000_000,[0.,0.,0.,1.]),
            (100_020_000_000,[0.,0.,0.,1.]),
            (100_040_000_000,[0.,0.,0.,1.])]
        q=[0.,0.,math.sin(.1),math.cos(.1)]
        bind_native_rotation(n,q,stamp_us=100_035_000,received_ns=10_035_000_000)
        n._publish_command([.3,.1,0.],10.05,True)
        m=n.commands[-1];env=n.envelopes[-1]
        self.assertTrue(m.valid)
        self.assertEqual(env.attitude_sample_stamp_ns,100_035_000_000)
        self.assertNotIn(env.attitude_sample_stamp_ns,[x[0] for x in n.platform_history.fields['attitude']])
        np.testing.assert_allclose(rotation_xyzw(q)@np.array([m.vx,-m.vy,-m.vz]),[.3,.1,0.],atol=1e-12)

    def test_atomic_binding_origin_receipt_and_invalid_q_retract_finite_zero(self):
        mutations=('binding','origin','receipt','nan','zero','nonnull_norm')
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                n=fixture();binding,q,origin,received=n.native_attitude_snapshot
                if mutation=='binding':binding=(*binding[:-1],binding[-1]-1)
                elif mutation=='origin':origin='other-origin'
                elif mutation=='receipt':received=9_000_000_000
                elif mutation=='nan':q=(math.nan,0.,0.,1.)
                elif mutation=='zero':q=(0.,0.,0.,0.)
                else:q=(2.,0.,0.,0.)
                n.native_attitude_snapshot=(binding,q,origin,received)
                n._publish_command([.3,.1,0.],10.,True)
                self.assertFalse(n.commands[-1].valid)
                self.assertEqual([getattr(n.commands[-1],x) for x in ('vx','vy','vz','wz')],[0.]*4)

    def test_world_norm_clamp_does_not_distort_rotation(self):
        n=fixture();n.platform_rot=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        n.yaw_cmd=None
        n.platform_history.fields['attitude']=[(99_990_000_000,[.5,.5,.5,.5])]
        bind_native_rotation(n,[.5,.5,.5,.5])
        n._publish_command([3.,4.,999.],10.,True)
        m=n.commands[-1]
        np.testing.assert_allclose(n.platform_rot@np.array([m.vx,-m.vy,-m.vz]),[.3,.4,0.])

    def test_yaw_rate_sign_bound_wrap_and_zero(self):
        for yaw,expected in ((.1,-.3),(-.1,.3),(0.,0.),(2.,-.35)):
            n=fixture();n.yaw_cmd=yaw;n._publish_command([0.,0.,0.],10.,True)
            self.assertAlmostEqual(n.commands[-1].wz,expected)
        n=fixture();a=math.pi-.02
        n.platform_rot=np.array([[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]])
        n.yaw_cmd=-math.pi+.02;n._publish_command([0.,0.,0.],10.,True)
        self.assertAlmostEqual(n.commands[-1].wz,-.12)
        for field in ('yaw_cmd','yaw_rate_gain','yaw_rate_ff','last_published_yaw'):
            for invalid in (math.nan,math.inf,-math.inf):
                n=fixture();setattr(n,field,invalid)
                n._publish_command([.3,0.,0.],10.,True)
                self.assertFalse(n.commands[-1].valid)
                self.assertEqual(n.commands[-1].wz,0.)

    def test_standoff_too_close_retreat_and_world_vertical_ignored(self):
        n=fixture();n.est_pos=np.array([10.2,20.,-999.])
        velocity,valid=n._update_native_mission(10.,.01)
        self.assertTrue(valid);self.assertLess(velocity[0],0.)
        self.assertLessEqual(np.linalg.norm(velocity[:2]),.5)
        self.assertEqual(velocity[2],0.)

    def test_tracking_loss_latches_and_never_restarts_waypoint(self):
        n=fixture();n._publish_command([.3,0.,0.],10.,True)
        n.horizontal_navigation_mode='waypoint';n.target_locked=False
        velocity,valid=n._update_native_mission(10.03,.01)
        self.assertFalse(valid);self.assertTrue(n.horizontal_fault_reason)
        n._publish_command(velocity,10.03,valid);n._publish_navigation(10.11)
        self.assertFalse(n.commands[-1].valid);self.assertFalse(n.navigation[-1].valid)
        n.target_locked=True;n.vision_rel=np.ones(3);n.est_pos=np.array([12.,20.,100.]);n.vision_time=10.12
        self.assertFalse(n._update_native_mission(10.12,.01)[1])

    def test_native_revoke_after_tracking_clears_pending_velocity(self):
        n=fixture();n._publish_command([.3,0.,0.],10.,True)
        n._native_navigation_current=lambda now:False
        velocity,valid=n._update_native_mission(10.03,.01)
        self.assertFalse(valid);self.assertEqual(n.horizontal_fault_reason,'NATIVE_NOT_CURRENT')
        np.testing.assert_array_equal(velocity,[0.,0.,0.])

    def test_invalid_with_missing_or_nan_rotation_publishes_finite_zero(self):
        for rotation in (None,np.full((3,3),math.nan)):
            n=fixture();n.platform_rot=rotation
            n._publish_command([math.nan,0.,0.],10.,False)
            m=n.commands[-1]
            self.assertFalse(m.valid)
            self.assertEqual([m.vx,m.vy,m.vz,m.wz],[0.]*4)

    def test_only_new_native_task_epoch_clears_tracking_failure(self):
        class Gate:
            generation=0
            reason='NATIVE_SESSION_CHANGED'
            def accept(self,*args):
                self.generation+=1
                return False
        for task,expected in ((('boot',17,'source',2),True),
                              (('boot',18,'new-source',0),False),
                              (('boot',17,'source',1),False)):
            n=fixture();n.native_tracking_started=True;n.horizontal_fault_reason='TARGET_LOST'
            n.native_gate=Gate();n.native_task_identity=('boot',17,'source',1)
            n.native_flight_topic='/native';n.count_publishers=lambda topic:1
            n.native_navigation_was_current=False
            n._on_native_flight(NS(boot_id=task[0],controller_pid=task[1],source_id=task[2],task_epoch=task[3]))
            self.assertEqual(not n.native_tracking_started,expected)
            self.assertEqual(n.horizontal_fault_reason=='',expected)

    def test_actual_production_launch_and_config_choose_track(self):
        import yaml
        config=yaml.safe_load((ROOT/'target/src/fc01_mission/config/mission.yaml').read_text())
        params=config['fc01_mission']['ros__parameters']
        self.assertEqual(params['engage_mode'],'track');self.assertFalse(params['control_enabled'])
        self.assertFalse(params['standoff_calibration_confirmed'])
        for path in ('target/src/fc01_mission/launch/mission.launch.py','target/src/fc01_bringup/launch/robot.launch.py'):
            tree=ast.parse((ROOT/path).read_text())
            self.assertNotIn('push',[x.value for x in ast.walk(tree) if isinstance(x,ast.Constant)])
        first=(ROOT/'target/src/fc01_bringup/launch/horizontal_first_test.launch.py').read_text()
        self.assertIn("'engage_mode':'track'",first)
        self.assertIn("DeclareLaunchArgument('site_mission_config', description=",first)


if __name__=='__main__':unittest.main()

"""实机mission不自行起飞/定高：原生授权证据、时间和最终发布边界。"""
import ast
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'target/src/fc01_mission'))
from fc01_mission.native_flight_gate import NativeFlightGate
from fc01_mission.sensor_contract import fresh, finite, rotation_xyzw, stamp_ns
from fc01_mission.platform_feedback import quaternion_ned_frd_to_enu_flu


def state(ns=100_000_000_000,mono=10.,seq=1,**changes):
    data=dict(schema_version=1,sequence=seq,stamp=NS(sec=ns//10**9,nanosec=ns%10**9),
      source_id='boot-pid-start',boot_id='boot',controller_pid=17,task_epoch=0,profile='attitude_altitude',
      native_monotonic_ns=int(mono*1e9),local_receive_monotonic_ns=int(mono*1e9),
      local_sample_stamp_us=ns//1000,attitude_stamp_us=ns//1000,
      attitude_sample_stamp_us=ns//1000,phase='HEIGHT_HOLD',reason='TARGET_HEIGHT_SETTLED',
      feedback_current=True,publication_current=True,ready_wait_arm=False,armed=True,offboard=True,
      landed=False,kill=False,operator_takeover=False,failsafe=False,rc_navigation_enabled=True,
      rc_manual_arm_proven=True,ground_reference_valid=True,ground_reference_qualified=True,
      liftoff_confirmed=True,target_height_settled=True,navigation_allowed=True,
      rc_mode_slot=1,rc_navigation_mode_slot=1,z_reset_counter=0,vz_reset_counter=0,quat_reset_counter=0,reference_z_ned=23.,z_ned=21.5,vz_ned=0.,
      relative_height_m=1.5,target_height_m=1.5,height_tolerance_m=.1,speed_tolerance_mps=.15,settle_seconds=.8)
    data.update(changes)
    return NS(**data)


def methods(*names):
    path=ROOT/'target/src/fc01_mission/fc01_mission/mission_node.py'
    cls=next(x for x in ast.parse(path.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
    module=ast.Module(body=[ast.ClassDef(name='Actual',bases=[],keywords=[],body=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in names],decorator_list=[])],type_ignores=[])
    ast.fix_missing_locations(module)
    ns={'np':np,'math':math,'fresh':fresh,'finite':finite,'rotation_xyzw':rotation_xyzw,
        'stamp_ns':stamp_ns,'time':time,'quaternion_ned_frd_to_enu_flu':quaternion_ned_frd_to_enu_flu}
    exec(compile(module,str(path),'exec'),ns)
    return ns['Actual']


class NativeGateTests(unittest.TestCase):
    def snapshot_receiver(self):
        n=methods('_on_native_snapshot','_on_native_flight','_native_snapshot_binding',
                  '_native_attitude_current','_native_navigation_current')()
        n.platform_input_mode='px4';n.native_flight_topic='/fc01/native/flight_snapshot'
        n.count_publishers=lambda _:1
        n.native_gate=NativeFlightGate('boot')
        n.sensor_timeout_s=.3;n.native_attitude_snapshot=None
        n.native_navigation_was_current=False;n.native_task_identity=None
        n.native_tracking_started=False;n.horizontal_fault_reason=''
        n.platform_history=NS(identity=('platform',0,'origin'))
        n.platform_reference_counters=(0,0,0)
        n.now_ns=100_000_000_000
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=n.now_ns))
        n.target_revocations=[];n._clear_target=n.target_revocations.append
        n.reset_counter=0;n.flight_state='wait_native';n.takeoff_complete_time=None
        return n

    def snapshot(self, **changes):
        fields=dict(schema_version=1,state=state(),
            attitude_wxyz=[math.sqrt(.5),0.,0.,math.sqrt(.5)],
            origin_id='origin',attitude_receive_monotonic_ns=10_000_000_000)
        fields.update(changes)
        return NS(**fields)

    def test_atomic_snapshot_accepts_native_raw_missing_from_bridge_and_copies_raw_q(self):
        n=self.snapshot_receiver();snapshot=self.snapshot()
        # 桥完全没有该raw样本的历史；授权只绑定同一原子消息，不能要求降采样历史重合。
        n.platform_history.fields={'attitude':[]}
        with patch('time.monotonic',return_value=10.):
            n._on_native_snapshot(snapshot)
        self.assertTrue(n._native_navigation_current(10.))
        np.testing.assert_allclose(n._native_attitude_current(10.),np.eye(3),atol=1e-12)
        snapshot.attitude_wxyz[0]=0.
        self.assertAlmostEqual(n.native_attitude_snapshot[1][0],math.sqrt(.5))

    def test_atomic_duplicate_payload_changes_and_old_raw_cannot_extend_receipt(self):
        for change in ('q','origin','receipt','stamp'):
            with self.subTest(change=change):
                n=self.snapshot_receiver()
                with patch('time.monotonic',return_value=10.):
                    n._on_native_snapshot(self.snapshot())
                msg=self.snapshot()
                if change=='q':msg.attitude_wxyz=[1.,0.,0.,0.]
                elif change=='origin':msg.origin_id='other-origin'
                elif change=='receipt':msg.attitude_receive_monotonic_ns-=1
                else:msg.state.attitude_sample_stamp_us-=1
                with patch('time.monotonic',return_value=10.01):
                    n._on_native_snapshot(msg)
                self.assertIsNone(n.native_gate.latest)
                self.assertIsNone(n.native_attitude_snapshot)
                self.assertEqual(n.native_gate.reason,'NATIVE_ATOMIC_DUPLICATE_INCONSISTENT')
        n=self.snapshot_receiver()
        with patch('time.monotonic',return_value=10.):n._on_native_snapshot(self.snapshot())
        atom=n.native_attitude_snapshot;received=n.native_gate.received
        with patch('time.monotonic',return_value=10.1):n._on_native_snapshot(self.snapshot())
        self.assertIs(n.native_attitude_snapshot,atom)
        self.assertEqual(n.native_gate.received,received)
        msg=self.snapshot(state=state(100_100_000_000,10.1,2,attitude_sample_stamp_us=100_000_000),
                          attitude_receive_monotonic_ns=10_100_000_000)
        with patch('time.monotonic',return_value=10.1):n._on_native_snapshot(msg)
        self.assertEqual(n.native_gate.reason,'NATIVE_ATOMIC_DUPLICATE_INCONSISTENT')
        self.assertIsNone(n.native_attitude_snapshot)

    def test_atomic_bad_schema_q_origin_source_or_receive_never_grants_motion(self):
        changes=({'schema_version':2},{'attitude_wxyz':[math.nan,0.,0.,1.]},
                 {'attitude_wxyz':[0.]*4},{'attitude_wxyz':[2.,0.,0.,0.]},
                 {'origin_id':''},{'attitude_receive_monotonic_ns':0},
                 {'attitude_receive_monotonic_ns':10_010_000_000},
                 {'attitude_receive_monotonic_ns':9_000_000_000},
                 {'state':state(attitude_sample_stamp_us=99_000_000)})
        for change in changes:
            with self.subTest(change=change):
                n=self.snapshot_receiver()
                with patch('time.monotonic',return_value=10.):
                    n._on_native_snapshot(self.snapshot(**change))
                self.assertIsNone(n.native_gate.latest)
                self.assertIsNone(n.native_attitude_snapshot)

    def test_atomic_source_expiry_and_platform_origin_change_revoke_current(self):
        for change in ('source_age','receive_age','origin','multiple_publishers'):
            n=self.snapshot_receiver()
            with patch('time.monotonic',return_value=10.):n._on_native_snapshot(self.snapshot())
            if change=='source_age':n.now_ns+=310_000_000
            elif change=='receive_age':n.native_attitude_snapshot=(*n.native_attitude_snapshot[:3],9_000_000_000)
            elif change=='origin':n.platform_history.identity=('platform',0,'relocated-origin')
            else:n.count_publishers=lambda _:2
            self.assertFalse(n._native_navigation_current(10.01))

    def test_current_native_measured_hold_is_required(self):
        for changes in ({'liftoff_confirmed':False},{'target_height_settled':False},
                        {'navigation_allowed':False},{'publication_current':False},
                        {'feedback_current':False},{'armed':False},{'offboard':False},
                        {'landed':True},{'kill':True},{'operator_takeover':True},{'failsafe':True},
                        {'rc_mode_slot':2},{'target_height_m':2.78},{'vz_ned':.8},
                        {'relative_height_m':1.5,'z_ned':23.},{'phase':'RC_WAIT_ARM'},
                        {'boot_id':'prior-boot'},{'controller_pid':0},{'relative_height_m':math.nan}):
            with self.subTest(changes=changes):
                gate=NativeFlightGate('boot')
                self.assertFalse(gate.accept(state(**changes),100_000_000_000,10.))
        self.assertTrue(NativeFlightGate('boot').accept(state(),100_000_000_000,10.))

    def test_old_ready_future_duplicate_and_local_progress_cannot_renew(self):
        g=NativeFlightGate('boot')
        self.assertFalse(g.accept(state(ns=1_000_000_000),100_000_000_000,10.))
        g=NativeFlightGate('boot')
        self.assertFalse(g.accept(state(ns=101_000_000_000),100_000_000_000,10.))
        g=NativeFlightGate('boot');self.assertTrue(g.accept(state(),100_000_000_000,10.))
        self.assertTrue(g.accept(state(),100_100_000_000,10.1))
        self.assertEqual(g.received,10.)
        self.assertFalse(g.accept(state(navigation_allowed=False),100_110_000_000,10.11))
        self.assertIsNone(g.latest)
        g=NativeFlightGate('boot');g.accept(state(),100_000_000_000,10.)
        self.assertFalse(g.current(100_310_000_000,10.31))
        g=NativeFlightGate('boot');g.accept(state(),100_000_000_000,10.)
        for i in range(1,5):
            ns=100_000_000_000+i*100_000_000
            got=g.accept(state(ns,10.+i*.1,i+1,local_sample_stamp_us=100_000_000),ns,10.+i*.1)
        self.assertFalse(got)

    def test_current_rechecks_original_sample_and_monotonic_age(self):
        for changes in ({'local_sample_stamp_us':99_710_000},
                        {'local_receive_monotonic_ns':9_710_000_000},
                        {'native_monotonic_ns':9_710_000_000}):
            with self.subTest(changes=changes):
                g=NativeFlightGate('boot')
                self.assertTrue(g.accept(state(**changes),100_000_000_000,10.))
                self.assertFalse(g.current(100_200_000_000,10.2))
                self.assertEqual(g.reason,'NATIVE_UNDERLYING_MEASUREMENT_STALE')

    def test_duplicate_with_conflicting_safety_content_revokes(self):
        for changes in ({'armed':False},{'offboard':False},{'landed':True},
                        {'rc_mode_slot':2},{'relative_height_m':0.,'z_ned':23.},
                        {'vz_ned':1.},{'liftoff_confirmed':False},
                        {'local_receive_monotonic_ns':10_010_000_000}):
            with self.subTest(changes=changes):
                g=NativeFlightGate('boot');self.assertTrue(g.accept(state(),100_000_000_000,10.))
                self.assertFalse(g.accept(state(**changes),100_020_000_000,10.02))
                self.assertEqual(g.reason,'NATIVE_DUPLICATE_INCONSISTENT')
                self.assertIsNone(g.latest)
                self.assertEqual(g.received,10.)
        for ns,seq in ((100_010_000_000,1),(100_000_000_000,2)):
            g=NativeFlightGate('boot');g.accept(state(),100_000_000_000,10.)
            self.assertFalse(g.accept(state(ns,10.,seq),100_020_000_000,10.02))
            self.assertEqual(g.reason,'NATIVE_DUPLICATE_INCONSISTENT')

    def test_restart_task_and_source_regression_revoke(self):
        for change in ({'source_id':'new-process','controller_pid':18},{'task_epoch':1},
                       {'z_reset_counter':1},{'vz_reset_counter':1},{'quat_reset_counter':1}):
            g=NativeFlightGate('boot');g.accept(state(),100_000_000_000,10.)
            self.assertFalse(g.accept(state(100_020_000_000,10.02,2,**change),100_020_000_000,10.02))
            self.assertTrue(g.accept(state(100_040_000_000,10.04,3,**change),100_040_000_000,10.04))
            self.assertFalse(g.accept(state(100_060_000_000,10.06,4),100_060_000_000,10.06))
        g=NativeFlightGate('boot');g.accept(state(seq=3),100_000_000_000,10.)
        self.assertFalse(g.accept(state(100_010_000_000,10.01,2),100_010_000_000,10.01))
        self.assertFalse(g.accept(state(100_020_000_000,10.02,4),100_020_000_000,10.02))

    def test_stale_unknown_session_revokes_without_retiring_live_session(self):
        g=NativeFlightGate('boot')
        self.assertTrue(g.accept(state(),100_000_000_000,10.))
        identity=g.identity
        self.assertFalse(g.accept(state(98_000_000_000,8.,2,
            source_id='unseen-old-process',controller_pid=3),100_010_000_000,10.01))
        self.assertIsNone(g.latest)
        self.assertEqual(g.identity,identity)
        self.assertNotIn(identity,g.retired)
        self.assertTrue(g.accept(state(100_020_000_000,10.02,3),100_020_000_000,10.02))

    def test_independent_monotonic_source_and_graph_validity(self):
        for changes in ({'native_monotonic_ns':9_000_000_000},
                        {'local_receive_monotonic_ns':9_000_000_000}):
            self.assertFalse(NativeFlightGate('boot').accept(state(**changes),100_000_000_000,10.))
        g=NativeFlightGate('boot');self.assertFalse(g.accept(state(),100_000_000_000,10.,2))
        g=NativeFlightGate('boot');g.accept(state(),100_000_000_000,10.)
        self.assertFalse(g.current(99_000_000_000,10.01))
        self.assertFalse(g.current(101_000_000_000,11.))

    def test_native_and_platform_reset_counters_must_match(self):
        n=methods('_native_navigation_current','_native_attitude_current','_native_snapshot_binding')()
        n.platform_input_mode='px4';n.native_flight_topic='/native'
        n.count_publishers=lambda topic:1
        n.native_gate=NativeFlightGate('boot');n.native_gate.accept(state(),100_000_000_000,10.)
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
        n.platform_reference_counters=(0,0,0)
        n.sensor_timeout_s=.3
        n.platform_history=NS(identity=('platform',0,'origin'))
        n.native_attitude_snapshot=(n._native_snapshot_binding(n.native_gate.latest),
            (math.sqrt(.5),0.,0.,math.sqrt(.5)),'origin',10_000_000_000)
        self.assertTrue(n._native_navigation_current(10.))
        for counters in (None,(1,0,0),(0,1,0),(0,0,1)):
            n.platform_reference_counters=counters
            self.assertFalse(n._native_navigation_current(10.))
            self.assertEqual(n.native_gate.reason,'NATIVE_PLATFORM_REFERENCE_MISMATCH')

    def fixture(self):
        n=methods('_update_mission','_update_native_mission','_blend_angle',
                  '_tracking_policy_current','_waypoint_policy_current','_clear_target')()
        n.platform_input_mode='px4';n.guidance_invalidated=False
        n.platform_pos=np.array([10.,20.,100.]);n.platform_vel=np.zeros(3)
        n.control_enabled=True;n.flight_state='takeoff';n.target_locked=True
        n.vision_rel=np.array([3.,1.,2.]);n.est_pos=np.array([13.,21.,999.]);n.vision_time=10.
        n.target_vel_est=np.array([0.,0.,50.]);n.standoff_m=1.;n.controller_gain=1.
        n.track_max_speed=2.;n.search_speed=1.;n.command_timeout_s=.5;n.yaw_cmd=0.
        n.horizontal_fault_reason='';n.native_tracking_started=False
        n.horizontal_navigation_mode='hold';n.horizontal_max_radius_m=2.
        n.horizontal_max_speed_mps=.5
        n.standoff_calibration_confirmed=n.camera_extrinsics_confirmed=True
        n.standoff_calibration_id=n.camera_extrinsics_id='fixture-confirmed'
        n.last_target_sample=100_000_000_000;n.vision_state_time=10.;n.sensor_timeout_s=.3
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000))
        n._native_navigation_current=lambda now:False
        return n

    def test_no_native_gate_even_target_and_time_cannot_start_takeoff(self):
        n=self.fixture()
        for now in (10.,20.,1000.):
            v,valid=n._update_mission(now,.01)
            self.assertFalse(valid);np.testing.assert_array_equal(v,[0.,0.,0.])
            self.assertEqual(n.flight_state,'wait_native')

    def test_production_horizontal_only_ignores_simulated_absolute_altitudes(self):
        n=self.fixture();n._native_navigation_current=lambda now:True
        v,valid=n._update_mission(10.,.01)
        self.assertTrue(valid);self.assertGreater(np.linalg.norm(v[:2]),0.)
        self.assertEqual(v[2],0.);self.assertEqual(n.flight_state,'track')
        n.control_enabled=False
        v,valid=n._update_mission(10.,.01)
        self.assertFalse(valid);self.assertEqual(n.flight_state,'native_height_hold')

    def test_missing_target_and_unconfirmed_waypoint_keeps_native_hold(self):
        n=self.fixture();n._native_navigation_current=lambda now:True
        n.target_locked=False;n.platform_history=NS(identity=('source',0,'origin'))
        n.waypoint_alignment_confirmed=False;n.waypoint_alignment_origin_id=''
        v,valid=n._update_mission(10.,.01)
        self.assertFalse(valid);np.testing.assert_array_equal(v,[0.,0.,0.])
        self.assertEqual(n.flight_state,'native_height_hold')


if __name__=='__main__':unittest.main()

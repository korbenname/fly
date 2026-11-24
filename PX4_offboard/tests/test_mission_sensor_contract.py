"""独立解析值的传感器边界测试，无ROS图或飞行输出。"""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'target/src/fc01_mission'))
from fc01_mission.sensor_contract import MeasurementIdentity,PlatformHistory,rotation_xyzw,slerp


def platform(ns=100_000_000_000, mono=10., **changes):
    data=dict(schema_version=1,source_id='bridge_session',reset_counter=0,origin_id='origin0',
        frame_id='px4_local_enu',body_frame_id='base_link_flu',ready=True,valid_fields=15,reason='OK',
        position=[3.,7.,1.5],velocity=[.2,.3,.4],attitude_xyzw=[0.,0.,0.,1.],angular_velocity=[.1,.2,.3],
        local_receive_monotonic_ns=int(mono*1e9),attitude_receive_monotonic_ns=int(mono*1e9),angular_velocity_receive_monotonic_ns=int(mono*1e9),
        local_timestamp_sample_us=ns//1000,attitude_timestamp_sample_us=ns//1000,angular_velocity_timestamp_sample_us=ns//1000)
    for name in ('position','velocity','attitude','angular_velocity'):data[name+'_stamp_ns']=ns
    data.update(changes);return NS(**data)


class MissionSensorContractTests(unittest.TestCase):
    def test_identity_duplicate_never_renews_and_regression_needs_epoch(self):
        x=MeasurementIdentity();self.assertEqual(x.observe('a',0,1,100,1.),'OK')
        self.assertEqual(x.observe('a',0,1,100,2.),'DUPLICATE');self.assertEqual(x.received,1.)
        self.assertEqual(x.observe('a',0,2,90,3.),'SOURCE_REGRESSED')
        self.assertEqual(x.observe('a',0,3,110,4.),'SOURCE_REGRESSED')
        self.assertEqual(x.observe('a',1,1,120,5.),'SOURCE_RESET')
        self.assertEqual(x.observe('a',0,4,130,6.),'RETIRED_OR_TOO_MANY_SOURCES')

    def test_identity_restart_quarantines_retired_source(self):
        x=MeasurementIdentity();x.observe('old',0,100,1000,1.)
        self.assertEqual(x.observe('new',0,1,1100,2.),'SOURCE_RESET')
        self.assertEqual(x.observe('new',0,2,1200,3.),'OK')
        self.assertEqual(x.observe('old',0,101,1300,4.),'RETIRED_OR_TOO_MANY_SOURCES')

    def test_interpolation_at_measurement_time_no_first_sample_zeroing(self):
        h=PlatformHistory();m=platform();self.assertTrue(h.accept(m,100_000_000_000,10.))
        later=platform(100_040_000_000,10.04,position=[3.04,7.,1.5],attitude_xyzw=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        self.assertTrue(h.accept(later,100_040_000_000,10.04))
        a=h.aligned(100_020_000_000)
        np.testing.assert_allclose(a.position,[3.02,7.,1.5])
        np.testing.assert_allclose(a.rotation@[1.,0.,0.],[math.sqrt(.5),math.sqrt(.5),0.],atol=1e-12)
        self.assertRaises(ValueError,h.aligned,99_990_000_000)

    def test_independent_receive_and_source_deadlines(self):
        for change in ({'angular_velocity_stamp_ns':99_600_000_000},
                       {'angular_velocity_receive_monotonic_ns':9_600_000_000},
                       {'velocity_stamp_ns':99_600_000_000}):
            h=PlatformHistory();self.assertFalse(h.accept(platform(**change),100_000_000_000,10.))
        h=PlatformHistory();h.accept(platform(),100_000_000_000,10.)
        self.assertFalse(h.current(100_350_000_000,10.35))

    def test_duplicate_snapshot_cannot_refresh_same_measurement(self):
        h=PlatformHistory();m=platform();h.accept(m,100_000_000_000,10.)
        h.accept(m,100_100_000_000,10.1)
        self.assertEqual(h.received['position'],10.)
        self.assertFalse(h.current(100_100_000_000,10.31))

    def test_invalid_replay_cannot_revalidate_same_raw_sample(self):
        h=PlatformHistory();m=platform();h.accept(m,100_000_000_000,10.)
        bad=platform(100_010_000_000,10.01,ready=False,valid_fields=0,reason='RESET')
        self.assertFalse(h.accept(bad,100_010_000_000,10.01))
        replay=platform(100_010_000_000,10.01)
        self.assertFalse(h.accept(replay,100_020_000_000,10.02))
        self.assertTrue(h.accept(platform(100_030_000_000,10.03),100_030_000_000,10.03))

    def test_malformed_ready_snapshot_consumes_all_samples_before_rejection(self):
        for field,bad in [('position',[math.nan,0.,0.]),('attitude_xyzw',[0.,0.,0.,0.])]:
            with self.subTest(field=field):
                h=PlatformHistory();h.accept(platform(),100_000_000_000,10.)
                self.assertFalse(h.accept(platform(100_010_000_000,10.01,**{field:bad}),100_010_000_000,10.01))
                self.assertFalse(h.accept(platform(100_010_000_000,10.01),100_020_000_000,10.02))
                self.assertTrue(all(not entries for entries in h.fields.values()))
                self.assertTrue(h.accept(platform(100_030_000_000,10.03),100_030_000_000,10.03))

    def test_local_reset_discards_old_image_history(self):
        h=PlatformHistory();h.accept(platform(),100_000_000_000,10.)
        self.assertTrue(h.accept(platform(100_020_000_000,10.02,origin_id='origin1',reset_counter=1),100_020_000_000,10.02))
        self.assertRaises(ValueError,h.aligned,100_000_000_000)
        self.assertFalse(h.accept(platform(100_030_000_000,10.03),100_030_000_000,10.03))

    def test_nan_frame_nonunit_attitude_and_alignment_reject(self):
        for change in ({'position':[math.nan,0,0]},{'attitude_xyzw':[0,0,0,0]},
                       {'frame_id':'fc01_vision_ned'},{'attitude_stamp_ns':99_900_000_000}):
            h=PlatformHistory();self.assertFalse(h.accept(platform(**change),100_000_000_000,10.))

    def test_optional_omega_is_none_not_fake_zero(self):
        h=PlatformHistory(require_omega=False)
        self.assertTrue(h.accept(platform(valid_fields=7,angular_velocity=[math.nan]*3),100_000_000_000,10.))
        self.assertIsNone(h.latest.omega);self.assertIsNone(h.aligned(100_000_000_000).omega)

    def test_multiple_publishers_and_clock_regression_fail(self):
        h=PlatformHistory();self.assertFalse(h.accept(platform(),100_000_000_000,10.,2))
        h=PlatformHistory();h.accept(platform(),100_000_000_000,10.)
        self.assertFalse(h.accept(platform(100_010_000_000,10.01),99_900_000_000,10.01))
        self.assertTrue(h.clock_fault)

    def test_quaternion_sign_equivalence_and_pitch_right_vector(self):
        q=[0.,math.sin(math.pi/12),0.,math.cos(math.pi/12)]
        np.testing.assert_allclose(slerp(q,[-x for x in q],.5),q,atol=1e-12)
        np.testing.assert_allclose(rotation_xyzw(q)@[0.,-1.,0.],[0.,-1.,0.],atol=1e-12)

if __name__=='__main__':unittest.main()

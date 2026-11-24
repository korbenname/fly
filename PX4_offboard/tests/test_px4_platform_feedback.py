"""实际平台反馈纯模块边界回归，不启动ROS、不读取飞控、不生成飞行输出。"""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

MODULE = (
    Path(__file__).resolve().parents[1]
    / 'target/src/fc01_mission/fc01_mission/platform_feedback.py'
)
spec = importlib.util.spec_from_file_location('fc01_platform_feedback_test', MODULE)
p = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = p
spec.loader.exec_module(p)


def local(us, **changes):
    values = dict(
        timestamp=us,
        timestamp_sample=us,
        xy_valid=True,
        z_valid=True,
        v_xy_valid=True,
        v_z_valid=True,
        x=1.0,
        y=2.0,
        z=-1.5,
        vx=0.2,
        vy=0.3,
        vz=-0.1,
        xy_reset_counter=0,
        z_reset_counter=0,
        vxy_reset_counter=0,
        vz_reset_counter=0,
        heading_reset_counter=0,
        ref_timestamp=123,
        dead_reckoning=False,
    )
    values.update(changes)
    return SimpleNamespace(**values)


def attitude(us, **changes):
    values = dict(timestamp=us, timestamp_sample=us, q=[1.0, 0.0, 0.0, 0.0], quat_reset_counter=0)
    values.update(changes)
    return SimpleNamespace(**values)


def omega(us, **changes):
    values = dict(timestamp=us, timestamp_sample=us, xyz=[0.1, 0.2, 0.3])
    values.update(changes)
    return SimpleNamespace(**values)


def rotate_xyzw(q, v):
    # 独立向量公式验证结果，不重复被测的四元数乘法实现。
    x, y, z, w = q
    cross = (y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0])
    cross2 = (y * cross[2] - z * cross[1], z * cross[0] - x * cross[2], x * cross[1] - y * cross[0])
    return tuple(v[i] + 2 * w * cross[i] + 2 * cross2[i] for i in range(3))


class PX4PlatformFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.us = 100_000_000
        self.host = self.us * 1000
        self.mono = 1_000_000_000
        self.f = p.PlatformFeedback('fixture-session')

    def feed(self, us=None, host=None, mono=None):
        us = self.us if us is None else us
        host = self.host if host is None else host
        mono = self.mono if mono is None else mono
        self.f.local(local(us), host, mono, 'local-gid')
        self.f.attitude(attitude(us), host, mono, 'attitude-gid')
        self.f.angular_velocity(omega(us), host, mono, 'omega-gid')

    def test_coordinates_and_absolute_local_origin(self):
        self.feed()
        s = self.f.snapshot(self.host, self.mono)
        self.assertTrue(s['ready'])
        self.assertEqual(s['position'], (2.0, 1.0, 1.5))
        self.assertEqual(s['velocity'], (0.3, 0.2, 0.1))
        self.assertEqual(s['angular_velocity'], (0.1, -0.2, -0.3))
        self.assertIn('123', s['origin_id'])
        # PX4机头北=ENU北，FLU的前轴从body x转到ENU y。
        self.assertSequenceAlmostEqual(rotate_xyzw(s['attitude_xyzw'], [1, 0, 0]), [0, 1, 0])

    def assertSequenceAlmostEqual(self, a, b):
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, places=12)

    def test_yaw_90_degree_and_tilt(self):
        root = math.sqrt(0.5)
        q = p.quaternion_ned_frd_to_enu_flu([root, 0, 0, root])
        # PX4 yaw90朝东 -> ENU x；FLU左轴朝北。
        self.assertSequenceAlmostEqual(rotate_xyzw(q, [1, 0, 0]), [1, 0, 0])
        self.assertSequenceAlmostEqual(rotate_xyzw(q, [0, 1, 0]), [0, 1, 0])
        angle = 0.3
        q = p.quaternion_ned_frd_to_enu_flu([math.cos(angle / 2), math.sin(angle / 2), 0, 0])
        self.assertSequenceAlmostEqual(
            rotate_xyzw(q, [0, 1, 0]), [-math.cos(angle), 0, math.sin(angle)]
        )

    def test_nan_and_invalid_quaternion_revoke(self):
        self.feed()
        self.f.attitude(
            attitude(self.us + 1, q=[0, 0, 0, 0]),
            self.host + 1000,
            self.mono + 1000,
            'attitude-gid',
        )
        self.assertFalse(
            self.f.snapshot(self.host + 1000, self.mono + 1000)['valid_fields'] & p.ATTITUDE
        )
        self.f.local(
            local(self.us + 1, x=float('nan')), self.host + 1000, self.mono + 1000, 'local-gid'
        )
        s = self.f.snapshot(self.host + 1000, self.mono + 1000)
        self.assertFalse(s['valid_fields'] & p.POSITION)
        self.assertTrue(s['valid_fields'] & p.VELOCITY)
        self.assertTrue(math.isnan(s['position'][0]))

    def test_position_and_velocity_valid_are_independent(self):
        self.feed()
        self.f.local(
            local(self.us + 1, v_xy_valid=False), self.host + 1000, self.mono + 1000, 'local-gid'
        )
        s = self.f.snapshot(self.host + 1000, self.mono + 1000)
        self.assertTrue(s['valid_fields'] & p.POSITION)
        self.assertFalse(s['valid_fields'] & p.VELOCITY)
        self.assertIn('VELOCITY_INVALID', s['reason'])

    def test_duplicate_raw_sample_does_not_extend_receive_ttl(self):
        self.feed()
        self.f.local(
            local(self.us + 100_000, timestamp_sample=self.us),
            self.host + 100_000_000,
            self.mono + 100_000_000,
            'local-gid',
        )
        s = self.f.snapshot(self.host + 251_000_000, self.mono + 251_000_000)
        self.assertFalse(s['ready'])
        self.assertEqual(s['local_receive_monotonic_ns'], self.mono)

    def test_old_future_zero_and_sample_after_envelope_rejected(self):
        for us, sample, host in (
            (self.us - 300_000, self.us - 300_000, self.host),
            (self.us + 21_000, self.us + 21_000, self.host),
            (0, 0, self.host),
            (self.us, self.us + 1, self.host),
        ):
            f = p.PlatformFeedback('fixture')
            f.local(local(us, timestamp_sample=sample), host, self.mono, 'l')
            self.assertEqual(f.snapshot(host, self.mono)['valid_fields'] & 3, 0)

    def test_source_age_and_monotonic_age_both_required(self):
        self.feed()
        self.assertFalse(self.f.snapshot(self.host + 251_000_000, self.mono)['ready'])
        f = p.PlatformFeedback('fixture')
        self.f = f
        self.feed()
        self.assertFalse(f.snapshot(self.host, self.mono + 251_000_000)['ready'])

    def test_omega_independently_stales_without_zero_fallback(self):
        self.feed()
        step = 260_000
        self.f.local(
            local(self.us + step), self.host + step * 1000, self.mono + step * 1000, 'local-gid'
        )
        self.f.attitude(
            attitude(self.us + step),
            self.host + step * 1000,
            self.mono + step * 1000,
            'attitude-gid',
        )
        s = self.f.snapshot(self.host + step * 1000, self.mono + step * 1000)
        self.assertEqual(s['valid_fields'] & 7, 7)
        self.assertFalse(s['valid_fields'] & p.ANGULAR_VELOCITY)
        self.assertFalse(s['ready'])
        self.assertTrue(math.isnan(s['angular_velocity'][0]))
        self.assertIn('ANGULAR_VELOCITY', s['reason'])

    def test_omega_optional_policy_explicit(self):
        self.f = p.PlatformFeedback('fixture', require_angular_velocity=False)
        self.f.local(local(self.us), self.host, self.mono, 'l')
        self.f.attitude(attitude(self.us), self.host, self.mono, 'a')
        s = self.f.snapshot(self.host, self.mono)
        self.assertTrue(s['ready'])
        self.assertFalse(s['require_angular_velocity'])
        self.assertFalse(s['valid_fields'] & p.ANGULAR_VELOCITY)

    def test_time_alignment_not_just_independent_freshness(self):
        self.feed()
        self.f.attitude(
            attitude(self.us + 100_000),
            self.host + 100_000_000,
            self.mono + 100_000_000,
            'attitude-gid',
        )
        s = self.f.snapshot(self.host + 100_000_000, self.mono + 100_000_000)
        self.assertFalse(s['valid_fields'] & p.ATTITUDE)
        self.assertIn('TIME_ALIGNMENT', s['reason'])

    def test_local_reset_explicit_invalid_then_new_epoch(self):
        self.feed()
        self.f.local(
            local(self.us + 1, z_reset_counter=1, z=42),
            self.host + 1000,
            self.mono + 1000,
            'local-gid',
        )
        s = self.f.snapshot(self.host + 1000, self.mono + 1000)
        self.assertEqual(s['valid_fields'], 0)
        self.assertEqual(s['reset_counter'], 1)
        self.assertIn('COORDINATE_RESET', s['reason'])
        self.f.attitude(attitude(self.us + 1), self.host + 1000, self.mono + 1000, 'attitude-gid')
        self.f.angular_velocity(omega(self.us + 1), self.host + 1000, self.mono + 1000, 'omega-gid')
        s = self.f.snapshot(self.host + 1000, self.mono + 1000)
        self.assertTrue(s['ready'])
        self.assertEqual(s['position'], (2, 1, -42))

    def test_attitude_reset_and_writer_restart_change_epoch(self):
        self.feed()
        self.f.attitude(
            attitude(self.us + 1, quat_reset_counter=1),
            self.host + 1000,
            self.mono + 1000,
            'attitude-gid',
        )
        self.assertEqual(self.f.snapshot(self.host + 1000, self.mono + 1000)['valid_fields'], 0)
        self.f.local(local(self.us + 2), self.host + 2000, self.mono + 2000, 'new-local-gid')
        self.assertEqual(self.f.snapshot(self.host + 2000, self.mono + 2000)['reset_counter'], 2)

    def test_regressed_sample_latches_until_new_writer_epoch(self):
        self.feed()
        self.f.local(local(self.us - 1), self.host, self.mono + 1000, 'local-gid')
        self.f.local(local(self.us + 1), self.host + 1000, self.mono + 2000, 'local-gid')
        self.assertFalse(self.f.snapshot(self.host + 1000, self.mono + 2000)['ready'])
        self.assertIn('REGRESSED', self.f.snapshot(self.host + 1000, self.mono + 2000)['reason'])
        self.f.local(local(self.us + 2), self.host + 2000, self.mono + 3000, 'new-local-gid')
        self.assertIn(
            'SESSION_CHANGED', self.f.snapshot(self.host + 2000, self.mono + 3000)['reason']
        )

    def test_invalid_new_sample_cannot_be_repaired_with_same_stamp(self):
        self.feed()
        self.f.local(
            local(self.us + 1, xy_valid=False, z_valid=False, v_xy_valid=False, v_z_valid=False),
            self.host + 1000,
            self.mono + 1000,
            'local-gid',
        )
        self.f.local(local(self.us + 1), self.host + 1000, self.mono + 2000, 'local-gid')
        s = self.f.snapshot(self.host + 1000, self.mono + 2000)
        self.assertFalse(s['valid_fields'] & 3)
        self.assertEqual(s['local_receive_monotonic_ns'], self.mono + 1000)

    def test_multiple_publishers_and_host_clock_regression_revoke(self):
        self.feed()
        self.assertFalse(self.f.snapshot(self.host, self.mono, {'local': 2})['ready'])
        self.assertFalse(self.f.snapshot(self.host, self.mono)['ready'])
        self.f.local(local(self.us + 1), self.host + 1000, self.mono + 1000, 'local-gid')
        self.assertTrue(self.f.snapshot(self.host + 1000, self.mono + 1000)['ready'])
        self.assertFalse(self.f.snapshot(self.host - 1, self.mono + 1001)['ready'])
        self.assertIn(
            'HOST_CLOCK_REGRESSED', self.f.snapshot(self.host + 1001, self.mono + 1002)['reason']
        )

    def test_dead_reckoning_and_invalid_last_payload_revoke(self):
        self.feed()
        self.f.local(
            local(self.us + 1, dead_reckoning=True), self.host + 1000, self.mono + 1000, 'local-gid'
        )
        s = self.f.snapshot(self.host + 1000, self.mono + 1000)
        self.assertFalse(s['ready'])
        self.assertFalse(s['valid_fields'] & 3)


if __name__ == '__main__':
    unittest.main()

"""高帧率相机不能饿死较低频平台采样的真实时间插值，不放宽源戳或TTL。"""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'target/src/fc01_mission'), str(ROOT / 'tests')]
from fc01_mission.sensor_contract import PlatformHistory, MeasurementIdentity, fresh, stamp_ns
from test_mission_sensor_contract import platform


def implementation():
    path = ROOT / 'target/src/fc01_mission/fc01_mission/mission_node.py'
    tree = ast.parse(path.read_text())
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == 'MissionNode')
    selected = ('_on_state', '_consume_pending', '_ingest', '_clear_target')
    module = ast.Module(
        body=[
            ast.ClassDef(
                name='Actual',
                bases=[],
                keywords=[],
                decorator_list=[],
                body=[x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name in selected],
            )
        ],
        type_ignores=[],
    )
    ast.fix_missing_locations(module)
    import time

    namespace = dict(
        np=np,
        time=time,
        fresh=fresh,
        stamp_ns=stamp_ns,
        finite=lambda values, size: np.asarray(values, dtype=float),
    )
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace['Actual']


def fixture():
    node = implementation()()
    node.platform_history = PlatformHistory()
    node.target_identity = MeasurementIdentity()
    node.pending_target = None
    node.sensor_timeout_s = 0.3
    node.target_lock_frames = 3
    node.target_consecutive = 0
    node.last_target_sample = 0
    node.est_pos = node.vision_rel = None
    node.est_time = 0.0
    node.target_vel_est = np.zeros(3)
    node.camera_pos = np.zeros(3)
    node.target_locked = False
    node.platform_input_mode = 'px4'
    node.native_tracking_started = False
    node.horizontal_fault_reason = ''
    node.camera_extrinsics_confirmed = True
    node.camera_extrinsics_id = 'synthetic'
    node.vision_state_topic = '/target'
    node.count_publishers = lambda _: 1
    node.now_ns = 100_000_000_000
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=node.now_ns))
    node._message_age = lambda _: 0.0
    node.terminal_start = None
    node.flight_state = 'native_height_hold'
    node.terminal_cmd = node.last_v_cmd = np.zeros(3)
    node.now_mono = lambda: 10.0 + (node.now_ns - 100_000_000_000) * 1e-9
    return node


def frame(ns, seq, valid=True):
    return NS(
        stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9),
        source_id='camera',
        reset_counter=0,
        sequence=seq,
        schema_version=1,
        frame_id='body_frd',
        body_frame_id='body_frd',
        initialized=valid,
        tracking=valid,
        calibration_valid=valid,
        valid_fields=1,
        POSITION=1,
        VELOCITY=2,
        POSE_COVARIANCE=16,
        TWIST_COVARIANCE=32,
        QUALITY=64,
        position=[2.0, 0.2, 0.0],
        pose_covariance=[0.0] * 36,
        twist_covariance=[0.0] * 36,
    )


class PendingAlignmentTests(unittest.TestCase):
    def test_camera_120hz_cannot_starve_platform_50hz_bracketing(self):
        n = fixture()
        # 每个平台样本到达前先有新相机帧，复现异步DDS正常到达顺序。
        # 若总把pending换成最新帧，它永远在最新平台样本之后，无法完成插值。
        with patch('time.monotonic', side_effect=n.now_mono):
            for ms in range(0, 401):
                n.now_ns = 100_000_000_000 + ms * 1_000_000
                if ms % 8 == 0:
                    n._on_state(frame(n.now_ns, ms // 8 + 1))
                if ms % 20 == 0:
                    sample = n.now_ns - 15_000_000
                    n.platform_history.accept(
                        platform(sample, n.now_mono()), n.now_ns, n.now_mono()
                    )
                    n._consume_pending(n.now_mono())
        self.assertTrue(n.target_locked)
        self.assertGreaterEqual(n.target_consecutive, 10)
        self.assertGreater(n.last_target_sample, 100_350_000_000)
        self.assertEqual(n.horizontal_fault_reason, '')

    def test_new_invalid_frame_clears_pending_before_future_platform_can_revive(self):
        n = fixture()
        with patch('time.monotonic', side_effect=n.now_mono):
            n.platform_history.accept(platform(99_998_000_000, 10.0), n.now_ns, 10.0)
            n._on_state(frame(n.now_ns, 1))
            self.assertIsNotNone(n.pending_target)
            n.now_ns += 8_000_000
            n._on_state(frame(n.now_ns, 2, valid=False))
            self.assertIsNone(n.pending_target)
            n.platform_history.accept(platform(n.now_ns, n.now_mono()), n.now_ns, n.now_mono())
            n._consume_pending(n.now_mono())
        self.assertEqual(n.last_target_sample, 0)
        self.assertFalse(n.target_locked)

    def test_new_frames_cannot_refresh_expired_pending_or_clear_active_fault(self):
        n = fixture()
        with patch('time.monotonic', side_effect=n.now_mono):
            n.platform_history.accept(platform(99_998_000_000, 10.0), n.now_ns, 10.0)
            n._on_state(frame(n.now_ns, 1))
            pending = n.pending_target
            n.native_tracking_started = True
            n.now_ns += 350_000_000
            # 有新图也不能把旧pending源时刻续期，仍走原故障撤销。
            n._on_state(frame(n.now_ns, 2))
        self.assertEqual(n.horizontal_fault_reason, 'TARGET_PENDING_STALE')
        self.assertNotEqual(n.pending_target[1], pending[1])
        self.assertFalse(n.target_locked)

    def test_source_reset_retracts_pending_before_alignment(self):
        n = fixture()
        with patch('time.monotonic', side_effect=n.now_mono):
            n.platform_history.accept(platform(99_998_000_000, 10.0), n.now_ns, 10.0)
            n._on_state(frame(n.now_ns, 1))
            n.now_ns += 8_000_000
            changed = frame(n.now_ns, 1)
            changed.source_id = 'restarted-camera'
            n._on_state(changed)
        self.assertIsNone(n.pending_target)
        self.assertEqual(n.target_reason, 'SOURCE_RESET')
        self.assertFalse(n.target_locked)


if __name__ == '__main__':
    unittest.main()

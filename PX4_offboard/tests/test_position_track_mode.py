"""Position-track maintenance transactions with recorded site fixtures only."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import Mock

import navigation_prebuilt_service as service
import navigation_vision_mode as modes
from tests.test_navigation_vision_mode import Setup, site


class PositionTrackModeTests(unittest.TestCase):
    def test_position_runs_without_vision_or_a_site(self):
        with tempfile.TemporaryDirectory() as directory:
            f = Setup(directory)
            modes.configure(f.entry, 'position', run=f.run, check_stopped=f.stopped)
            _, params = modes.load_parameters(f.dep / 'runtime.yaml', 'fc01_system')
            self.assertIs(params['px4_takeoff_position_hold'], True)
            self.assertIs(params.get('px4_position_mission_enabled', False), False)
            self.assertEqual(modes.configured_mode(f.entry, params),
                             dict(mode='position', enabled=False))
            popen = Mock()
            auxiliary = service.VisionAuxiliary(f.entry, modes.configured_mode(f.entry, params),
                                                popen=popen)
            auxiliary.start()
            auxiliary.close()
            popen.assert_not_called()
            status = json.loads((f.dep / 'vision-runtime.json').read_text())
            self.assertEqual(status['mode'], 'position')
            self.assertFalse(status['mission_control_enabled'])

    def test_position_track_is_explicit_and_fixes_the_production_entry(self):
        with tempfile.TemporaryDirectory() as directory:
            f = Setup(directory)
            result = modes.configure(f.entry, 'position-track', f.site, run=f.run,
                                     check_stopped=f.stopped)
            _, params = modes.load_parameters(f.dep / 'runtime.yaml', 'fc01_system')
            current = modes.configured_mode(f.entry, params)
            self.assertEqual(current['mode'], 'position-track')
            self.assertTrue(result['mission_control_enabled'])
            self.assertFalse(result['automatic_arm'])
            for key in ('px4_takeoff_position_hold', 'px4_position_mission_enabled',
                        'visual_navigation_enabled', 'px4_horizontal_enabled'):
                self.assertIs(params[key], True)
                with self.subTest(incomplete_flag=key):
                    with self.assertRaises(ValueError):
                        modes.configured_mode(f.entry, dict(params, **{key: False}))
            _, mission = modes.load_parameters(current['mission_path'], 'fc01_mission')
            self.assertEqual(mission['engage_mode'], 'track')
            self.assertEqual(mission['horizontal_navigation_mode'], 'hold')
            self.assertIs(mission['position_takeoff_anchor_authority'], True)
            self.assertEqual(mission['takeoff_altitude_m'], 1.5)
            self.assertIs(mission['control_enabled'], True)
            self.assertFalse(Path('/dev/v4l/by-id/site-camera').exists())
            self.assertEqual(f.calls, [['/bin/bash', str(f.entry / 'prebuilt_boot.sh'), 'check']])

    def test_unverified_or_outside_envelope_site_cannot_change_position_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            f = Setup(directory)
            path = f.dep / 'runtime.yaml'
            before = path.read_bytes()
            for value, message in (
                ({'horizontal_max_radius_m': .5}, '安全间距'),
                ({'horizontal_max_speed_mps': .7}, '超出配置界限'),
                ({'horizontal_navigation_mode': 'waypoint'}, '原点确认'),
            ):
                with self.subTest(site_mission_values=value):
                    record = site()
                    record['mission'].update(value)
                    f.site.write_text(json.dumps(record))
                    with self.assertRaisesRegex(ValueError, message):
                        modes.configure(f.entry, 'position-track', f.site, run=f.run,
                                        check_stopped=f.stopped)
            with self.assertRaisesRegex(ValueError, '--site'):
                modes.configure(f.entry, 'position-track', run=f.run, check_stopped=f.stopped)
            with self.assertRaisesRegex(ValueError, 'pending-calibration'):
                modes.configure(f.entry, 'position-track', pending_calibration=True,
                                run=f.run, check_stopped=f.stopped)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(f.calls, [])

    def test_position_track_migrates_waypoint_and_can_revert_to_position_hold(self):
        with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
            old, new = Setup(first), Setup(second)
            record = site()
            record['mission'].update(horizontal_navigation_mode='waypoint',
                                     waypoint_alignment_confirmed=True,
                                     waypoint_alignment_origin_id='px4_local:1000:0:0:0:0:0',
                                     navigation_waypoint_x=3.45, navigation_waypoint_y=.2,
                                     horizontal_max_speed_mps=.2)
            old.site.write_text(json.dumps(record))
            modes.configure(old.entry, 'position-track', old.site, run=old.run,
                            check_stopped=old.stopped)
            shutil.copy2(old.dep / 'runtime.yaml', new.dep / 'runtime.yaml')
            modes.migrate(new.entry, old.entry)
            _, params = modes.load_parameters(new.dep / 'runtime.yaml', 'fc01_system')
            current = modes.configured_mode(new.entry, params)
            self.assertEqual(current['mode'], 'position-track')
            _, mission = modes.load_parameters(current['mission_path'], 'fc01_mission')
            # 绝对 ENU 航点离坐标原点可超过半径，实际半径相对本次起飞点。
            self.assertEqual(mission['navigation_waypoint_x'], 3.45)
            self.assertEqual(params['visual_max_speed'], .2)
            self.assertIs(params['px4_position_waypoint_enabled'], True)
            self.assertEqual(params['px4_position_waypoint_origin_id'], 'px4_local:1000:0:0:0:0:0')
            self.assertEqual(params['px4_position_waypoint_north_m'], .2)
            self.assertEqual(params['px4_position_waypoint_east_m'], 3.45)
            with self.assertRaisesRegex(ValueError, '不一致'):
                modes.configured_mode(new.entry, dict(params, visual_max_speed=.4))
            with self.assertRaisesRegex(ValueError, '原生航点'):
                modes.configured_mode(new.entry, dict(params, px4_position_waypoint_east_m=2.))
            before = {path.name: path.read_bytes() for path in (new.dep / 'runtime.yaml',
                       new.dep / 'vision_mode.json', new.dep / 'vision_mission.yaml')}

            def failed(argv, **_kwargs):
                raise subprocess.CalledProcessError(1, argv)

            with self.assertRaises(subprocess.CalledProcessError):
                modes.configure(new.entry, 'position', run=failed, check_stopped=new.stopped)
            self.assertEqual(before, {name: (new.dep / name).read_bytes() for name in before})
            modes.configure(new.entry, 'position', run=new.run, check_stopped=new.stopped)
            _, params = modes.load_parameters(new.dep / 'runtime.yaml', 'fc01_system')
            self.assertIs(params['px4_position_mission_enabled'], False)
            self.assertIs(params['px4_position_waypoint_enabled'], False)
            self.assertEqual(modes.configured_mode(new.entry, params),
                             dict(mode='position', enabled=False))


if __name__ == '__main__':
    unittest.main()

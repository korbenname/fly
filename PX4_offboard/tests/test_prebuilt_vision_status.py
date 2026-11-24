"""预编译视觉状态只读回归：软件待机与飞机准入、原生授权与目标锁定分别显示。"""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import navigation
import navigation_status as display
import navigation_prebuilt_install as installer
from tests.test_navigation_feedback_freshness import ready_state
from tests.test_navigation_prebuilt_install import Fixture


def observed():
    return dict(schema_version=1,boot_id='test-boot',supervisor_pid=20,
        receive_monotonic_ns=100_000_000_000,mode='track',
        observations=dict(
            camera=dict(receive_monotonic_ns=99_900_000_000,publisher_count=1,
                value=dict(ready=True,reason='FRAME_CURRENT',
                    configuration='CONFIGURATION_CONFIRMED',missing=[])),
            mission=dict(receive_monotonic_ns=99_900_000_000,publisher_count=1,
                value=dict(target_locked=True,continuous_new_frames=5,
                    target_reason='TARGET_LOCKED',missing=[]))))



class PrebuiltVisionStatusTests(unittest.TestCase):
    def test_current_camera_and_target_are_displayed_without_granting_native_authority(self):
        info=dict(ActiveState='active',MainPID='20',boot_id='test-boot')
        lines=display.vision_display_lines(observed(),info,now=1000.1,monotonic=100.1)
        text='\n'.join(lines)
        self.assertIn('ready=true',text);self.assertIn('目标锁定：true',text)
        self.assertIn('不能代替原生控制授权',text)

    def test_outer_rewrite_cannot_refresh_camera_or_target_and_identity_is_required(self):
        info=dict(ActiveState='active',MainPID='20',boot_id='test-boot')
        value=observed();value['receive_monotonic_ns']=102_000_000_000
        text='\n'.join(display.vision_display_lines(value,info,now=1002.1,monotonic=102.1))
        self.assertIn('无新鲜诊断',text);self.assertIn('目标锁定：未知',text)
        self.assertNotIn('目标锁定：true',text)
        for key,item in (('boot_id','old-boot'),('supervisor_pid',99)):
            value=observed();value[key]=item
            text='\n'.join(display.vision_display_lines(value,info,now=1000.1,monotonic=100.1))
            self.assertIn('STALE',text);self.assertNotIn('ready=true',text)

    def test_clock_invalid_or_feedback_absent_remains_unknown(self):
        info=dict(ActiveState='active',MainPID='20',boot_id='test-boot')
        for key,item in (('receive_monotonic_ns',float('nan')),('receive_monotonic_ns',101_000_000_000),
                         ('schema_version',2),('schema_version',True),
                         ('receive_monotonic_ns',10**400),('supervisor_pid',None)):
            value=observed();value[key]=item
            self.assertFalse(display.vision_runtime_current(value,info,now=1000.1,monotonic=100.1))
        value=observed();value['observations'].clear()
        text='\n'.join(display.vision_display_lines(value,info,now=1000.1,monotonic=100.1))
        self.assertIn('无新鲜诊断',text);self.assertIn('目标锁定：未知',text)

    def test_altitude_mode_does_not_require_camera(self):
        info=dict(ActiveState='active',MainPID='20',boot_id='test-boot')
        value=observed();value['mode']='altitude';value['observations'].clear()
        text='\n'.join(display.vision_display_lines(value,info,now=1000.1,monotonic=100.1))
        self.assertIn('altitude',text);self.assertIn('相机/任务未启用',text)

    def test_installer_status_separates_vision_and_aircraft_and_never_mutates_observation(self):
        with tempfile.TemporaryDirectory() as root:
            fixture=Fixture(root);folder=fixture.old/'deployment';folder.mkdir()
            path=folder/'vision-runtime.json';path.write_text(json.dumps(observed()))
            fixture.info.update(MainPID='20')
            before=path.read_bytes()
            with patch.object(installer.time,'monotonic',return_value=100.1),\
                 patch.object(installer.time,'time',return_value=1000.1):
                vision=installer.vision_readiness(fixture,fixture.service())
                ready=installer.flight_readiness(fixture)
            self.assertTrue(vision['current']);self.assertTrue(vision['camera_current'])
            self.assertTrue(vision['mission']['target_locked'])
            self.assertFalse(ready['ready_for_manual_rc']);self.assertTrue(ready['missing'])
            self.assertEqual(path.read_bytes(),before)
            value=observed();value['observations']['camera']['receive_monotonic_ns']=98_000_000_000
            path.write_text(json.dumps(value))
            with patch.object(installer.time,'monotonic',return_value=100.1),\
                 patch.object(installer.time,'time',return_value=1000.1):
                self.assertFalse(installer.vision_readiness(fixture,fixture.service())['camera_current'])

    def test_multiple_publishers_or_malformed_child_cannot_show_a_current_lock(self):
        for count in (0,2,True):
            value=observed();value['observations']['mission']['publisher_count']=count
            self.assertIsNone(display.vision_observation(value,'mission',monotonic=100.1))
        value=observed();value['observations']['mission']={'invalid':'bad JSON',
            'receive_monotonic_ns':100_000_000_000}
        self.assertIsNone(display.vision_observation(value,'mission',monotonic=100.1))

    def test_actual_service_writer_is_readable_by_both_status_entrypoints(self):
        # 使用生产写入器实际落盘，避免只按测试自己发明的JSON契约得到假通过。
        import navigation_prebuilt_service as service
        with tempfile.TemporaryDirectory() as root:
            fixture=Fixture(root);(fixture.old/'deployment').mkdir()
            observer=service.VisionAuxiliary(fixture.old,dict(mode='track',enabled=True))
            observer.latest=observed()['observations']
            with patch.object(service.time,'monotonic_ns',return_value=100_000_000_000),\
                 patch.object(service.os,'getpid',return_value=20):
                observer._write()
            value=json.loads((fixture.old/'deployment/vision-runtime.json').read_text())
            info=fixture.service();info.update(MainPID='20',boot_id=value['boot_id'])
            fixture.boot_id=lambda:value['boot_id']
            self.assertTrue(display.vision_runtime_current(value,info,now=0.,monotonic=100.1))
            self.assertTrue(display.vision_observation(value,'mission',monotonic=100.1)['target_locked'])
            with patch.object(installer.time,'monotonic',return_value=100.1):
                actual=installer.vision_readiness(fixture,info)
            self.assertTrue(actual['current']);self.assertTrue(actual['camera_current'])
            self.assertTrue(actual['mission_current'])

    def test_track_install_acceptance_requires_actual_auxiliary_software_but_no_sensor_feedback(self):
        import os
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as d:
            entry=Path(d)/'entry';(entry/'deployment').mkdir(parents=True)
            marker=entry/'target/src/fc01_mission/package.xml';marker.parent.mkdir(parents=True)
            marker.write_text('<package/>')
            proc=Path(d)/'proc';proc.mkdir()
            nodes=('install/lib/fc01_vision/fc01_vision_node',
                   'install/lib/fc01_mission/px4_state_bridge','install/lib/fc01_mission/mission',
                   'install/lib/jikl_fc01_system/fc01_target_interface_node')
            rows={};hashes={}
            for pid,name in enumerate(nodes,31):
                path=entry/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'node')
                path=path.resolve();folder=proc/str(pid);folder.mkdir()
                argv=[str(path)] if name.endswith('_node') else ['/usr/bin/python3',str(path)]
                (folder/'cmdline').write_bytes(b'\0'.join(x.encode() for x in argv))
                hashes[name]=installer.digest(path)
                rows[pid]=dict(pid=pid,uid=os.getuid(),exe=str(path),sha256=hashes[name],argv=argv)
            rows[30]=dict(pid=30,uid=os.getuid(),argv=['/usr/bin/python3',str(entry/'install/bin/ros2'),
                'launch','fc01_bringup','horizontal_first_test.launch.py'])
            b=SimpleNamespace(uid=os.getuid(),proc=proc,boot_id=lambda:'test-boot',
                process=lambda pid:rows[pid],descendant=lambda pid,main:
                (pid==30 and main==20) or (pid in range(31,35) and main==30))
            value=observed();value.update(enabled=True,auxiliary_pid=30,auxiliary_returncode=None,error='')
            # 无目标、无飞机状态、相机未连接都不参与本机软件验收。
            value['observations'].clear()
            path=entry/'deployment/vision-runtime.json';path.write_text(json.dumps(value))
            config={'fc01_system':{'ros__parameters':{'visual_navigation_enabled':True}}}
            info=dict(MainPID='20');manifest=dict(payload_sha256=hashes)
            with patch.object(installer.time,'monotonic',return_value=100.1):
                installer.verify_vision_software(b,entry,info,config,manifest)
                for changed in ({'auxiliary_returncode':1},{'error':'VISION_AUXILIARY_EXITED:1'},
                                {'supervisor_pid':999},{'receive_monotonic_ns':90_000_000_000}):
                    path.write_text(json.dumps(dict(value,**changed)))
                    with self.assertRaises(installer.InstallError):
                        installer.verify_vision_software(b,entry,info,config,manifest)
                path.write_text(json.dumps(value))
                (proc/'32/cmdline').unlink()
                with self.assertRaisesRegex(installer.InstallError,'唯一track辅助软件节点'):
                    installer.verify_vision_software(b,entry,info,config,manifest)
                # 定高模式完全不依赖视觉文件和进程。
                config['fc01_system']['ros__parameters']['visual_navigation_enabled']=False
                path.unlink();installer.verify_vision_software(b,entry,info,config,manifest)

    def test_historical_ready_or_native_authority_never_displays_current_grant(self):
        state=ready_state();state.update(rc_target_height=1.5,navigation_allowed=True,
                                       visual_reason='HORIZONTAL_TRACK_PX4_VELOCITY_CLOSED_LOOP')
        row=dict(status='READY_WAIT_ARM',reason='old',supervisor_pid=10,controller_pid=22,
                 state_controller_pid=22,boot_id='test-boot',updated_unix=1000.,
                 state=state,state_age_seconds=0.,feedback_current=True,_vision_runtime=observed())
        info=dict(ActiveState='active',MainPID='99',boot_id='test-boot')
        with patch.object(navigation.time,'monotonic',return_value=100.1):
            text=navigation.render_status(row,info,now=1000.1)
        self.assertIn('原生到高授权：未知',text)
        self.assertNotIn('原生到高授权：true',text)
        self.assertNotIn('目标锁定：true',text)

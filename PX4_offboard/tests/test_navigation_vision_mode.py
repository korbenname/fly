"""地面配置和辅助链生命周期回归；不连接ROS、systemd或真实设备。"""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import Mock,patch
import yaml
import navigation_vision_mode as modes
import navigation_prebuilt_service as service

ROOT=Path(__file__).resolve().parents[1]


def site():
    return dict(schema_version=1,provenance={k:'现场证据/20261118/'+k for k in
        ('camera_device','intrinsics','extrinsics','target_model','capture_latency','standoff')},
        camera=dict(device_path='/dev/v4l/by-id/site-camera',camera_fx=466.7,camera_fy=466.7,
            camera_cx=320.,camera_cy=240.,frame_width=640,frame_height=480,frame_fps=30,
            camera_to_body_wxyz=[1.,0.,0.,0.],target_diameter_m=.2,calibration_confirmed=True,
            calibration_id='site-K',image_rectified_confirmed=True,extrinsics_confirmed=True,
            extrinsics_id='site-extrinsics',target_model_confirmed=True,target_model_id='site-circle',
            capture_latency_confirmed=True,capture_latency_bound_s=.01,target_shape='circle'),
        mission=dict(camera_extrinsics_confirmed=True,camera_extrinsics_id='site-extrinsics',
            standoff_calibration_confirmed=True,standoff_calibration_id='site-distance',
            camera_pos_x=.1,camera_pos_y=0.,camera_pos_z=.02,standoff_m=1.2))


class Setup:
    def __init__(self,path):
        self.entry=Path(path)/'release';self.dep=self.entry/'deployment';self.dep.mkdir(parents=True)
        for rel in ('fc01_vision/config/vision.yaml','fc01_mission/config/mission.yaml'):
            dest=self.entry/'target/src'/rel;dest.parent.mkdir(parents=True);shutil.copy2(ROOT/'target/src'/rel,dest)
        self.profile=yaml.safe_load((ROOT/'tests/fixtures/stage3_runtime_20261114.yaml').read_text())
        self.profile['fc01_system']['ros__parameters']['rc_target_height']=1.5
        self.profile['fc01_system']['ros__parameters']['rc_navigation_mode_slot']=1
        (self.dep/'runtime.yaml').write_text(yaml.safe_dump(self.profile))
        self.site=Path(path)/'site.json';self.site.write_text(json.dumps(site()))
        self.calls=[]
    def run(self,argv,**kwargs):self.calls.append(argv);return subprocess.CompletedProcess(argv,0,'','')
    def stopped(self,**kwargs):pass


class VisionModeTests(unittest.TestCase):
    def test_pending_track_is_software_only_and_does_not_claim_any_site_calibration(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);before=copy.deepcopy(f.profile['fc01_system']['ros__parameters'])
            result=modes.configure(f.entry,'track',pending_calibration=True,
                                   run=f.run,check_stopped=f.stopped)
            _,params=modes.load_parameters(f.dep/'runtime.yaml','fc01_system')
            current=modes.configured_mode(f.entry,params)
            self.assertTrue(params['visual_navigation_enabled']);self.assertTrue(params['px4_horizontal_enabled'])
            self.assertEqual(current['calibration_state'],'pending');self.assertFalse(current['control_enabled'])
            self.assertFalse(result['mission_control_enabled'])
            _,camera=modes.load_parameters(current['camera_path'],'/**')
            _,mission=modes.load_parameters(current['mission_path'],'fc01_mission')
            self.assertEqual(camera['device_path'],'/dev/fc01-camera-unconfigured')
            for key in ('calibration_confirmed','image_rectified_confirmed','extrinsics_confirmed',
                        'target_model_confirmed','capture_latency_confirmed'):
                self.assertIs(camera[key],False)
            for key in ('calibration_id','extrinsics_id','target_model_id'):
                self.assertEqual(camera[key],'')
            self.assertFalse(mission['control_enabled']);self.assertEqual(mission['engage_mode'],'track')
            self.assertFalse(mission['camera_extrinsics_confirmed'])
            self.assertFalse(mission['standoff_calibration_confirmed'])
            self.assertFalse(mission['waypoint_alignment_confirmed'])
            self.assertEqual(mission['horizontal_navigation_mode'],'hold')
            self.assertEqual(mission['standoff_m'],0.)
            for key,value in before.items():
                if not key.startswith('visual_') and key!='px4_horizontal_enabled':
                    self.assertEqual(params[key],value,key)
            self.assertEqual(json.loads((f.dep/'vision_site.json').read_text()),modes.PENDING_SITE)

    def test_pending_mode_does_not_mix_calibration_and_pending_marker_or_altitude(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);before=(f.dep/'runtime.yaml').read_bytes()
            for mode,path in [('altitude',None),('track',f.site)]:
                with self.assertRaisesRegex(ValueError,'pending-calibration'):
                    modes.configure(f.entry,mode,path,pending_calibration=True,
                                    run=f.run,check_stopped=f.stopped)
            f.site.write_text(json.dumps(modes.PENDING_SITE))
            with self.assertRaises(ValueError):
                modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            self.assertEqual((f.dep/'runtime.yaml').read_bytes(),before);self.assertEqual(f.calls,[])

    def test_pending_hashes_cannot_hide_calibration_confirmation_or_source_injection(self):
        for name in ('vision_camera.yaml','vision_site.json'):
            with self.subTest(name=name),tempfile.TemporaryDirectory() as d:
                f=Setup(d);modes.configure(f.entry,'track',pending_calibration=True,
                                         run=f.run,check_stopped=f.stopped)
                path=f.dep/name
                if name.endswith('.yaml'):
                    value=yaml.safe_load(path.read_text());value['/**']['ros__parameters']['calibration_confirmed']=True
                    path.write_text(yaml.safe_dump(value))
                else:
                    value=json.loads(path.read_text());value['camera']={'calibration_confirmed':True}
                    path.write_text(json.dumps(value))
                record=json.loads((f.dep/modes.MODE_FILE).read_text())
                record[('camera' if name.endswith('.yaml') else 'site')+'_sha256']=modes.sha(path)
                (f.dep/modes.MODE_FILE).write_text(json.dumps(record))
                _,params=modes.load_parameters(f.dep/'runtime.yaml','fc01_system')
                with self.assertRaises(ValueError):modes.configured_mode(f.entry,params)

    def test_pending_migrates_without_enabling_control_and_real_site_can_replace_it(self):
        with tempfile.TemporaryDirectory() as a,tempfile.TemporaryDirectory() as b:
            old,new=Setup(a),Setup(b)
            modes.configure(old.entry,'track',pending_calibration=True,run=old.run,check_stopped=old.stopped)
            shutil.copy2(old.dep/'runtime.yaml',new.dep/'runtime.yaml')
            modes.migrate(new.entry,old.entry)
            _,params=modes.load_parameters(new.dep/'runtime.yaml','fc01_system')
            current=modes.configured_mode(new.entry,params)
            self.assertEqual(current['calibration_state'],'pending');self.assertFalse(current['control_enabled'])
            modes.configure(new.entry,'track',new.site,run=new.run,check_stopped=new.stopped)
            _,params=modes.load_parameters(new.dep/'runtime.yaml','fc01_system')
            current=modes.configured_mode(new.entry,params)
            self.assertEqual(current['calibration_state'],'confirmed');self.assertTrue(current['control_enabled'])

    def test_track_requires_site_sources_without_camera_connected_and_preserves_rc_thrust(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);result=modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            after=yaml.safe_load((f.dep/'runtime.yaml').read_text())
            before=f.profile['fc01_system']['ros__parameters'];p=after['fc01_system']['ros__parameters']
            changed={k for k in set(before)|set(p) if before.get(k)!=p.get(k)}
            self.assertTrue(changed <= {'visual_navigation_enabled','px4_horizontal_enabled'}|{k for k in p if k.startswith('visual_')})
            self.assertEqual(p['rc_target_height'],1.5);self.assertEqual(p['rc_navigation_mode_slot'],1)
            self.assertEqual(p['aa_hover_thrust'],before['aa_hover_thrust'])
            current=modes.configured_mode(f.entry,p);self.assertEqual(current['mode'],'track')
            self.assertFalse(Path('/dev/v4l/by-id/site-camera').exists())
            self.assertTrue(Path(result['backup']).is_dir());self.assertEqual(len(f.calls),1)
            _,mission=modes.load_parameters(current['mission_path'],'fc01_mission')
            self.assertEqual(mission['engage_mode'],'track');self.assertEqual(mission['native_flight_topic'],'/fc01/native/flight_snapshot')

    def test_altitude_restore_switches_only_visual_flags(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            modes.configure(f.entry,'altitude',run=f.run,check_stopped=f.stopped)
            p=yaml.safe_load((f.dep/'runtime.yaml').read_text())['fc01_system']['ros__parameters']
            self.assertFalse(p['visual_navigation_enabled']);self.assertFalse(p['px4_horizontal_enabled'])
            self.assertEqual(modes.configured_mode(f.entry,p),dict(mode='altitude',enabled=False))

    def test_incomplete_nan_bad_quaternion_and_false_calibration_refuse_before_write(self):
        for key,value in [('capture_latency_bound_s',float('nan')),('camera_to_body_wxyz',[0,0,0,0]),('calibration_confirmed',False),('device_path','')]:
            with self.subTest(key=key),tempfile.TemporaryDirectory() as d:
                f=Setup(d);v=site();v['camera'][key]=value;f.site.write_text(json.dumps(v));before=(f.dep/'runtime.yaml').read_bytes()
                with self.assertRaises(ValueError):modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
                self.assertEqual((f.dep/'runtime.yaml').read_bytes(),before);self.assertEqual(f.calls,[])
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);v=site();del v['provenance']['intrinsics'];f.site.write_text(json.dumps(v))
            with self.assertRaisesRegex(ValueError,'provenance'):modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)

    def test_software_check_failure_restores_runtime_and_original_mode_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);before=(f.dep/'runtime.yaml').read_bytes()
            def failed(*args,**kwargs):raise subprocess.CalledProcessError(1,args[0])
            with self.assertRaises(subprocess.CalledProcessError):modes.configure(f.entry,'track',f.site,run=failed,check_stopped=f.stopped)
            self.assertEqual((f.dep/'runtime.yaml').read_bytes(),before)
            self.assertFalse((f.dep/'vision_mode.json').exists());self.assertFalse((f.dep/'vision_camera.yaml').exists())

    def test_mode_change_updates_only_current_install_receipt_and_failure_restores_it(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);state=f.dep/'INSTALL_STATE.json'
            original=dict(backup='/existing/original-backup',phase='active',dropin_sha256='pin',
                          runtime_sha256=modes.sha(f.dep/'runtime.yaml'))
            state.write_text(json.dumps(original))
            modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            receipt=json.loads(state.read_text())
            self.assertEqual(receipt,original|dict(runtime_sha256=modes.sha(f.dep/'runtime.yaml')))
            before=state.read_bytes();runtime=(f.dep/'runtime.yaml').read_bytes()
            def failed(*args,**kwargs):raise subprocess.CalledProcessError(1,args[0])
            with self.assertRaises(subprocess.CalledProcessError):
                modes.configure(f.entry,'altitude',run=failed,check_stopped=f.stopped)
            self.assertEqual(state.read_bytes(),before)
            self.assertEqual((f.dep/'runtime.yaml').read_bytes(),runtime)
            modes.configure(f.entry,'altitude',run=f.run,check_stopped=f.stopped)
            self.assertEqual(json.loads(state.read_text())['runtime_sha256'],modes.sha(f.dep/'runtime.yaml'))

    def test_unknown_runtime_edit_is_not_adopted_by_mode_configuration(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);state=f.dep/'INSTALL_STATE.json'
            state.write_text(json.dumps(dict(runtime_sha256='0'*64)))
            before=(f.dep/'runtime.yaml').read_bytes()
            with self.assertRaisesRegex(ValueError,'安装记录'):
                modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            self.assertEqual((f.dep/'runtime.yaml').read_bytes(),before);self.assertEqual(f.calls,[])

    def test_service_or_native_alive_blocks_without_reading_historical_status(self):
        run=Mock(return_value=subprocess.CompletedProcess([],0,'ActiveState=active\nMainPID=42\n',''))
        with self.assertRaisesRegex(ValueError,'停止'):modes.assert_stopped(run=run)
        with tempfile.TemporaryDirectory() as d:
            proc=Path(d);(proc/'123').mkdir();(proc/'123/exe').symlink_to('/tmp/fc01_system_node')
            run.return_value.stdout='ActiveState=inactive\nMainPID=0\n'
            with self.assertRaisesRegex(ValueError,'123'):modes.assert_stopped(run=run,proc=proc)
            (proc/'123/exe').unlink()
            modes.assert_stopped(run=run,proc=proc)

    def test_config_hash_tamper_and_unrecorded_enabled_mode_fail_closed(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);p=f.profile['fc01_system']['ros__parameters'];p.update(visual_navigation_enabled=True,px4_horizontal_enabled=True)
            with self.assertRaises(FileNotFoundError):modes.configured_mode(f.entry,p)
            modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
            (f.dep/'vision_camera.yaml').write_text('# modified\n')
            with self.assertRaisesRegex(ValueError,'摘要'):modes.configured_mode(f.entry,p)

    def test_migration_copies_verified_site_without_modifying_old_or_activating(self):
        with tempfile.TemporaryDirectory() as a,tempfile.TemporaryDirectory() as b:
            old,new=Setup(a),Setup(b)
            modes.configure(old.entry,'track',old.site,run=old.run,check_stopped=old.stopped)
            original={p.name:p.read_bytes() for p in old.dep.iterdir() if p.is_file()}
            shutil.copy2(old.dep/'runtime.yaml',new.dep/'runtime.yaml')
            result=modes.migrate(new.entry,old.entry)
            self.assertFalse(result['activated'])
            _,params=modes.load_parameters(new.dep/'runtime.yaml','fc01_system')
            self.assertTrue(modes.configured_mode(new.entry,params)['enabled'])
            self.assertEqual(original,{p.name:p.read_bytes() for p in old.dep.iterdir() if p.is_file()})

    def test_site_cannot_borrow_candidate_intrinsics_or_override_task_safety(self):
        for section,key in [('camera','camera_fx'),('camera','target_diameter_m'),('mission','standoff_m')]:
            with self.subTest(key=key),tempfile.TemporaryDirectory() as d:
                f=Setup(d);v=site();del v[section][key];f.site.write_text(json.dumps(v))
                with self.assertRaises(ValueError):modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);v=site();v['mission']['sensor_timeout_s']=100;f.site.write_text(json.dumps(v))
            with self.assertRaisesRegex(ValueError,'时效'):modes.configure(f.entry,'track',f.site,run=f.run,check_stopped=f.stopped)


    def test_deployment_lock_prevents_concurrent_config_and_service(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d)
            with modes.deployment_lock(f.entry):
                with self.assertRaisesRegex(ValueError,'正在使用'):
                    modes.configure(f.entry,'altitude',run=f.run,check_stopped=f.stopped)

    def test_default_altitude_aux_does_not_launch_or_import_ros(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);popen=Mock();aux=service.VisionAuxiliary(f.entry,dict(mode='altitude',enabled=False),popen=popen)
            aux.start();aux.close();popen.assert_not_called()
            value=json.loads((f.dep/'vision-runtime.json').read_text());self.assertEqual(value['mode'],'altitude');self.assertIsNone(value['auxiliary_pid'])
            self.assertEqual(value['supervisor_pid'],os.getpid())

    def test_pending_aux_launch_cannot_override_disabled_mission_control(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);modes.configure(f.entry,'track',pending_calibration=True,
                                     run=f.run,check_stopped=f.stopped)
            _,params=modes.load_parameters(f.dep/'runtime.yaml','fc01_system')
            proc=Mock(pid=424242,returncode=None);proc.poll.return_value=None
            aux=service.VisionAuxiliary(f.entry,modes.configured_mode(f.entry,params),
                                        popen=Mock(return_value=proc))
            with patch.object(service.threading,'Thread') as thread,patch.object(service.os,'killpg',side_effect=[None,ProcessLookupError()]):
                thread.return_value.is_alive.return_value=False
                aux.start();aux.close()
            self.assertIn('control_enabled:=false',aux.popen.call_args.args[0])
            value=json.loads((f.dep/'vision-runtime.json').read_text())
            self.assertEqual(value['calibration_state'],'pending');self.assertIs(value['mission_control_enabled'],False)

    def test_launch_only_contains_aux_and_cleanup_signals_process_group(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);proc=Mock(pid=424242,returncode=None);proc.poll.return_value=None
            aux=service.VisionAuxiliary(f.entry,dict(mode='track',enabled=True,mission_path='/site/mission.yaml',camera_path='/site/camera.yaml'),popen=Mock(return_value=proc))
            with patch.object(service.threading,'Thread') as thread,patch.object(service.os,'killpg',side_effect=[None,ProcessLookupError()]):
                thread.return_value.is_alive.return_value=False
                aux.start();aux.close()
            argv=aux.popen.call_args.args[0]
            self.assertIn('horizontal_first_test.launch.py',argv);self.assertNotIn('fc01_system_node',' '.join(argv))
            self.assertIn('control_enabled:=true',argv);proc.wait.assert_called_once_with(timeout=12)

    def test_stuck_observer_still_cleans_auxiliary_group_and_log(self):
        with tempfile.TemporaryDirectory() as d:
            f=Setup(d);proc=Mock(pid=424242,returncode=None);proc.poll.return_value=None
            aux=service.VisionAuxiliary(f.entry,dict(mode='track',enabled=True))
            aux.process=proc;aux.thread=Mock();aux.thread.is_alive.return_value=True
            aux.log=Mock()
            with patch.object(service.os,'killpg',side_effect=[None,ProcessLookupError()]) as kill:
                with self.assertRaisesRegex(service.ControlError,'线程未退出'):aux.close()
            kill.assert_any_call(proc.pid,service.signal.SIGINT)
            aux.log.close.assert_called_once();proc.wait.assert_called_once_with(timeout=12)


if __name__=='__main__':unittest.main()

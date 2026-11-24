"""预编译安装事务离线回归；fake systemd/proc绝不操作宿主服务或设备。"""
import copy
import ast
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml
import navigation_prebuilt_install as m


def elf(machine=183):
    data=bytearray(64);data[:6]=b'\x7fELF\x02\x01';data[18:20]=machine.to_bytes(2,'little')
    return bytes(data)+b'FC01_TEST_ONLY_NOT_EXECUTABLE'


class Fixture:
    """Only local software/process facts; aircraft and Agent session queries fail loudly."""
    acceptance_stability_seconds = 0.0
    def __init__(self, temp):
        self.home=Path(temp);self.uid=os.getuid();self.user='orangepi'
        self.units=self.home/'units';self.units.mkdir()
        self.old=self.home/'old';self.old.mkdir()
        self.binary=self.old/'fc01_system_node';self.binary.write_bytes(elf());self.binary.chmod(0o700)
        self.runtime=self.home/'FC01_navigation/runtime.yaml';self.runtime.parent.mkdir()
        self.config={'fc01_system':{'ros__parameters':dict(fc01_runtime_mode='flight-test',
            profile='attitude_altitude',flight_enabled=False,flight_actions_enabled=True,
            takeoff_enabled=True,flight_parameters_confirmed=True,bench_arm_only_enabled=False,
            rc_takeover_confirmed=True,autonomous_failsafe_confirmed=False,
            rc_target_height=1.23,rc_navigation_mode_slot=1,rc_navigation_enabled=True,
            arm_enabled=False,visual_navigation_enabled=False,aa_hover_thrust=.5,
            aa_takeoff_reserve_thrust=.04,aa_thrust_min=.12,aa_thrust_max=.8,aa_thrust_slew=.4,
            agent_device='/dev/ttyS2',agent_baud=921600)}}
        self.runtime.write_text(yaml.safe_dump(self.config))
        self.sdk=self.home/'sdk';self.sdk.mkdir()
        self.env=dict(ROS_DOMAIN_ID='0',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            FC01_PROJECT_ROOT=str(self.sdk),FASTRTPS_DEFAULT_PROFILES_FILE='/unchanged/profile.xml')
        self.agent_binary=self.home/'fc01_agent_build_offline/MicroXRCEAgent'
        self.agent_binary.parent.mkdir();self.agent_binary.write_text('#!/bin/sh\nexit 0\n');self.agent_binary.chmod(0o700)
        self.unit=self.units/m.SERVICE
        self.unit.write_text(m.MARKER+'\n[Service]\nUser=orangepi\nWorkingDirectory='+str(self.old)
            +'\nExecStart=/old/navigation_boot.sh run --mode flight-test\n')
        self.original_unit=self.unit.read_bytes();self.original_runtime=self.runtime.read_bytes()
        self.info=dict(LoadState='loaded',FragmentPath=str(self.unit),DropInPaths='',MainPID='10',
            ActiveState='active',UnitFileState='enabled',User=self.user,WorkingDirectory=str(self.old),
            Environment=' '.join(k+'='+v for k,v in self.env.items()),EnvironmentFiles='',
            ExecStart='/old/navigation_boot.sh')
        self.old_controller=dict(pid=12,uid=self.uid,
            argv=[str(self.binary),'--ros-args','--params-file',str(self.runtime),'-p','socket_path:=/fake/control.sock'],
            exe=str(self.binary),cwd=str(self.old),sha256=m.digest(self.binary))
        self.controller=copy.deepcopy(self.old_controller)
        self.calls=[];self.events=[];self.fail_deps=False;self.fail_new_start=False;self.fail_new_verify=False
        self.stuck=False;self.now=0.;self.stop_delay=0.;self.exit_at=None

    def backend(self):return self
    def check_host(self):self.calls.append('check_host')
    def authenticate(self):self.calls.append('authenticate')
    def service(self):return dict(self.info)
    def owned_processes(self,info):return []
    def same_process_alive(self,row):return False
    def software_snapshot(self):
        # 测试替身只生成本机原生IPC事实；不提供飞机新鲜度或READY。
        pid=self.controller['pid'] if self.controller else None
        return dict(supervisor_pid=int(self.info.get('MainPID') or 0),boot_id=self.boot_id(),
                    controller_pid=pid,state_controller_pid=pid,status='WAITING',state_age_seconds=0.,
                    state=dict(controller_pid=pid,diagnostic_monotonic_s=m.time.monotonic()))
    def controllers(self):return [copy.deepcopy(self.controller)] if self.controller else []
    def descendant(self,pid,main):return (pid,main) in ((12,10),(22,20))
    def environment(self,pid):
        if not int(self.info.get('MainPID') or 0):raise AssertionError('read environment of stopped service')
        return dict(self.env)
    def retained_agent(self,*args,**kwargs):raise AssertionError('maintenance must not query Agent session')
    def live_state(self,*args,**kwargs):raise AssertionError('maintenance must not query aircraft state')
    def boot_id(self):return 'test-boot'
    def dependencies(self,entry,manifest,env):
        self.calls.append('dependencies')
        if self.fail_deps:raise m.InstallError('missing library')
    def config_check(self,*args):self.calls.append('config_check')
    def systemd_verify(self,*args):self.calls.append('systemd_verify')
    def write_root(self,path,data,mode=0o644):
        self.calls.append('write_root');self.events.append(('write',str(path)))
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data);path.chmod(mode)
    def remove_root(self,path):
        self.calls.append('remove_root');Path(path).unlink(missing_ok=True)
    def reload(self):
        self.calls.append('reload')
        own=self.units/(m.SERVICE+'.d')/m.DROPIN
        if not self.unit.exists():
            self.info.update(LoadState='not-found',FragmentPath='',WorkingDirectory='',ExecStart='',
                             DropInPaths='',User='',MainPID='0',ActiveState='inactive')
        elif own.is_file():
            line=next(l for l in own.read_text().splitlines() if l.startswith('ExecStart=/bin/bash '))
            entry=Path(line.split('"')[1]).parent
            self.info.update(LoadState='loaded',FragmentPath=str(self.unit),User=self.user,
                WorkingDirectory=str(entry),ExecStart='/bin/bash '+str(entry/'prebuilt_boot.sh'),DropInPaths=str(own))
        else:self.info.update(WorkingDirectory=str(self.old),ExecStart='/old/navigation_boot.sh',DropInPaths='')
    def become_inactive(self):
        if self.controller:self.events.append(('controller-exited',self.controller['pid']))
        self.controller=None;self.info.update(MainPID='0',ActiveState='inactive');self.exit_at=None
    def missing_unit(self):
        self.become_inactive();self.unit.unlink()
        self.info.update(LoadState='not-found',FragmentPath='',DropInPaths='',User='',
                         WorkingDirectory='',ExecStart='',Environment='',EnvironmentFiles='',UnitFileState='')
    def stop(self):
        self.calls.append('stop');self.events.append(('stop',self.controller['pid'] if self.controller else None))
        if self.stuck:return
        if self.stop_delay:
            self.exit_at=self.now+self.stop_delay;self.info['ActiveState']='deactivating'
        else:self.become_inactive()
    def start(self):
        if self.controller is not None or int(self.info.get('MainPID') or 0):
            raise AssertionError('new start before old controller/service exited')
        self.calls.append('start');entry=Path(self.info['WorkingDirectory'])
        self.events.append(('start','old' if entry==self.old else 'new'))
        if entry==self.old:
            self.controller=copy.deepcopy(self.old_controller);self.info.update(MainPID='10',ActiveState='active')
        else:
            if self.fail_new_start:raise m.InstallError('new start failed')
            manifest=json.loads((entry/m.MANIFEST).read_text());binary=entry/manifest['controller_relative']
            self.runtime.write_bytes((entry/'deployment/runtime.yaml').read_bytes())
            self.controller=dict(pid=22,uid=self.uid,
                argv=[str(binary),'--ros-args','--params-file',str(self.runtime),'-p','socket_path:=/fake/new.sock'],
                exe=str(binary),cwd=str(entry),sha256=m.digest(binary))
            self.info.update(MainPID='20',ActiveState='active')
    def sleep(self,s):
        self.now+=s
        if self.exit_at is not None and self.now>=self.exit_at:self.become_inactive()
        if self.fail_new_verify:raise m.InstallError('new verify failed')
        if self.stuck:raise m.InstallError('old controller still running')

    def archive(self,machine=183,extra=None,navigation=False):
        content={'install/lib/jikl_fc01_system/fc01_system_node':elf(machine),
            'install/lib/jikl_fc01_system/fc01_config_check':elf(machine),
            'prebuilt_boot.sh':b'#!/bin/bash\n', 'navigation_prebuilt_service.py':b'# entry\n'}
        if navigation:
            root=Path(__file__).resolve().parents[1]
            for name in ('navigation.py','navigation_status.py'):
                content[name]=(root/name).read_bytes()
        manifest=dict(version='20261116.stage3.test',source_commit='a'*40,architecture='aarch64',
            ubuntu='22.04',ros_distro='humble',controller_relative='install/lib/jikl_fc01_system/fc01_system_node',
            payload_sha256={name:hashlib.sha256(data).hexdigest() for name,data in content.items()})
        content[m.MANIFEST]=json.dumps(manifest).encode()
        archive=self.home/('runtime'+str(machine)+'.tar.gz')
        with tarfile.open(archive,'w:gz') as tar:
            for name,data in content.items():
                item=tarfile.TarInfo('FC01_OrangePi_20261116_stage3_arm64/'+name);item.size=len(data);item.mode=0o700
                tar.addfile(item,io.BytesIO(data))
            if extra:tar.addfile(extra)
        return archive,m.digest(archive)


class InstallerTest(unittest.TestCase):
    def test_auxiliary_software_failure_during_switch_restores_old_service(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            original=m.verify_active
            def immediate(*args,**kwargs):
                return original(*args,**dict(kwargs,timeout=0.))
            with patch.object(m,'verify_active',side_effect=immediate),\
                 patch.object(m,'verify_vision_software',side_effect=m.InstallError('missing bridge')):
                with self.assertRaisesRegex(m.InstallError,'原版'):
                    m.install(archive,sha,f)
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertEqual(f.info['ActiveState'],'active')
            self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            self.assertEqual([event for event in f.events if event[0]=='start'],
                             [('start','new'),('start','old')])

    def test_explicit_paired_visual_runtime_can_be_inherited_without_aircraft_queries(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp)
            values=f.config['fc01_system']['ros__parameters']
            values.update(visual_navigation_enabled=True,px4_horizontal_enabled=True,
                          rc_target_height=1.5,rc_navigation_mode_slot=1)
            f.runtime.write_text(yaml.safe_dump(f.config))
            # 现场配置搬迁本身由独立工具验证；此事务测试确认视觉开启不再被部署层硬禁。
            archive,sha=f.archive()
            with patch.object(m,'migrate_vision_configuration') as migrate:
                result=m.install(archive,sha,f)
            migrate.assert_called_once()
            actual=yaml.safe_load((Path(result['entry'])/'deployment/runtime.yaml').read_text())
            self.assertTrue(actual['fc01_system']['ros__parameters']['visual_navigation_enabled'])
            self.assertEqual(actual['fc01_system']['ros__parameters']['rc_target_height'],1.5)
            self.assertEqual(f.controller['pid'],22)

    def test_visual_malformed_pair_height_and_slot_fail_before_stop(self):
        changes=({'visual_navigation_enabled':True},
                 {'visual_navigation_enabled':'true'},
                 {'visual_navigation_enabled':False,'px4_horizontal_enabled':True},
                 {'visual_navigation_enabled':True,'px4_horizontal_enabled':True,'rc_target_height':2.},
                 {'visual_navigation_enabled':True,'px4_horizontal_enabled':True,
                  'rc_target_height':1.5,'rc_navigation_mode_slot':4},
                 {'visual_navigation_enabled':True,'px4_horizontal_enabled':True,
                  'rc_target_height':1.5,'rc_navigation_mode_slot':True})
        for changed in changes:
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);f.config['fc01_system']['ros__parameters'].update(changed)
                f.runtime.write_text(yaml.safe_dump(f.config));archive,sha=f.archive()
                with self.assertRaises(m.InstallError):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls);self.assertNotIn('start',f.calls)

    def test_missing_visual_site_configuration_refuses_before_stopping_old_service(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp)
            f.config['fc01_system']['ros__parameters'].update(
                visual_navigation_enabled=True,px4_horizontal_enabled=True,rc_target_height=1.5)
            f.runtime.write_text(yaml.safe_dump(f.config));archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'视觉升级缺少'):
                m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertEqual(f.controller['pid'],12)

    def test_visual_migration_uses_new_verified_tool_and_old_entry_without_live_state(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);entry=f.home/'new';entry.mkdir()
            (entry/'navigation_vision_mode.py').write_text('# tool')
            f.run=unittest.mock.Mock(return_value=subprocess.CompletedProcess([],0,''))
            config=copy.deepcopy(f.config)
            config['fc01_system']['ros__parameters']['visual_navigation_enabled']=True
            m.migrate_vision_configuration(f,entry,config,f.old)
            f.run.assert_called_once_with(['/usr/bin/python3','-B',str(entry/'navigation_vision_mode.py'),
                'migrate','--entry',str(entry),'--source-entry',str(f.old)])
            self.assertNotIn('stop',f.calls)

    def test_absent_stale_and_invalid_aircraft_cache_never_gate_maintenance(self):
        snapshots=(None, {}, {'feedback_current':False,'state':{'armed':True,'nav_state':14}},
                   {'state':{'diagnostic_monotonic_s':-1000,'thrust_body':[0,0,float('nan')],
                             'rc_switches_fresh':False}})
        for cached in snapshots:
            with self.subTest(cached=cached),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);path=f.runtime.parent/'status.json'
                if cached is not None:path.write_text(json.dumps(cached))
                original_cache=path.read_bytes() if path.exists() else None
                before=path.read_bytes() if path.exists() else None
                archive,sha=f.archive();result=m.install(archive,sha,f)
                self.assertTrue(m.install(archive,sha,f)['already_active'])
                m.rollback(result['backup'],f)
                self.assertEqual(f.controller['exe'],str(f.binary))
                if cached is not None:self.assertEqual(path.read_bytes(),before)

    def test_no_running_agent_or_session_query_is_needed_for_every_lifecycle(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            result=m.install(archive,sha,f)
            self.assertTrue(m.install(archive,sha,f)['already_active'])
            f.become_inactive()
            self.assertTrue(m.install(archive,sha,f)['already_active'])
            m.rollback(result['backup'],f)
            self.assertEqual(f.controller['exe'],str(f.binary))

    def test_stopped_old_service_preserves_static_environment_and_rollback_state(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.become_inactive();archive,sha=f.archive()
            result=m.install(archive,sha,f)
            saved=json.loads((Path(result['entry'])/'deployment/environment.json').read_text())
            self.assertEqual(saved['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'],'/unchanged/profile.xml')
            self.assertEqual(saved['environment']['RMW_IMPLEMENTATION'],'rmw_fastrtps_cpp')
            self.assertEqual(saved['agent_path'],str(f.agent_binary))
            m.rollback(result['backup'],f)
            self.assertIsNone(f.controller);self.assertEqual(f.info['ActiveState'],'inactive')
            self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)

    def test_first_service_install_uses_existing_runtime_then_rollback_removes_only_created_unit(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.missing_unit();archive,sha=f.archive()
            result=m.install(archive,sha,f)
            self.assertTrue(f.unit.read_text().startswith(m.MARKER+'\n'))
            self.assertEqual(f.controller['exe'],str(Path(result['entry'])/'install/lib/jikl_fc01_system/fc01_system_node'))
            m.rollback(result['backup'],f)
            self.assertFalse(f.unit.exists());self.assertIsNone(f.controller)
            self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
            self.assertEqual(f.info['ActiveState'],'inactive')

    def test_first_install_rejects_symlinked_dropin_parent_before_writing(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.missing_unit();outside=f.home/'other-directory';outside.mkdir()
            (f.units/(m.SERVICE+'.d')).symlink_to(outside,target_is_directory=True)
            archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'符号链接|链接'):m.install(archive,sha,f)
            self.assertEqual(list(outside.iterdir()),[])
            self.assertNotIn('write_root',f.calls);self.assertNotIn('start',f.calls)

    def test_missing_existing_runtime_never_creates_parameters_or_service(self):
        for first in (False,True):
            with self.subTest(first=first),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);f.become_inactive()
                if first:f.missing_unit()
                f.runtime.unlink();archive,sha=f.archive()
                with self.assertRaisesRegex(m.InstallError,'runtime.yaml'):m.install(archive,sha,f)
                self.assertFalse(f.runtime.exists());self.assertNotIn('start',f.calls)
                self.assertNotIn('write_root',f.calls)

    def test_stopped_candidate_repeat_keeps_original_backup_and_never_backs_up_itself(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            backup=Path(result['backup']);record=(backup/'BACKUP.json').read_bytes()
            f.become_inactive();again=m.install(archive,sha,f)
            self.assertTrue(again['already_active']);self.assertEqual(again['backup'],result['backup'])
            self.assertEqual((backup/'BACKUP.json').read_bytes(),record)
            self.assertEqual(len(list((f.home/'FC01_navigation/prebuilt-backups').iterdir())),1)
            m.rollback(result['backup'],f);self.assertEqual(f.controller['exe'],str(f.binary))

    def test_stopped_candidate_repeat_start_failure_restores_the_same_original_backup(self):
        for original_active in (True,False):
            with self.subTest(original_active=original_active),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp)
                if not original_active:f.become_inactive()
                archive,sha=f.archive();r=m.install(archive,sha,f)
                f.become_inactive();f.fail_new_start=True
                with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
                self.assertEqual(f.info['ActiveState'],'active' if original_active else 'inactive')
                self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
                self.assertEqual(f.unit.read_bytes(),f.original_unit)
                record=json.loads((Path(r['backup'])/'BACKUP.json').read_text())
                self.assertEqual(record['phase'],'failed_restored')
                self.assertEqual(len(list((f.home/'FC01_navigation/prebuilt-backups').iterdir())),1)

    def test_failed_new_start_restores_original_inactive_and_first_install_absence(self):
        for first in (False,True):
            with self.subTest(first=first),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);f.become_inactive()
                if first:f.missing_unit()
                f.fail_new_start=True;archive,sha=f.archive()
                with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
                self.assertIsNone(f.controller);self.assertEqual(f.info['ActiveState'],'inactive')
                self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
                self.assertEqual(f.unit.exists(),not first)
                if not first:self.assertEqual(f.unit.read_bytes(),f.original_unit)
                backup=next((f.home/'FC01_navigation/prebuilt-backups').iterdir())
                self.assertEqual(json.loads((backup/'BACKUP.json').read_text())['phase'],'failed_restored')

    def test_delayed_graceful_exit_is_observed_before_new_start_without_force(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.stop_delay=61.;archive,sha=f.archive()
            with patch.object(m.time,'monotonic',side_effect=lambda:f.now):m.install(archive,sha,f)
            self.assertGreaterEqual(f.now,61.)
            exit_index=f.events.index(('controller-exited',12))
            start_index=f.events.index(('start','new'))
            self.assertLess(exit_index,start_index)
            self.assertTrue(all(event[0] not in ('kill','terminate') for event in f.events))

    def test_software_stop_deadline_keeps_old_controller_and_unit_without_starting_candidate(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.stop_delay=121.;archive,sha=f.archive()
            with patch.object(m.time,'monotonic',side_effect=lambda:f.now):
                with self.assertRaisesRegex(m.InstallError,'原服务配置未修改.*未强杀') as caught:
                    m.install(archive,sha,f)
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            self.assertNotIn('start',f.calls);self.assertNotIn('write_root',f.calls)
            self.assertNotIn('已恢复',str(caught.exception))
            self.assertEqual(f.info['ActiveState'],'deactivating')
            backup=next((f.home/'FC01_navigation/prebuilt-backups').iterdir())
            self.assertEqual(json.loads((backup/'BACKUP.json').read_text())['phase'],'stop_incomplete')

    def test_operator_parameter_change_during_preflight_is_not_replaced_by_stale_snapshot(self):
        # sudo交互和依赖检查都可能耗时；维护人员在此期间修改参数，不应被旧读取值覆盖。
        for change_at in ('dependencies','authenticate'):
            with self.subTest(change_at=change_at),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                operation=getattr(f,change_at)
                changed=copy.deepcopy(f.config)
                changed['fc01_system']['ros__parameters']['rc_target_height']=1.47
                def change(*args,**kwargs):
                    result=operation(*args,**kwargs)
                    f.runtime.write_text(yaml.safe_dump(changed))
                    return result
                setattr(f,change_at,change)
                with self.assertRaisesRegex(m.InstallError,'参数.*修改|参数.*变化'):
                    m.install(archive,sha,f)
                self.assertEqual(yaml.safe_load(f.runtime.read_text()),changed)
                self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_stopped_environment_file_preserves_dds_and_agent_without_process_environment(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.become_inactive()
            envfile=f.home/'dds.env';envfile.write_text('ROS_DOMAIN_ID=0\n'
                'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"\nFASTRTPS_DEFAULT_PROFILES_FILE="/stored/dds profile.xml"\n')
            f.info.update(Environment='ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0',
                          EnvironmentFiles=str(envfile)+' (ignore_errors=no)')
            archive,sha=f.archive()
            # 起服后/proc环境由生成的部署环境提供；停服期间不准尝试读PID0。
            original_start=f.start
            def start_from_saved_environment():
                entry=Path(f.info['WorkingDirectory'])
                if entry!=f.old:
                    f.env.update(json.loads((entry/'deployment/environment.json').read_text())['environment'])
                return original_start()
            f.start=start_from_saved_environment
            result=m.install(archive,sha,f)
            saved=json.loads((Path(result['entry'])/'deployment/environment.json').read_text())
            self.assertEqual(saved['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'],'/stored/dds profile.xml')
            self.assertEqual(saved['agent_path'],str(f.agent_binary))
            m.rollback(result['backup'],f)
            self.assertEqual(f.info['ActiveState'],'inactive');self.assertIsNone(f.controller)
            self.assertTrue(envfile.is_file())

    def test_rollback_stops_candidate_before_original_starts(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.events.clear();m.rollback(result['backup'],f)
            self.assertLess(f.events.index(('stop',22)),f.events.index(('controller-exited',22)))
            self.assertLess(f.events.index(('controller-exited',22)),f.events.index(('start','old')))

    def test_unmanaged_or_wrong_user_controller_refuses_before_stop(self):
        for change in ('unmanaged','wrong_user'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                if change=='unmanaged':f.descendant=lambda *_:False
                else:f.controller['uid']+=1
                with self.assertRaisesRegex(m.InstallError,'不属于'):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_service_or_controller_change_during_authentication_refuses_before_stop(self):
        for changed in ('pid','unit'):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                def changed_during_authentication():
                    if changed=='pid':f.controller['pid']=99
                    else:f.unit.write_bytes(f.original_unit+b'# external update\n')
                f.authenticate=changed_during_authentication
                with self.assertRaises(m.InstallError):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_unknown_service_unit_or_runtime_symlink_refuses_before_stop(self):
        for changed in ('unit','runtime'):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                if changed=='unit':f.unit.write_text('[Service]\nExecStart=/unknown\n')
                else:
                    real=f.runtime.with_suffix('.real');f.runtime.rename(real);f.runtime.symlink_to(real)
                with self.assertRaises(m.InstallError):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_malformed_or_missing_readiness_is_separate_from_install_success(self):
        for text in (None,'not json','[]','null',json.dumps({'state':['invalid']}),
                     json.dumps({'state':'invalid'}),
                     json.dumps({'feedback_current':False,'state':{}})):
            with self.subTest(text=text),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
                if text is not None:(f.runtime.parent/'status.json').write_text(text)
                readiness=m.flight_readiness(f)
                self.assertFalse(readiness['ready_for_manual_rc'])
                self.assertTrue(readiness['missing'])
                self.assertEqual(f.controller['pid'],result['controller_pid'])

    def test_software_acceptance_refuses_wrong_process_hash_runtime_or_dds(self):
        for changed in ('exe','sha256','cwd','runtime','DDS'):
            with self.subTest(changed=changed),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive();r=m.install(archive,sha,f)
                entry=Path(r['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
                env=json.loads((entry/'deployment/environment.json').read_text())
                if changed in ('exe','sha256','cwd'):f.controller[changed]='/incorrect/value'
                elif changed=='runtime':
                    bad=yaml.safe_load(f.runtime.read_text())
                    bad['fc01_system']['ros__parameters']['rc_target_height']=2.
                    f.runtime.write_text(yaml.safe_dump(bad))
                else:f.env['RMW_IMPLEMENTATION']='different'
                with self.assertRaisesRegex(m.InstallError,'启动验收失败'):
                    m.verify_active(f,entry,manifest,env,timeout=0)

    def test_environment_file_and_saved_deployment_survive_stopped_install(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.become_inactive()
            deployment=f.old/'deployment';deployment.mkdir()
            saved={'environment':{'ROS_DOMAIN_ID':'0','ROS_LOCALHOST_ONLY':'0',
                    'CYCLONEDDS_URI':'file:///original/dds.xml'},'agent_path':str(f.agent_binary)}
            (deployment/'environment.json').write_text(json.dumps(saved))
            envfile=f.home/'dds.env';envfile.write_text('RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n')
            f.info.update(EnvironmentFiles=str(envfile)+' (ignore_errors=no)')
            f.env['CYCLONEDDS_URI']='file:///original/dds.xml'
            archive,sha=f.archive();r=m.install(archive,sha,f)
            actual=json.loads((Path(r['entry'])/'deployment/environment.json').read_text())
            self.assertEqual(actual['environment']['CYCLONEDDS_URI'],'file:///original/dds.xml')
            self.assertEqual(actual['environment']['RMW_IMPLEMENTATION'],'rmw_fastrtps_cpp')
            self.assertEqual(actual['agent_path'],str(f.agent_binary))

    def test_status_cli_returns_software_success_while_flight_feedback_absent(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            output=io.StringIO()
            with patch.object(m,'Backend',return_value=f),patch('sys.stdout',output):
                self.assertEqual(m.main(['--status']),0)
            data=json.loads(output.getvalue())
            self.assertEqual(data['service']['ActiveState'],'active')
            self.assertEqual(len(data['controllers']),1)
            self.assertEqual(data['controllers'][0]['pid'],result['controller_pid'])
            self.assertFalse(data['flight_readiness']['ready_for_manual_rc'])
            self.assertTrue(data['flight_readiness']['missing'])

    def test_install_cli_reports_success_with_malformed_optional_aircraft_json(self):
        for cache in ('[]','null','{"state": "not-a-mapping"}'):
            with self.subTest(cache=cache),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                (f.runtime.parent/'status.json').write_text(cache)
                output=io.StringIO()
                with patch.object(m,'Backend',return_value=f),patch('sys.stdout',output):
                    self.assertEqual(m.main([str(archive),'--expected-sha256',sha]),0)
                self.assertIn('FC01_PREBUILT_INSTALL_OK',output.getvalue())
                self.assertIn('FC01_FLIGHT_READINESS=',output.getvalue())
                self.assertEqual(f.info['ActiveState'],'active');self.assertIsNotNone(f.controller)

    def test_readiness_identity_and_freshness_are_display_only(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();m.install(archive,sha,f)
            state={name:True for name in ('rc_link_ready','rc_source_valid','rc_switches_fresh',
                'attitude_fresh','attitude_sample_fresh','local_fresh','local_sample_fresh','z_valid','vz_valid',
                'battery_fresh','battery_connected','status_fresh','land_fresh','arming_state_known','preflight',
                'flight_actions_enabled','takeoff_enabled','rc_navigation_enabled')}
            timing=dict(timeout_s=.5,source_reason='VALID',receipt_reason='VALID',
                        envelope_age_s=.01,receipt_age_s=.01,sample_age_s=.01,sample_progress_age_s=.01)
            state.update(diagnostic_monotonic_s=99.98,source_timing={name:dict(timing) for name in
                         ('status','land','local','attitude','flags','battery')})
            row=dict(supervisor_pid=20,controller_pid=22,state_controller_pid=22,
                     boot_id='test-boot',feedback_current=True,state=state,status='READY_WAIT_ARM')
            path=f.runtime.parent/'status.json';path.write_text(json.dumps(row))
            with patch.object(m.time,'monotonic',return_value=100.):
                self.assertTrue(m.flight_readiness(f)['ready_for_manual_rc'])
            for key,value in (('supervisor_pid',999),('controller_pid',999),('state_controller_pid',999),
                              ('boot_id','old-boot'),('feedback_current',False)):
                altered=copy.deepcopy(row);altered[key]=value;path.write_text(json.dumps(altered))
                with patch.object(m.time,'monotonic',return_value=100.):
                    self.assertFalse(m.flight_readiness(f)['ready_for_manual_rc'])
            path.write_text(json.dumps(row))
            for now in (98.,100.48,103.):
                with patch.object(m.time,'monotonic',return_value=now):
                    self.assertFalse(m.flight_readiness(f)['ready_for_manual_rc'])
            # 读取旧状态不会重新续鲜，也不会对软件重复安装增设飞行反馈条件。
            self.assertEqual(json.loads(path.read_text()),row)
            self.assertTrue(m.install(archive,sha,f)['already_active'])

    def test_readiness_closed_gates_and_expired_sources_do_not_block_software_repeat(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();m.install(archive,sha,f)
            state={name:True for name in ('rc_link_ready','rc_source_valid','rc_switches_fresh',
                'attitude_fresh','attitude_sample_fresh','local_fresh','local_sample_fresh','z_valid','vz_valid',
                'battery_fresh','battery_connected','status_fresh','land_fresh','arming_state_known','preflight',
                'flight_actions_enabled','takeoff_enabled','rc_navigation_enabled')}
            timing=dict(timeout_s=.5,source_reason='VALID',receipt_reason='VALID',
                        envelope_age_s=.01,receipt_age_s=.01,sample_age_s=.01,sample_progress_age_s=.01)
            state.update(diagnostic_monotonic_s=100.,source_timing={name:dict(timing) for name in
                         ('status','land','local','attitude','flags','battery')})
            row=dict(supervisor_pid=20,controller_pid=22,state_controller_pid=22,
                     boot_id='test-boot',feedback_current=True,state=state,status='READY_WAIT_ARM')
            path=f.runtime.parent/'status.json'
            for gate in ('flight_actions_enabled','takeoff_enabled','rc_navigation_enabled'):
                changed=copy.deepcopy(row);changed['state'][gate]=False;path.write_text(json.dumps(changed))
                with patch.object(m.time,'monotonic',return_value=100.1):
                    self.assertFalse(m.flight_readiness(f)['ready_for_manual_rc'])
                self.assertTrue(m.install(archive,sha,f)['already_active'])
            path.write_text(json.dumps(row))
            for topic in ('status','land','local','attitude','flags','battery'):
                changed=copy.deepcopy(row);changed['state']['source_timing'][topic]['timeout_s']=.05
                path.write_text(json.dumps(changed))
                with patch.object(m.time,'monotonic',return_value=100.1):
                    self.assertFalse(m.flight_readiness(f)['ready_for_manual_rc'])
                self.assertTrue(m.install(archive,sha,f)['already_active'])

    def test_optional_readiness_subprocess_failure_never_changes_success(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            path=f.runtime.parent/'status.json';path.write_text(json.dumps({'state':{}}))
            with patch.object(f,'service',side_effect=subprocess.CalledProcessError(1,['systemctl'])):
                readiness=m.flight_readiness(f)
            self.assertFalse(readiness['ready_for_manual_rc'])
            self.assertEqual(f.controller['pid'],result['controller_pid'])

    def test_direct_navigation_queries_keep_exact_tree_and_repeat_install(self):
        # 真实新解释器没有 -B/环境开关：重现用户从版本目录直接执行管理入口。
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive(navigation=True)
            result=m.install(archive,sha,f);entry=Path(result['entry'])
            manifest=json.loads((entry/m.MANIFEST).read_text())
            env=dict(os.environ)
            for key in ('PYTHONDONTWRITEBYTECODE','PYTHONPYCACHEPREFIX','PYTHONPATH'):
                env.pop(key,None)
            help_result=subprocess.run([sys.executable,str(entry/'navigation.py'),'--help'],
                env=env,capture_output=True,text=True)
            self.assertEqual(help_result.returncode,0,help_result.stderr)
            # 只替换只读状态来源，不查询宿主systemd/用户快照；项目模块仍真实导入。
            query='''import sys,runpy
sys.path.insert(0,sys.argv[1])
namespace=runpy.run_path(sys.argv[1]+'/navigation.py')
main=namespace['main']
main.__globals__['service_info']=lambda:dict(ActiveState='inactive')
main.__globals__['snapshot']=lambda home:{}
raise SystemExit(main(['status']))
'''
            status_result=subprocess.run([sys.executable,'-c',query,str(entry)],
                env=env,capture_output=True,text=True)
            self.assertEqual(status_result.returncode,0,status_result.stderr)
            self.assertIn('STOPPED',status_result.stdout)
            self.assertEqual(list(entry.rglob('*.pyc')),[])
            self.assertEqual(list(entry.rglob('__pycache__')),[])
            m.verify_tree(entry,manifest)
            again=m.install(archive,sha,f)
            self.assertTrue(again['already_active'])
            self.assertEqual(again['backup'],result['backup'])
            self.assertEqual(f.calls.count('stop'),1)

    def test_unlisted_bytecode_still_rejected_before_stopping_service(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive(navigation=True)
            result=m.install(archive,sha,f);entry=Path(result['entry'])
            cache=entry/'__pycache__/navigation_status.cpython-310.pyc'
            cache.parent.mkdir();cache.write_bytes(b'unlisted bytecode')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'多出\\(1\\).*navigation_status'):
                m.install(archive,sha,f)
            self.assertEqual(f.calls.count('stop'),before)
            self.assertEqual(cache.read_bytes(),b'unlisted bytecode')

    def test_install_repeat_rollback_and_effective_inheritance(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            result=m.install(archive,sha,f)
            self.assertFalse(result['already_active']);self.assertEqual(f.calls.count('stop'),1)
            entry=Path(result['entry']);config=yaml.safe_load((entry/'deployment/runtime.yaml').read_text())
            new=config['fc01_system']['ros__parameters'];old=f.config['fc01_system']['ros__parameters']
            for key,value in old.items():self.assertEqual(new[key],value)
            self.assertEqual(new['fc01_config_path'],str(f.home/'FC01_navigation/runtime.yaml'))
            inherited=json.loads((entry/'deployment/environment.json').read_text())
            self.assertEqual(inherited['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'],'/unchanged/profile.xml')
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            dropin=(f.units/(m.SERVICE+'.d')/m.DROPIN).read_text()
            self.assertIn('\nWorkingDirectory='+str(entry)+'\n',dropin)
            self.assertNotIn('WorkingDirectory="',dropin)
            again=m.install(archive,sha,f)
            self.assertTrue(again['already_active']);self.assertEqual(f.calls.count('stop'),1)
            self.assertEqual(again['backup'],result['backup'])
            m.rollback(result['backup'],f)
            self.assertEqual(f.unit.read_bytes(),f.original_unit);self.assertEqual(f.runtime.read_bytes(),f.original_runtime)
            self.assertEqual(f.controller['exe'],str(f.binary));self.assertEqual(f.info['ActiveState'],'active')
            self.assertFalse((f.units/(m.SERVICE+'.d')/m.DROPIN).exists())

    @unittest.skipUnless(shutil.which('systemd-analyze'),'requires real systemd parser')
    def test_real_systemd_parser_accepts_scalar_working_directory(self):
        # 使用宿主真实解析器，不启动unit、不调用systemctl、不要求root。
        with tempfile.TemporaryDirectory(prefix='fc01 unit % ') as temp:
            root=Path(temp);boot=root/'prebuilt_boot.sh';boot.write_text('#!/bin/bash\n')
            unit=root/'fc01-scalar-test.service'
            unit.write_text('[Unit]\nDescription=FC01 parser-only fixture\n[Service]\nType=simple\n'
                +'ExecStart=/bin/bash '+m.systemd_quote(boot)+'\n'
                +'WorkingDirectory='+m.systemd_directory(root)+'\n')
            result=subprocess.run(['systemd-analyze','verify',str(unit)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertNotIn('WorkingDirectory="',unit.read_text())
            self.assertIn('%%',unit.read_text())

    def test_scalar_working_directory_rejects_ambiguous_values(self):
        for value in ('relative','/tmp/new\n[Service]','/tmp/new\\path','/tmp/new\tpath','/tmp/new '):
            with self.subTest(value=value),self.assertRaises(m.InstallError):m.systemd_directory(value)

    def test_bad_hash_and_architecture_rejected_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'SHA-256'):m.install(archive,'0'*64,f)
            archive,sha=f.archive(machine=62)
            with self.assertRaisesRegex(m.InstallError,'AArch64'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertEqual(f.runtime.read_bytes(),f.original_runtime)

    def test_missing_dependency_rejected_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.fail_deps=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'missing library'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_missing_local_agent_executable_is_software_failure_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.agent_binary.unlink();archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'Agent可执行'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_native_configuration_or_unit_parser_failure_is_before_stop(self):
        for step in ('config_check','systemd_verify'):
            with self.subTest(step=step),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);archive,sha=f.archive()
                def refuse(*_):raise m.InstallError('invalid local software configuration')
                setattr(f,step,refuse)
                with self.assertRaisesRegex(m.InstallError,'local software'):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls);self.assertNotIn('write_root',f.calls)

    def test_unsupported_domain_preserved_and_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.env['ROS_DOMAIN_ID']='9';archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'非域0'):m.install(archive,sha,f)
            self.assertEqual(f.env['ROS_DOMAIN_ID'],'9');self.assertNotIn('stop',f.calls)

    def test_archive_traversal_and_symlink_rejected(self):
        for name,typecode in [('FC01_OrangePi_20261116_stage3_arm64/../../outside',tarfile.REGTYPE),('FC01_OrangePi_20261116_stage3_arm64/link',tarfile.SYMTYPE)]:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as temp:
                f=Fixture(temp);item=tarfile.TarInfo(name);item.type=typecode;item.linkname='/etc/passwd'
                archive,sha=f.archive(extra=item)
                with self.assertRaisesRegex(m.InstallError,'不安全'):m.install(archive,sha,f)
                self.assertNotIn('stop',f.calls)

    def test_failed_new_start_restores_old_active(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.fail_new_start=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):m.install(archive,sha,f)
            self.assertEqual(f.controller['exe'],str(f.binary));self.assertEqual(f.info['ActiveState'],'active')
            self.assertEqual(f.runtime.read_bytes(),f.original_runtime)

    def test_original_dropin_and_environment_file_backed_up_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);drop=f.units/(m.SERVICE+'.d')/'10-original.conf';drop.parent.mkdir();drop.write_text('[Service]\nEnvironment=ROS_DOMAIN_ID=0\n')
            envfile=f.home/'dds.env';envfile.write_text('RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n')
            f.info.update(DropInPaths=str(drop),EnvironmentFiles=str(envfile)+' (ignore_errors=no)')
            archive,sha=f.archive();r=m.install(archive,sha,f)
            backup=Path(r['backup']);saved=json.loads((backup/'BACKUP.json').read_text())
            self.assertIn(str(drop),[x['path'] for x in saved['files']]);self.assertIn(str(envfile),[x['path'] for x in saved['files']])
            self.assertEqual(drop.read_text(),'[Service]\nEnvironment=ROS_DOMAIN_ID=0\n')

    def test_corrupted_existing_payload_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            binary=Path(result['entry'])/'install/lib/jikl_fc01_system/fc01_system_node'
            binary.write_bytes(elf()+b'edited')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'摘要'):m.install(archive,sha,f)
            self.assertEqual(f.calls.count('stop'),before)
            self.assertTrue(binary.read_bytes().endswith(b'edited'))

    def test_busy_old_controller_never_changes_service(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.stuck=True;archive,sha=f.archive()
            with self.assertRaisesRegex(m.InstallError,'原服务配置未修改'):m.install(archive,sha,f)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertNotIn('write_root',f.calls)

    def test_duplicate_controller_rejects_without_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            f.controllers=lambda:[copy.deepcopy(f.controller),copy.deepcopy(f.controller)]
            with self.assertRaisesRegex(m.InstallError,'多个控制器'):m.install(archive,sha,f)
            self.assertNotIn('stop',f.calls)

    def test_real_size_manifest_never_enters_dependency_process_argv(self):
        # 真实发行约5000成员；单个Linux argv不能超过MAX_ARG_STRLEN约128KiB。
        # 检查真正Backend生成的命令及内层Python，而不是Fixture的空dependencies替身。
        manifest={'payload_sha256':{
            'install/lib/project/'+('long_source_name_'*4)+str(i):'a'*64 for i in range(5000)}}
        self.assertGreater(len(json.dumps(manifest)),131072)
        commands=[]
        backend=object.__new__(m.Backend)
        def capture(argv):
            commands.append(argv);return types.SimpleNamespace(stdout='dependencies checked')
        backend.run=capture
        result=backend.dependencies(Path('/independent/release'),manifest,{})
        self.assertEqual(result,'dependencies checked')
        command=commands[0]
        self.assertTrue(all(len(arg.encode())<131072 for arg in command))
        self.assertEqual(command[-1],'/independent/release')
        self.assertEqual(len(command),5)
        script=command[2]
        self.assertIn('exec /usr/bin/python3 - "$1" <<',script)
        nested=script.split("<<'CHECK'\n",1)[1].rsplit('\nCHECK',1)[0]
        tree=ast.parse(nested)
        indices=[node.slice.value for node in ast.walk(tree)
                 if isinstance(node,ast.Subscript) and isinstance(node.value,ast.Attribute)
                 and isinstance(node.value.value,ast.Name) and node.value.value.id=='sys'
                 and node.value.attr=='argv' and isinstance(node.slice,ast.Constant)]
        self.assertEqual(indices,[1])

    def test_startup_acceptance_waits_for_measured_slow_preflight_not_stale_feedback(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            entry=Path(result['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
            environment=json.loads((entry/'deployment/environment.json').read_text())
            controller=f.controllers;clock=[0.]
            # 完整ARM64视觉运行库可能超过旧180秒预算；无需飞机反馈也应等软件预检。
            f.controllers=lambda:[] if clock[0]<240 else controller()
            f.sleep=lambda seconds:clock.__setitem__(0,clock[0]+10)
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
                actual=m.verify_active(f,entry,manifest,environment)
            self.assertGreaterEqual(clock[0],240)
            self.assertEqual(actual['controller']['exe'],str(entry/manifest['controller_relative']))
            self.assertEqual(m.STARTUP_ACCEPTANCE_SECONDS,600)
            # 同一验收等待仍有界；永远没控制器必须失败，不能变成无限重试。
            f.controllers=lambda:[];clock[0]=0
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
                with self.assertRaisesRegex(m.InstallError,'启动验收失败'):
                    m.verify_active(f,entry,manifest,environment)
            self.assertEqual(clock[0],600)

    def test_install_groups_full_ldd_in_required_config_check_without_shell_bypass(self):
        backend=object.__new__(m.Backend);commands=[]
        backend.run=lambda argv:commands.append(argv) or types.SimpleNamespace(stdout='checked')
        with patch.dict(os.environ,{'FC01_INSTALL_LOCAL_CHECK':'elf-and-types'}):
            backend.dependencies(Path('/unused'),{}, {})
        self.assertTrue(commands[-1][2].startswith('unset FC01_INSTALL_LOCAL_CHECK'))
        backend._installation_checks_follow=True
        backend.dependencies(Path('/unused'),{}, {})
        self.assertTrue(commands[-1][2].startswith('export FC01_INSTALL_LOCAL_CHECK=elf-and-types'))
        self.assertIn('if not ldd_required:continue',commands[-1][2])

    def test_dependency_subprocess_checks_each_hash_and_arch_before_reusing_ldd(self):
        # 执行Backend真正送入ARM Python的脚本；仅替代ROS导入和ldd，
        # 文件遍历、SHA及ELF判断仍使用真实临时文件。
        backend=object.__new__(m.Backend)
        commands=[]
        backend.run=lambda argv:commands.append(argv) or types.SimpleNamespace(stdout='')
        backend.dependencies(Path('/unused'),{}, {})
        nested=commands[0][2].split("<<'CHECK'\n",1)[1].rsplit('\nCHECK',1)[0]
        modules={}
        names={
            'rclpy.type_support':['check_for_type_support'],
            'px4_msgs.msg':['VehicleStatus','VehicleLocalPosition','VehicleAttitude','ManualControlSwitches'],
            'jikl_fc01_system.msg':['VisionState','VisionHealth','NavigationCommand','VelocityCommand','AttitudeThrustPreview'],
        }
        for name,attributes in names.items():
            module=types.ModuleType(name)
            for attribute in attributes:setattr(module,attribute,lambda *_:None)
            modules[name]=module
        with tempfile.TemporaryDirectory() as temp:
            entry=Path(temp);first=entry/'lib/a.so';alias=entry/'lib/a.so.1'
            other=entry/'other/a.so';first.parent.mkdir();other.parent.mkdir()
            for path in (first,alias,other):path.write_bytes(elf())
            def manifest():
                (entry/m.MANIFEST).write_text(json.dumps({'payload_sha256':{
                    path.relative_to(entry).as_posix():m.digest(path) for path in (first,alias,other)}}))
            def execute(run):
                with (patch.dict(sys.modules,modules),patch.object(sys,'argv',['dependency-check',str(entry)]),
                      patch.object(subprocess,'run',side_effect=run),patch('sys.stdout',new_callable=io.StringIO)):
                    exec(compile(nested,'<real-dependency-check>','exec'),{})
            def success(argv,**kwargs):return subprocess.CompletedProcess(argv,0,'resolved','')
            manifest();calls=[]
            execute(lambda argv,**kwargs:calls.append(argv) or success(argv,**kwargs))
            self.assertEqual(len(calls),2)
            self.assertEqual({Path(call[1]).parent for call in calls},{first.parent,other.parent})
            # 清单不符与非ARM别名均须拒绝，不能借同字节别名缓存跳过验证。
            alias.write_bytes(elf()+b'changed')
            with self.assertRaisesRegex(SystemExit,'ELF hash mismatch'):execute(success)
            alias.write_bytes(elf(machine=62));manifest()
            with self.assertRaisesRegex(SystemExit,'non-AArch64'):execute(success)
            alias.write_bytes(elf());manifest();failed=[]
            def failure(argv,**kwargs):
                failed.append(argv);return subprocess.CompletedProcess(argv,1,'','failed dependency')
            with self.assertRaisesRegex(SystemExit,'failed dependency'):execute(failure)
            self.assertEqual(len(failed),1)
            # 失败不跨检查缓存；修好依赖后两种$ORIGIN仍应分别真实检查。
            calls=[];execute(lambda argv,**kwargs:calls.append(argv) or success(argv,**kwargs))
            self.assertEqual(len(calls),2)

    def test_startup_identity_must_stay_present_for_a_full_stability_window(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            entry=Path(result['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
            environment=json.loads((entry/'deployment/environment.json').read_text())
            self.assertEqual(m.Backend.acceptance_stability_seconds,1.0)
            f.acceptance_stability_seconds=1.;clock=[0.];controllers=f.controllers;observed=[]
            def intermittent():
                present=not .39<=clock[0]<.79
                observed.append((clock[0],present))
                return controllers() if present else []
            f.controllers=intermittent
            f.sleep=lambda seconds:clock.__setitem__(0,round(clock[0]+seconds,6))
            with patch.object(m.time,'monotonic',side_effect=lambda:clock[0]):
                m.verify_active(f,entry,manifest,environment,timeout=5)
            self.assertTrue(any(not present for _,present in observed))
            self.assertGreaterEqual(clock[0],1.8)
            self.assertLess(clock[0],2.)

    def test_repeat_install_without_legacy_sdk_environment(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.env.pop('FC01_PROJECT_ROOT')
            self.assertTrue(m.install(archive,sha,f)['already_active'])
            self.assertEqual(f.calls.count('stop'),1)

    def test_inactive_failed_candidate_can_roll_back_without_live_controller(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.controller=None;f.info.update(ActiveState='failed',MainPID='0')
            m.rollback(result['backup'],f)
            self.assertEqual(f.controller['exe'],str(f.binary))

    def test_original_restart_different_parameters_is_not_reported_recovered(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            backup=Path(result['backup']);record=json.loads((backup/'BACKUP.json').read_text())
            f.controller=copy.deepcopy(f.old_controller)
            f.info.update(ActiveState='active',MainPID='10')
            wrong=copy.deepcopy(f.config);wrong['fc01_system']['ros__parameters']['rc_target_height']=2.
            f.runtime.write_text(yaml.safe_dump(wrong))
            with self.assertRaisesRegex(m.InstallError,'参数.*原始备份'):
                m.verify_original(f,backup,record,timeout=0)

    def test_external_unit_edit_blocks_rollback_before_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            f.unit.write_bytes(f.original_unit+b'# operator edit\n')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'外部修改'):m.rollback(result['backup'],f)
            self.assertTrue(f.unit.read_bytes().endswith(b'# operator edit\n'))
            self.assertEqual(f.calls.count('stop'),before)
            self.assertEqual(f.info['ActiveState'],'active')

    def test_rollback_dropin_edit_during_authentication_is_preserved_before_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            own=f.units/(m.SERVICE+'.d')/m.DROPIN;original=own.read_bytes()
            def edit():own.write_bytes(original+b'# operator edit during authentication\n')
            f.authenticate=edit;stops=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'切换记录|drop-in|修改'):
                m.rollback(result['backup'],f)
            self.assertEqual(f.calls.count('stop'),stops)
            self.assertEqual(own.read_bytes(),original+b'# operator edit during authentication\n')
            self.assertEqual(f.controller['pid'],22)

    def test_rollback_dropin_edit_during_stop_is_not_erased_or_old_service_started(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            own=f.units/(m.SERVICE+'.d')/m.DROPIN;original=own.read_bytes();stop=f.stop
            def edit_during_stop():
                stop();own.write_bytes(original+b'# operator edit during stop\n')
            f.stop=edit_during_stop;starts=f.calls.count('start')
            with self.assertRaisesRegex(m.InstallError,'切换记录|drop-in|修改'):
                m.rollback(result['backup'],f)
            self.assertEqual(f.calls.count('start'),starts)
            self.assertEqual(own.read_bytes(),original+b'# operator edit during stop\n')

    def test_invalid_backup_rejected_without_configuration_write(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            (Path(result['backup'])/'runtime.yaml').write_text('corrupt')
            count=f.calls.count('write_root')
            before=f.calls.count('stop')
            with self.assertRaisesRegex(m.InstallError,'备份摘要'):m.rollback(result['backup'],f)
            self.assertEqual(f.calls.count('write_root'),count)
            self.assertEqual(f.calls.count('stop'),before)
            self.assertEqual(f.info['ActiveState'],'active')

    def test_active_supervisor_without_controller_can_be_replaced_in_maintenance(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);f.controller=None
            archive,sha=f.archive();result=m.install(archive,sha,f)
            self.assertEqual(f.controller['pid'],result['controller_pid'])
            self.assertIn(('stop',None),f.events)
            self.assertLess(f.events.index(('stop',None)),f.events.index(('start','new')))
            m.rollback(result['backup'],f)
            self.assertEqual(f.controller['exe'],str(f.binary));self.assertEqual(f.info['ActiveState'],'active')

    def test_failed_supervisor_without_controller_restores_old_active(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();start=f.start
            def start_without_new_controller():
                if Path(f.info['WorkingDirectory'])==f.old:return start()
                f.calls.append('start');f.controller=None
                f.info.update(ActiveState='active',MainPID='20')
            f.start=start_without_new_controller;f.fail_new_verify=True
            with self.assertRaisesRegex(m.InstallError,'原版配置和运行状态已恢复'):
                m.install(archive,sha,f)
            self.assertEqual(f.info['ActiveState'],'active')
            self.assertEqual(f.controller['exe'],str(f.binary))
            self.assertEqual(f.calls.count('stop'),2)

    def test_unmodified_sources_are_never_privileged_rewritten_on_rollback(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            calls=[];write=f.write_root
            def record_write(path,data,mode=0o644):
                calls.append(Path(path));return write(path,data,mode)
            f.write_root=record_write
            m.rollback(result['backup'],f)
            self.assertNotIn(f.unit,calls)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)


if __name__=='__main__':unittest.main()

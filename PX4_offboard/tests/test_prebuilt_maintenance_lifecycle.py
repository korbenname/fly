"""维护切换的软件身份/IPC回归；完全离线，不触碰真实systemd、串口或飞控。"""
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import navigation_prebuilt_install as m
from test_navigation_prebuilt_install import Fixture


class LifecycleTest(unittest.TestCase):
    def test_owned_auxiliary_must_exit_before_start(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            row=dict(pid=91,start_ticks=123,uid=f.uid,argv=['/old/mission'])
            f.owned_processes=lambda info:[row] if f.now<1.2 else []
            f.same_process_alive=lambda item:f.now<1.2
            with patch.object(m.time,'monotonic',side_effect=lambda:f.now):
                result=m.install(archive,sha,f)
            self.assertGreaterEqual(f.now,1.2)
            self.assertEqual(result['controller_pid'],22)
            self.assertLess(f.events.index(('controller-exited',12)),f.events.index(('start','new')))

    def test_residual_owned_auxiliary_keeps_original_unit_and_never_starts_new(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive()
            row=dict(pid=91,start_ticks=123,uid=f.uid,argv=['/old/mission'])
            f.owned_processes=lambda info:[row]
            f.same_process_alive=lambda item:True
            with patch.object(m.time,'monotonic',side_effect=lambda:f.now):
                with self.assertRaisesRegex(m.InstallError,'原服务配置未修改.*未强杀'):
                    m.install(archive,sha,f)
            self.assertNotIn('start',f.calls)
            self.assertEqual(f.unit.read_bytes(),f.original_unit)
            self.assertNotIn('write_root',f.calls)
            backup=next((f.home/'FC01_navigation/prebuilt-backups').iterdir())
            self.assertEqual(json.loads((backup/'BACKUP.json').read_text())['phase'],'stop_incomplete')

    def test_current_ipc_allows_no_aircraft_fields_and_waiting(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp)
            with patch.object(m.time,'monotonic',return_value=50.):
                stamp=m.software_ipc_stamp(f,f.service(),f.controller,since=49.)
            self.assertEqual(stamp,50.)
            self.assertFalse(m.flight_readiness(f)['ready_for_manual_rc'])

    def test_old_or_wrong_identity_ipc_never_satisfies_install(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp)
            with patch.object(m.time,'monotonic',return_value=50.):
                original=f.software_snapshot()
                variants=[]
                for field,value in (('boot_id','old-boot'),('supervisor_pid',9),
                                    ('controller_pid',11),('state_controller_pid',11),
                                    ('ipc_error',{'kind':'clean_eof'}),('status','STOPPED')):
                    row=copy.deepcopy(original);row[field]=value;variants.append(row)
                for field,value in (('controller_pid',11),('diagnostic_monotonic_s',48.),
                                    ('diagnostic_monotonic_s',float('nan'))):
                    row=copy.deepcopy(original);row['state'][field]=value;variants.append(row)
                for row in variants:
                    with self.subTest(row=row):
                        f.software_snapshot=lambda:row
                        with self.assertRaises(m.InstallError):
                            m.software_ipc_stamp(f,f.service(),f.controller,since=49.)

    def test_ipc_has_to_advance_not_only_survive_as_a_file(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            entry=Path(result['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
            env=json.loads((entry/'deployment/environment.json').read_text())
            f.acceptance_stability_seconds=1.
            f.sleep=lambda seconds:setattr(f,'now',f.now+seconds)
            with patch.object(m.time,'monotonic',side_effect=lambda:f.now):
                frozen=f.software_snapshot()
                f.software_snapshot=lambda:frozen
                with self.assertRaisesRegex(m.InstallError,'启动验收失败'):
                    m.verify_active(f,entry,manifest,env,timeout=2.)

    def test_systemd_software_failure_does_not_wait_whole_budget(self):
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp);archive,sha=f.archive();result=m.install(archive,sha,f)
            entry=Path(result['entry']);manifest=json.loads((entry/m.MANIFEST).read_text())
            env=json.loads((entry/'deployment/environment.json').read_text())
            f.become_inactive();f.info.update(ActiveState='failed',Result='exit-code',ExecMainStatus='23')
            f.sleep=lambda seconds:self.fail('failed unit must fail without waiting')
            with self.assertRaisesRegex(m.InstallError,'systemd Result=exit-code.*23'):
                m.verify_active(f,entry,manifest,env)

    def test_systemd_start_is_nonblocking_with_bounded_acceptance(self):
        backend=m.Backend.__new__(m.Backend);commands=[]
        backend.privileged=lambda argv:commands.append(argv)
        backend.start()
        self.assertEqual(commands,[['/usr/bin/systemctl','start','--no-block',m.SERVICE]])
        with tempfile.TemporaryDirectory() as temp:
            f=Fixture(temp)
            unit=m.first_unit(f,f.old).decode()
            self.assertIn('TimeoutStartSec='+str(m.STARTUP_ACCEPTANCE_SECONDS)+'s',unit)
            self.assertIn('TimeoutStopSec=infinity',unit)
            self.assertIn('SendSIGKILL=no',unit)

    def test_proc_identity_and_cgroup_filter_preserve_external_nodes_and_agent(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);proc=root/'proc';proc.mkdir()
            backend=m.Backend.__new__(m.Backend);backend.proc=proc;backend.uid=__import__('os').getuid()
            def add(pid,name,argv,ppid,group='/system.slice/fc01-navigation.service',ticks=123):
                item=proc/str(pid);item.mkdir()
                (item/'cmdline').write_bytes(b'\0'.join(x.encode() for x in argv)+b'\0')
                (item/'status').write_text('PPid:\t'+str(ppid)+'\n')
                # fields after comm: state=3, ppid=4, starttime=22.
                fields=['S',str(ppid)]+['0']*17+[str(ticks)]
                (item/'stat').write_text(str(pid)+' ('+name+') '+' '.join(fields))
                (item/'cgroup').write_text('0::'+group+'\n')
                (item/'exe').symlink_to('/usr/bin/python3')
            add(10,'super visor',['python3','/old/navigation_prebuilt_service.py'],1)
            add(12,'mission',['python3','/old/install/lib/fc01_mission/mission'],10)
            add(13,'bridge',['python3','/old/install/lib/fc01_mission/px4_state_bridge'],1)
            add(14,'agent',['/usr/bin/MicroXRCEAgent','serial'],10)
            add(15,'mission',['python3','/other/mission'],1,'/user.slice/other.service')
            info=dict(MainPID='10',ControlGroup='/system.slice/fc01-navigation.service')
            rows=backend.owned_processes(info)
            self.assertEqual({row['pid'] for row in rows},{10,12,13})
            old=next(row for row in rows if row['pid']==13)
            self.assertTrue(backend.same_process_alive(old))
            (proc/'13/stat').write_text((proc/'13/stat').read_text().replace('123','124'))
            self.assertFalse(backend.same_process_alive(old))


if __name__=='__main__':
    unittest.main()

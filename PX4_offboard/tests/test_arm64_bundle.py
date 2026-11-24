"""发行证据不能凭空/部分绑定真实产物；不执行飞控。"""
import copy
from pathlib import Path
import subprocess
import tempfile
import unittest

from build_arm64_bundle import (ROOT, TARGET_INSTALL_FILES, copy_install, sha,
                                source_names, target_source_names,
                                validate_evidence, write_installer)


class BundleEvidenceTest(unittest.TestCase):
    def fixture(self, root):
        paths = ['native/src/jikl_fc01_system/CMakeLists.txt',
                 'native/src/jikl_fc01_system/src/fc01_system_node.cpp',
                 *['native/tools/' + n for n in ('vision_odometry_adapter.py',
                    'velocity_command_diagnostic.py', 'vision_state_validation.py', 'vision_resource_budget.py')],
                 'install/lib/jikl_fc01_system/fc01_system_node',
                 'install/lib/jikl_fc01_system/fc01_config_check']
        for n in paths:
            p = root / n
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(n)
        return dict(control_candidate_commit='a'*40, architecture='aarch64', ubuntu='22.04', ros_distro='humble',
                    native_source_sha256={n: sha(root/n) for n in paths if n.startswith('native/')},
                    install_sha256={n[8:]: sha(root/n) for n in paths if n.startswith('install/')})

    def target_fixture(self, root):
        """文件级证据夹具不伪装成ARM二进制；架构另由copy/extractor覆盖。"""
        evidence = self.fixture(root)
        sources = {
            'target/src/fc01_vision/CMakeLists.txt': 'project(fc01_vision)',
            'target/src/fc01_vision/config/vision.yaml': 'camera: unconfirmed',
            'target/src/fc01_mission/fc01_mission/mission_node.py': 'CONTROL=False\n',
            'target/src/fc01_mission/config/mission.yaml': 'control_enabled: false',
            'target/src/fc01_mission/config/platform.yaml': 'source: px4',
            'target/src/fc01_bringup/fc01_bringup/__init__.py': '',
            'target/src/fc01_bringup/launch/robot.launch.py': '# original launch',
            'target/src/fc01_bringup/launch/horizontal_first_test.launch.py': '# track only',
        }
        for name, content in sources.items():
            p = root/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content)
        install = root/'install'
        for name in TARGET_INSTALL_FILES:
            p = install/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(name)
        for source, content in sources.items():
            parts = Path(source).parts
            if len(parts) >= 5 and parts[3] == parts[2]:
                name = 'lib/python3.10/site-packages/' + '/'.join(parts[3:])
            elif len(parts) >= 5 and parts[3] in ('config', 'launch'):
                name = '/'.join(('share', parts[2], *parts[3:]))
            else:
                continue
            p = install/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(content)
        p = install/('local/lib/python3.10/dist-packages/jikl_fc01_system/'
                     'jikl_fc01_system_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so')
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('typed ARM64 support fixture')
        evidence['target_source_sha256'] = {name: sha(root/name) for name in target_source_names(root)}
        evidence['install_sha256'] = {p.relative_to(install).as_posix(): sha(p)
                                     for p in install.rglob('*') if p.is_file()}
        return evidence

    def test_complete_evidence_accepts_and_partial_or_changed_rejects(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            evidence = self.fixture(root)
            validate_evidence(root, evidence, 'a'*40)
            for field in ('native_source_sha256', 'install_sha256'):
                bad = copy.deepcopy(evidence)
                bad[field].pop(next(iter(bad[field])))
                with self.assertRaisesRegex(ValueError, 'coverage'):
                    validate_evidence(root, bad, 'a'*40)
            (root/'install/lib/jikl_fc01_system/fc01_system_node').write_text('different binary')
            with self.assertRaisesRegex(ValueError, 'bytes differ'):
                validate_evidence(root, evidence, 'a'*40)

    def test_install_directory_link_is_explicitly_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            source = root/'install'
            source.mkdir()
            (source/'ordinary').mkdir()
            (source/'alias').symlink_to(source/'ordinary', target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'directory symlink'):
                copy_install(source, root/'output')

    def test_install_copy_excludes_bytecode_without_losing_python_source(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            source = root/'install'
            module = source/'lib/python3.10/site-packages/fc01_mission/mission_node.py'
            module.parent.mkdir(parents=True)
            module.write_text('CONTROL=False\n')
            for relative in ('__pycache__/mission_node.cpython-310.pyc', 'stray.pyc'):
                p = module.parent/relative
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_bytes(b'cache must not become immutable release data')
            copy_install(source, root/'output')
            self.assertEqual((root/'output'/module.relative_to(source)).read_bytes(), module.read_bytes())
            files = [p.relative_to(root/'output').as_posix()
                     for p in (root/'output').rglob('*') if p.is_file()]
            self.assertEqual(files, [module.relative_to(source).as_posix()])

    def test_target_source_closure_excludes_caches_and_unrelated_evidence(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            self.target_fixture(root)
            for name in ('target/src/fc01_mission/fc01_mission/__pycache__/old.pyc',
                         'target/src/fc01_vision/build/old.cpp', 'target/docs/audit.txt',
                         'tools/uart_unrelated.py', 'target/src/unreviewed_package/main.py'):
                p = root/name
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text('excluded')
            selected = source_names(root)
            self.assertIn('target/src/fc01_mission/fc01_mission/mission_node.py', selected)
            self.assertIn('target/src/fc01_bringup/launch/horizontal_first_test.launch.py', selected)
            self.assertFalse(any('old.' in p or 'unrelated' in p or 'unreviewed' in p or 'audit' in p
                                 for p in selected))

    def test_complete_target_evidence_requires_all_sources_and_runtime(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            evidence = self.target_fixture(root)
            validate_evidence(root, evidence, 'a'*40)
            for missing_field in ('target_source_sha256',):
                bad = copy.deepcopy(evidence)
                del bad[missing_field]
                with self.assertRaisesRegex(ValueError, 'coverage'):
                    validate_evidence(root, bad, 'a'*40)
            bad = copy.deepcopy(evidence)
            bad['target_source_sha256'].pop(next(iter(bad['target_source_sha256'])))
            with self.assertRaisesRegex(ValueError, 'coverage'):
                validate_evidence(root, bad, 'a'*40)
            p = root/'install/lib/fc01_mission/px4_state_bridge'
            p.unlink()
            evidence['install_sha256'].pop('lib/fc01_mission/px4_state_bridge')
            with self.assertRaisesRegex(ValueError, 'lacks target runtime'):
                validate_evidence(root, evidence, 'a'*40)

    def test_install_digest_alone_cannot_authorize_stale_mission_or_config(self):
        for name in ('lib/python3.10/site-packages/fc01_mission/mission_node.py',
                     'share/fc01_mission/config/mission.yaml',
                     'share/fc01_bringup/launch/robot.launch.py'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as t:
                root = Path(t)
                evidence = self.target_fixture(root)
                p = root/'install'/name
                p.write_text('valid hash but old behavior/configuration')
                evidence['install_sha256'][name] = sha(p)
                with self.assertRaisesRegex(ValueError, 'target (Python|configuration) differs'):
                    validate_evidence(root, evidence, 'a'*40)

    def test_multiple_python_message_libraries_cannot_hide_overlay_ambiguity(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            evidence = self.target_fixture(root)
            name = ('lib/python3.10/site-packages/jikl_fc01_system/'
                    'jikl_fc01_system_s__rosidl_typesupport_c.cpython-310-aarch64-linux-gnu.so')
            p = root/'install'/name
            p.parent.mkdir(parents=True)
            p.write_text('second copy')
            evidence['install_sha256'][name] = sha(p)
            with self.assertRaisesRegex(ValueError, 'unique ARM64 Python message typesupport'):
                validate_evidence(root, evidence, 'a'*40)

    def test_target_source_symlink_is_not_silently_dereferenced(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            self.target_fixture(root)
            (root/'target/src/fc01_mission/alias.py').symlink_to(root/'target/src/fc01_mission/fc01_mission/mission_node.py')
            with self.assertRaisesRegex(ValueError, 'target source contains symlink'):
                target_source_names(root)

    def test_standalone_installer_pins_archive_and_runs_without_project(self):
        with tempfile.TemporaryDirectory() as t:
            output = Path(t)/'maintenance installer.sh'
            write_installer((ROOT/'navigation_prebuilt_install.py').read_text(), 'a'*64, output)
            subprocess.run(['bash', '-n', str(output)], check=True)
            result = subprocess.run(['bash', str(output), '--help'], cwd=t,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--rollback', result.stdout)
            bad = subprocess.run(['bash', str(output), '--expected-sha256', 'b'*64, '/no-package'],
                                 cwd=t, capture_output=True, text=True)
            self.assertNotEqual(bad.returncode, 0)
            self.assertIn('invalid choice', bad.stderr)
            self.assertEqual(Path(str(output)+'.sha256').read_text(), sha(output)+'  '+output.name+'\n')
            self.assertTrue(output.stat().st_mode & 0o111)


if __name__ == '__main__':
    unittest.main()

"""ARM依赖收集的边界回归；合成ELF只核对文件合同，不冒称真实库可执行。"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import build_arm64_runtime as runtime


def elf(path, marker=b'arm', machine=183):
    header=bytearray(64)
    header[:6]=b'\x7fELF\x02\x01'
    header[18:20]=machine.to_bytes(2,'little')
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(bytes(header)+marker)
    return path


def dependency_free_elf(path, *, executable=False, needed=False):
    header=bytearray(64)
    header[:6]=b'\x7fELF\x02\x01';header[16:18]=(2 if executable else 3).to_bytes(2,'little')
    header[18:20]=(183).to_bytes(2,'little');header[32:40]=(64).to_bytes(8,'little')
    header[54:56]=(56).to_bytes(2,'little');header[56:58]=(1).to_bytes(2,'little')
    program=bytearray(56);program[:4]=(2).to_bytes(4,'little')
    program[8:16]=(120).to_bytes(8,'little');program[32:40]=(32 if needed else 16).to_bytes(8,'little')
    dynamic=(bytes([1])+bytes(15) if needed else b'')+bytes(16)
    path.write_bytes(header+program+dynamic)


class Arm64RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='fc01-runtime-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.rootfs=self.root/'rootfs';self.rootfs.mkdir()
        self.install=self.root/'build/install';self.install.mkdir(parents=True)
        self.binary=elf(self.install/'lib/fc01_vision/fc01_vision_node')
        self.lib=elf(self.rootfs/'usr/lib/aarch64-linux-gnu/libcamera_codec.so.2.1')
        alias=self.rootfs/'lib';alias.symlink_to('/usr/lib',target_is_directory=True)
        (self.lib.parent/'libcamera_codec.so.2').symlink_to('libcamera_codec.so.2.1')
        self.libc=elf(self.lib.parent/'libc.so.6',b'base')
        self.loader=elf(self.rootfs/'usr/lib/ld-linux-aarch64.so.1',b'loader')
        self.numpy=self.rootfs/'usr/lib/python3/dist-packages/numpy/__init__.py'
        self.numpy.parent.mkdir(parents=True)
        self.numpy.write_text('VERSION="fixture"\n')
        self.copyright=self.rootfs/'usr/share/doc/fixture-runtime/copyright'
        self.copyright.parent.mkdir(parents=True);self.copyright.write_text('Copyright fixture\n')
        self.report=self.root/'report.json'
        paths=dict(libcamera_codec='/lib/aarch64-linux-gnu/libcamera_codec.so.2',
                   libc='/lib/aarch64-linux-gnu/libc.so.6',loader='/lib/ld-linux-aarch64.so.1')
        stdout=(f"libcamera_codec.so.2 => {paths['libcamera_codec']} (0x550000)\n"
                f"libc.so.6 => {paths['libc']} (0x560000)\n"
                f"{paths['loader']} (0x570000)\n")
        dep={}
        for soname,guest,source in [('libcamera_codec.so.2',paths['libcamera_codec'],self.lib),
                                  ('libc.so.6',paths['libc'],self.libc),
                                  ('ld-linux-aarch64.so.1',paths['loader'],self.loader)]:
            dep[soname+'\0'+guest]=dict(path=guest,sha256=runtime.sha(source),package='fixture-runtime')
        self.data=dict(schema_version=1,architecture='aarch64',ubuntu='22.04',ros_distro='humble',
            install_prefix='/delivery/install',install_sha256=runtime.installed_files(self.install),
            selected_ros_packages=[],selected_python_packages=['fixture-runtime'],
            package_runtime_files=[dict(path='/usr/lib/python3/dist-packages/numpy/__init__.py',
                destination='lib/python3.10/site-packages/numpy/__init__.py',
                sha256=runtime.sha(self.numpy),package='fixture-runtime',executable=False)],
            elfs=[dict(path='/delivery/install/lib/fc01_vision/fc01_vision_node',
                sha256=runtime.sha(self.binary),returncode=0,stdout=stdout,stderr='')],
            dependencies=dep,packages={'fixture-runtime':dict(name='fixture-runtime',version='1',architecture='arm64')},
            copyrights={'fixture-runtime':dict(path='/usr/share/doc/fixture-runtime/copyright',sha256=runtime.sha(self.copyright))},
            limitations=['fixture'])

    def run_collect(self,data=None,output=None):
        self.report.write_text(json.dumps(self.data if data is None else data))
        dest=self.root/'output' if output is None else output
        runtime.collect(argparse.Namespace(rootfs=self.rootfs,install_tree=self.install,
            output=dest,ldd_report=self.report))
        return dest

    def test_runtime_closure_copies_soname_and_license_but_never_base_abi(self):
        out=self.run_collect()
        for name in ('libcamera_codec.so.2','libcamera_codec.so.2.1'):
            p=out/'lib'/name
            self.assertFalse(p.is_symlink());self.assertEqual(p.read_bytes(),self.lib.read_bytes())
        self.assertFalse((out/'lib/libc.so.6').exists())
        self.assertFalse((out/'lib/ld-linux-aarch64.so.1').exists())
        self.assertEqual((out/'share/fc01_runtime_dependencies/licenses/fixture-runtime/copyright').read_text(),'Copyright fixture\n')
        self.assertEqual((out/'lib/python3.10/site-packages/numpy/__init__.py').read_text(),self.numpy.read_text())
        summary=json.loads((out/'RUNTIME_DEPENDENCIES.json').read_text())
        self.assertEqual(len(summary['provided_by_target']),2)
        self.assertEqual(runtime.sha(self.binary),self.data['install_sha256']['lib/fc01_vision/fc01_vision_node'])

    def test_guest_absolute_and_relative_links_stay_inside_arm_root(self):
        p=runtime.resolve_guest(self.rootfs,'/lib/aarch64-linux-gnu/libcamera_codec.so.2')
        self.assertEqual(p,self.lib)
        (self.rootfs/'escape').symlink_to('../../outside')
        with self.assertRaisesRegex(ValueError,'escapes'):
            runtime.resolve_guest(self.rootfs,'/escape/file')

    def test_scan_identity_rejects_changed_build_or_runtime_dependency(self):
        self.binary.write_bytes(self.binary.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError,'install changed'):
            self.run_collect()
        self.data['install_sha256']=runtime.installed_files(self.install)
        self.data['elfs'][0]['sha256']=runtime.sha(self.binary)
        self.lib.write_bytes(self.lib.read_bytes()+b'changed')
        with self.assertRaisesRegex(ValueError,'changed after scan'):
            self.run_collect()
        self.assertFalse((self.root/'output').exists())

    def test_incomplete_or_foreign_ldd_is_rejected(self):
        data=copy.deepcopy(self.data);data['elfs']=[]
        with self.assertRaisesRegex(ValueError,'exactly cover'):
            self.run_collect(data)
        data=copy.deepcopy(self.data);data['elfs'][0]['stdout']='libabsent.so => not found\n'
        with self.assertRaisesRegex(ValueError,'unresolved'):
            self.run_collect(data)
        data=copy.deepcopy(self.data);data['elfs'][0]['path']='/unexpected/native'
        with self.assertRaisesRegex(ValueError,'not bound'):
            self.run_collect(data)

    def test_wrong_architecture_dependency_and_unsafe_destination_are_rejected(self):
        elf(self.lib,machine=62)
        data=copy.deepcopy(self.data)
        for record in data['dependencies'].values():
            if 'codec' in record['path']:record['sha256']=runtime.sha(self.lib)
        with self.assertRaisesRegex(ValueError,'AArch64'):
            self.run_collect(data)
        data=copy.deepcopy(self.data)
        data['package_runtime_files'][0]['destination']='../../outside'
        with self.assertRaisesRegex(ValueError,'relative'):
            self.run_collect(data)

    def test_existing_output_or_collision_never_overwrites(self):
        dest=self.root/'existing';dest.mkdir();(dest/'user').write_text('keep')
        with self.assertRaisesRegex(ValueError,'must not exist'):
            self.run_collect(output=dest)
        self.assertEqual((dest/'user').read_text(),'keep')
        collision=self.install/'lib/libcamera_codec.so.2';collision.write_text('not the same')
        self.data['install_sha256']=runtime.installed_files(self.install)
        with self.assertRaisesRegex(ValueError,'collision'):
            self.run_collect()
        self.assertFalse((self.root/'output').exists())

    def test_source_selection_excludes_cache_headers_and_unused_cv_python(self):
        self.assertIsNone(runtime.runtime_ros_destination('/opt/ros/humble/include/cv_bridge/file.h'))
        self.assertIsNone(runtime.runtime_ros_destination('/opt/ros/humble/local/lib/python3.10/dist-packages/cv_bridge/core.py'))
        self.assertIsNone(runtime.runtime_python_destination('/usr/lib/python3/dist-packages/numpy/__pycache__/a.pyc'))
        self.assertEqual(runtime.runtime_ros_destination('/opt/ros/humble/lib/libcv_bridge.so'),'lib/libcv_bridge.so')
        self.assertEqual(runtime.runtime_python_destination('/usr/lib/python3/dist-packages/numpy/core/a.so'),
                         'lib/python3.10/site-packages/numpy/core/a.so')

    def test_soname_conflict_and_unexpected_ldd_text_fail_closed(self):
        with self.assertRaisesRegex(ValueError,'ambiguous'):
            runtime.parse_ldd('liba.so => /lib/a.so (0x1)\nliba.so => /lib/b.so (0x2)\n')
        with self.assertRaisesRegex(ValueError,'unrecognized'):
            runtime.parse_ldd('arbitrary command output\n')
        with self.assertRaisesRegex(ValueError,'empty'):
            runtime.parse_ldd('linux-vdso.so.1 (0x1)\n')

    def test_selected_ros_package_gets_colcon_registration_for_its_hooks(self):
        source=self.rootfs/'opt/ros/humble/share/sensor_msgs/package.xml'
        source.parent.mkdir(parents=True)
        source.write_text('<package><name>sensor_msgs</name><depend>geometry_msgs</depend>'
                          '<exec_depend>rosidl_default_runtime</exec_depend>'
                          '<exec_depend>python3-yaml</exec_depend></package>')
        data=copy.deepcopy(self.data)
        data['package_runtime_files'].append(dict(path='/opt/ros/humble/share/sensor_msgs/package.xml',
            destination='share/sensor_msgs/package.xml',sha256=runtime.sha(source),
            package='fixture-runtime',executable=False))
        out=self.run_collect(data)
        marker=out/'share/colcon-core/packages/sensor_msgs'
        self.assertEqual(set(marker.read_text().split(':')),
            {'fc01_runtime_dependencies','geometry_msgs','rosidl_default_runtime','python3-yaml'})
        self.assertEqual((out/'share/sensor_msgs/package.xml').read_bytes(),source.read_bytes())
        summary=json.loads((out/'RUNTIME_DEPENDENCIES.json').read_text())
        self.assertEqual(summary['generated_ros_colcon_markers'],['sensor_msgs'])

    def test_scan_repeated_external_dependency_keeps_provenance_consistent(self):
        # 两个ELF共同依赖同一库：第二次扫描不可把已添加的dpkg来源误判为字节变化。
        elf(self.install/'lib/second.so')
        info=dict(name='fixture-runtime',version='1',architecture='arm64')
        text=f'libcodec.so.2 => {self.lib} (0x550000)\n'
        result=argparse.Namespace(returncode=0,stdout=text,stderr='')
        real_text=Path.read_text;real_is_file=Path.is_file;real_sha=runtime.sha
        def read_text(path,*args,**kwargs):
            return 'VERSION_ID="22.04"\n' if str(path)=='/etc/os-release' else real_text(path,*args,**kwargs)
        def is_file(path):
            return True if str(path)=='/usr/share/doc/fixture-runtime/copyright' else real_is_file(path)
        def digest(path):
            return 'a'*64 if str(path)=='/usr/share/doc/fixture-runtime/copyright' else real_sha(path)
        with mock.patch.object(runtime.platform,'machine',return_value='aarch64'), \
             mock.patch.dict(runtime.os.environ,{'ROS_DISTRO':'humble'}), \
             mock.patch.object(Path,'read_text',read_text), \
             mock.patch.object(Path,'is_file',is_file), \
             mock.patch.object(runtime,'sha',side_effect=digest), \
             mock.patch.object(runtime,'command',return_value=''), \
             mock.patch.object(runtime,'package_info',return_value=info), \
             mock.patch.object(runtime,'package_owner',return_value=info), \
             mock.patch.object(runtime.subprocess,'run',return_value=result):
            runtime.scan(argparse.Namespace(install_tree=self.install,report=self.report,
                ros_package=['fixture-runtime'],python_package=['fixture-runtime']))
        report=json.loads(self.report.read_text())
        self.assertEqual(len(report['elfs']),2)
        self.assertEqual(len(report['dependencies']),1)
        self.assertEqual(next(iter(report['dependencies'].values()))['package'],'fixture-runtime')

    def test_atomic_publish_preserves_concurrently_created_empty_output(self):
        source=self.root/'temporary';source.mkdir();(source/'payload').write_text('candidate')
        destination=self.root/'concurrent';destination.mkdir()
        with self.assertRaisesRegex(ValueError,'appeared during'):
            runtime.publish_no_replace(source,destination)
        self.assertEqual(list(destination.iterdir()),[])
        self.assertEqual((source/'payload').read_text(),'candidate')

    def test_ldd_dependency_free_python_extension_requires_verified_dynamic_elf(self):
        dependency_free_elf(self.binary)
        self.assertTrue(runtime.dependency_free_shared_object(self.binary))
        self.assertEqual(runtime.checked_ldd_dependencies(self.binary,'statically linked\n'),{})
        data=copy.deepcopy(self.data)
        data['install_sha256']=runtime.installed_files(self.install)
        data['elfs'][0].update(sha256=runtime.sha(self.binary),stdout='statically linked\n')
        data['dependencies']={}
        out=self.run_collect(data)
        self.assertEqual((out/'lib/fc01_vision/fc01_vision_node').read_bytes(),self.binary.read_bytes())
        for options in (dict(executable=True),dict(needed=True)):
            dependency_free_elf(self.binary,**options)
            with self.assertRaisesRegex(ValueError,'non-dynamic'):
                runtime.checked_ldd_dependencies(self.binary,'statically linked\n')
        self.binary.write_bytes(self.binary.read_bytes()[:100])
        with self.assertRaisesRegex(ValueError,'non-dynamic'):
            runtime.checked_ldd_dependencies(self.binary,'statically linked\n')


if __name__=='__main__':unittest.main()

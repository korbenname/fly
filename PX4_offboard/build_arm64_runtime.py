#!/usr/bin/env python3
"""收集已编译ARM64程序的离线视觉依赖，不修改系统库或飞行配置。

scan必须在Ubuntu22.04 ARM64/Humble构建环境执行，记录真实ldd、包来源和SHA。
collect可在宿主执行，只把已验证文件复制到新的merged install目录；目标设备不联网。
ldd不能发现所有dlopen插件，生成物仍须做搬迁后的导入/相机无设备启动验证。
"""
from __future__ import annotations

import argparse
import ctypes
import errno
import functools
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

SCHEMA = 1
# 保留目标Ubuntu的加载器与C/C++基本ABI；绝不能向私有前缀投放另一套glibc。
BASE_SONAMES = frozenset((
    'ld-linux-aarch64.so.1', 'libc.so.6', 'libm.so.6', 'libdl.so.2',
    'libpthread.so.0', 'librt.so.1', 'libresolv.so.2', 'libutil.so.1',
    'libanl.so.1', 'libBrokenLocale.so.1', 'libthread_db.so.1',
    'libstdc++.so.6', 'libgcc_s.so.1',
))
ROS_PREFIX = PurePosixPath('/opt/ros/humble')
PY_PREFIX = PurePosixPath('/usr/lib/python3/dist-packages')
DEFAULT_ROS_PACKAGES = ('ros-humble-cv-bridge', 'ros-humble-sensor-msgs')
PACKAGE_PATTERN = re.compile(r'[a-z0-9][a-z0-9+.-]*(?::(?:arm64|all))?')
SHA_PATTERN = re.compile(r'[0-9a-f]{64}')
CACHE_PARTS = frozenset(('__pycache__', '.pytest_cache'))
ROS_NAME_PATTERN = re.compile(r'[a-zA-Z][a-zA-Z0-9_]*')
ROS_DEPENDENCY_PATTERN = re.compile(r'[a-zA-Z][a-zA-Z0-9_-]*')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def clean_relative(value):
    p = PurePosixPath(value)
    if (not isinstance(value, str) or not value or p.is_absolute() or
            '..' in p.parts or '\\' in value or p.as_posix() != value):
        raise ValueError('invalid relative path: ' + str(value))
    return p


def absolute_guest(value):
    if not isinstance(value, str):
        raise ValueError('invalid guest path')
    p = PurePosixPath(value)
    if (not p.is_absolute() or '..' in p.parts or '\\' in value or
            p.as_posix() != value or any(ord(c) < 32 for c in value)):
        raise ValueError('invalid guest path: ' + value)
    return p


def is_elf(path):
    with Path(path).open('rb') as stream:
        return stream.read(4) == b'\x7fELF'


def require_arm_elf(path):
    with Path(path).open('rb') as stream:
        h = stream.read(64)
    if (len(h) < 64 or h[:6] != b'\x7fELF\x02\x01' or
            int.from_bytes(h[18:20], 'little') != 183):
        raise ValueError('not a little-endian AArch64 ELF: ' + str(path))


def runtime_ros_destination(guest):
    """只收ROS运行文件；cv_bridge的Python封装依赖cv2，但本项目仅用其C++库。"""
    p = absolute_guest(guest)
    if not p.is_relative_to(ROS_PREFIX):
        return None
    rel = p.relative_to(ROS_PREFIX)
    if not rel.parts or rel.parts[0] not in ('bin', 'lib', 'local', 'share'):
        return None
    if (set(rel.parts) & (CACHE_PARTS | {'include', 'cmake', 'rust'}) or
            p.suffix in ('.pyc', '.pyo', '.a', '.h', '.hpp')):
        return None
    if 'cv_bridge' in rel.parts and ('dist-packages' in rel.parts or 'site-packages' in rel.parts):
        return None
    if p.name.startswith('cv_bridge-') and ('dist-packages' in rel.parts or 'site-packages' in rel.parts):
        return None
    return rel.as_posix()


def runtime_python_destination(guest):
    p = absolute_guest(guest)
    if not p.is_relative_to(PY_PREFIX):
        return None
    rel = p.relative_to(PY_PREFIX)
    if not rel.parts or set(rel.parts) & CACHE_PARTS or p.suffix in ('.pyc', '.pyo', '.a', '.h', '.hpp'):
        return None
    return (PurePosixPath('lib/python3.10/site-packages') / rel).as_posix()


def command(argv):
    result = subprocess.run(argv, capture_output=True, text=True, check=False, timeout=120)
    if result.returncode:
        raise ValueError('command failed: ' + repr(argv) + '\n' + result.stdout + result.stderr)
    return result.stdout


@functools.lru_cache(maxsize=None)
def package_info(name):
    if not isinstance(name, str) or not PACKAGE_PATTERN.fullmatch(name):
        raise ValueError('invalid Debian package: ' + str(name))
    text = command(['dpkg-query', '-W', '-f=${Package}\t${Version}\t${Architecture}\t${Status}\n', name])
    fields = text.strip().split('\t')
    if len(fields) != 4 or fields[3] != 'install ok installed' or fields[2] not in ('arm64', 'all'):
        raise ValueError('not an installed ARM64/all package: ' + name)
    return dict(name=fields[0], version=fields[1], architecture=fields[2])


@functools.lru_cache(maxsize=None)
def package_owner(path):
    """先按原路径查dpkg；usrmerge别名不在list时查真实路径与/lib别名。"""
    candidates = [str(path), str(Path(path).resolve())]
    for item in list(candidates):
        if item.startswith('/usr/lib/'):
            candidates.append(item[4:])
        elif item.startswith('/lib/'):
            candidates.append('/usr' + item)
    for item in dict.fromkeys(candidates):
        r = subprocess.run(['dpkg-query', '-S', item], capture_output=True, text=True, check=False)
        if r.returncode == 0:
            for row in r.stdout.splitlines():
                if ': ' in row:
                    name, filename = row.split(': ', 1)
                    if filename == item and ', ' not in name:
                        return package_info(name)
    raise ValueError('runtime dependency lacks Debian package provenance: ' + str(path))


def parse_ldd(text):
    """返回真实ldd中SONAME→绝对路径；未知/缺失内容拒绝，vDSO不是磁盘文件。"""
    found = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith('linux-vdso.so.'):
            continue
        if '=> not found' in line or 'not a dynamic executable' in line or 'statically linked' in line:
            raise ValueError('unresolved/non-dynamic ldd result: ' + line)
        match = re.fullmatch(r'(\S+) => (/\S+) \(0x[0-9a-fA-F]+\)', line)
        if match:
            soname, guest = match.groups()
        else:
            match = re.fullmatch(r'(/\S+) \(0x[0-9a-fA-F]+\)', line)
            if not match:
                raise ValueError('unrecognized ldd line: ' + line)
            guest = match.group(1)
            soname = PurePosixPath(guest).name
        absolute_guest(guest)
        if '/' in soname or soname in ('.', '..'):
            raise ValueError('invalid SONAME from ldd')
        if soname in found and found[soname] != guest:
            raise ValueError('ambiguous ldd SONAME: ' + soname)
        found[soname] = guest
    if not found:
        raise ValueError('empty ldd dependency set')
    return found


def dependency_free_shared_object(path):
    """部分Python扩展没有DT_NEEDED，ldd会打印statically linked。

    仅允许真正的ET_DYN共享对象：无解释器，具有有界动态段且没有依赖项。
    普通静态程序、静态PIE和损坏的ELF均不能借此绕过依赖验证。
    """
    require_arm_elf(path)
    size = Path(path).stat().st_size
    with Path(path).open('rb') as stream:
        header = stream.read(64)
        if int.from_bytes(header[16:18], 'little') != 3:
            return False
        offset = int.from_bytes(header[32:40], 'little')
        entry_size = int.from_bytes(header[54:56], 'little')
        count = int.from_bytes(header[56:58], 'little')
        if entry_size != 56 or not count or offset < 64 or offset + entry_size * count > size:
            return False
        dynamic = []
        for index in range(count):
            stream.seek(offset + index * entry_size)
            program = stream.read(entry_size)
            kind = int.from_bytes(program[:4], 'little')
            if kind == 3:  # PT_INTERP表示可执行程序，不是独立加载的共享对象。
                return False
            if kind == 2:  # PT_DYNAMIC中每个Elf64_Dyn条目为16字节。
                start = int.from_bytes(program[8:16], 'little')
                length = int.from_bytes(program[32:40], 'little')
                if not length or length % 16 or start + length > size:
                    return False
                dynamic.append((start, length))
        if len(dynamic) != 1:
            return False
        stream.seek(dynamic[0][0])
        for _ in range(dynamic[0][1] // 16):
            tag = int.from_bytes(stream.read(16)[:8], 'little', signed=True)
            if tag == 1:  # DT_NEEDED
                return False
            if tag == 0:  # DT_NULL，动态段合法结束。
                return True
    return False


def checked_ldd_dependencies(path, text):
    if text.strip() == 'statically linked' and dependency_free_shared_object(path):
        return {}
    return parse_ldd(text)


def installed_files(tree):
    tree = Path(tree).resolve()
    result = {}
    for p in sorted(tree.rglob('*')):
        rel = p.relative_to(tree)
        if set(rel.parts) & CACHE_PARTS or p.suffix in ('.pyc', '.pyo'):
            continue
        if p.is_symlink() and p.is_dir():
            raise ValueError('install directory symlink is not portable: ' + str(p))
        if p.is_dir():
            continue
        if not p.is_file() or not p.resolve().is_relative_to(tree.parent):
            raise ValueError('unbounded install file: ' + str(p))
        clean_relative(rel.as_posix())
        result[rel.as_posix()] = sha(p)
    return result


def scan(args):
    """在真实ARM64用户态记录输入；不启动ROS节点、相机、Agent或服务。"""
    if platform.machine().lower() not in ('aarch64', 'arm64'):
        raise ValueError('scan must run inside native ARM64 userland')
    release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    if release.get('VERSION_ID', '').strip('"') != '22.04' or os.environ.get('ROS_DISTRO') != 'humble':
        raise ValueError('scan requires Ubuntu22.04 and sourced ROS2 Humble')
    install = Path(args.install_tree).resolve()
    inventory = installed_files(install)
    sources = {}
    packages = {}
    ros_packages = args.ros_package or list(DEFAULT_ROS_PACKAGES)
    python_packages = args.python_package or ['python3-numpy']
    for name, mapper in ([(n, runtime_ros_destination) for n in ros_packages] +
                         [(n, runtime_python_destination) for n in python_packages]):
        info = package_info(name)
        packages[info['name']] = info
        for guest in command(['dpkg-query', '-L', name]).splitlines():
            dest = mapper(guest) if guest.startswith('/') and guest != '/.' else None
            if dest is None or not Path(guest).is_file():
                continue
            record = dict(path=guest, destination=dest, sha256=sha(guest), package=info['name'],
                          executable=bool(Path(guest).stat().st_mode & 0o111))
            if dest in sources and sources[dest] != record:
                raise ValueError('selected package destination conflict: ' + dest)
            sources[dest] = record
    # 每个输入ELF都运行ldd；ldd包含其递归依赖，后续逐个验证路径和字节。
    candidates = {str(install / name): h for name, h in inventory.items() if is_elf(install / name)}
    candidates.update({r['path']: r['sha256'] for r in sources.values() if is_elf(r['path'])})
    elfs = []
    deps = {}
    for name, digest in sorted(candidates.items()):
        require_arm_elf(name)
        result = subprocess.run(['ldd', name], capture_output=True, text=True, check=False, timeout=120)
        if result.returncode or result.stderr.strip():
            raise ValueError('ldd failed: ' + name + '\n' + result.stdout + result.stderr)
        try:
            dependencies = checked_ldd_dependencies(name, result.stdout)
        except ValueError as error:
            raise ValueError('ldd inventory invalid for ' + name + ': ' + str(error)) from error
        elfs.append(dict(path=name, sha256=digest, returncode=0, stdout=result.stdout, stderr=result.stderr))
        for soname, guest in dependencies.items():
            require_arm_elf(guest)
            item = dict(path=guest, sha256=sha(guest))
            key = soname + '\0' + guest
            if not PurePosixPath(guest).is_relative_to(PurePosixPath(str(install))):
                info = package_owner(guest)
                packages[info['name']] = info
                item['package'] = info['name']
            if key in deps and deps[key] != item:
                raise ValueError('dependency changed during scan: ' + guest)
            deps[key] = item
    copyrights = {}
    for name in sorted(packages):
        path = Path('/usr/share/doc') / name / 'copyright'
        if not path.is_file():
            raise ValueError('missing dependency copyright: ' + name)
        copyrights[name] = dict(path=str(path), sha256=sha(path))
    report = dict(schema_version=SCHEMA, architecture='aarch64', ubuntu='22.04', ros_distro='humble',
                  install_prefix=str(install), install_sha256=inventory, selected_ros_packages=ros_packages,
                  selected_python_packages=python_packages, package_runtime_files=list(sources.values()),
                  elfs=elfs, dependencies=deps, packages=packages, copyrights=copyrights,
                  limitations=['ldd cannot discover every runtime dlopen plugin; relocated runtime validation required',
                               'glibc/loader/libstdc++/libgcc and existing unselected Humble packages remain target dependencies'])
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(report=str(args.report), elfs=len(elfs), packages=len(packages),
                          external_files=len(sources), install_files=len(inventory))))


def resolve_guest(rootfs, guest):
    """逐分量解引用：guest中的/lib绝对链接始终相对rootfs，而不是宿主根目录。"""
    root = Path(rootfs).resolve()
    pending = list(absolute_guest(guest).parts[1:])
    current = root
    links = 0
    while pending:
        part = pending.pop(0)
        if part in ('', '.'):
            continue
        if part == '..':
            if current == root:
                raise ValueError('guest link escapes rootfs: ' + guest)
            current = current.parent
            continue
        candidate = current / part
        if candidate.is_symlink():
            links += 1
            if links > 64:
                raise ValueError('guest symlink loop: ' + guest)
            target = os.readlink(candidate)
            link = PurePosixPath(target)
            if link.is_absolute():
                current = root
                pending = list(link.parts[1:]) + pending
            else:
                pending = list(link.parts) + pending
        else:
            current = candidate
        if not current.is_relative_to(root):
            raise ValueError('guest path escapes rootfs: ' + guest)
    if not current.is_file() or current.is_symlink():
        raise ValueError('guest source not an ordinary file: ' + guest)
    return current


def checked_record(rootfs, record):
    if not isinstance(record, dict) or not SHA_PATTERN.fullmatch(str(record.get('sha256', ''))):
        raise ValueError('invalid source record')
    source = resolve_guest(rootfs, record['path'])
    if sha(source) != record['sha256']:
        raise ValueError('runtime source changed after scan: ' + record['path'])
    return source


def put_file(source, destination, expected, *, executable=False):
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if not destination.is_file() or destination.is_symlink() or sha(destination) != expected:
            raise ValueError('runtime destination collision: ' + str(destination))
        return
    shutil.copyfile(source, destination)
    destination.chmod(0o755 if executable else 0o644)
    if sha(destination) != expected:
        raise ValueError('runtime copy readback mismatch: ' + str(destination))


def register_ros_packages(prefix, selected):
    """Debian ROS包没有colcon索引；补索引使merged环境发现其原有运行钩子。

    保留发行包的package.dsv字节，索引中的依赖只用于排列本前缀的加载顺序。
    不在本前缀的依赖由已有Humble提供，colcon会自动忽略它们。
    """
    registered = []
    for record in selected:
        rel = PurePosixPath(record['destination'])
        if len(rel.parts) != 3 or rel.parts[0] != 'share' or rel.name != 'package.xml':
            continue
        metadata = ET.parse(prefix / rel).getroot()
        name = (metadata.findtext('name') or '').strip()
        if not ROS_NAME_PATTERN.fullmatch(name) or rel.parts[1] != name:
            raise ValueError('ROS package metadata name does not match destination: ' + str(rel))
        dependencies = {'fc01_runtime_dependencies'}
        for tag in ('depend', 'exec_depend', 'build_export_depend'):
            for element in metadata.findall(tag):
                dependency = (element.text or '').strip()
                # ROS元数据也可使用python3-yaml这类系统rosdep键，允许连字符。
                if not ROS_DEPENDENCY_PATTERN.fullmatch(dependency):
                    raise ValueError('invalid ROS runtime dependency in ' + str(rel))
                dependencies.add(dependency)
        dependencies.discard(name)
        marker = prefix / 'share/colcon-core/packages' / name
        marker.parent.mkdir(parents=True, exist_ok=True)
        # 原构建产物的索引属于构建器；只为额外收集的Debian包补缺失项。
        if not marker.exists():
            marker.write_text(':'.join(sorted(dependencies)))
            registered.append(name)
    return sorted(registered)


def publish_no_replace(source, destination):
    """使用Linux原子操作发布新目录，并发创建的同名目录也绝不覆盖。"""
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise ValueError('renameat2 unavailable; output was not published')
    rename.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        code = ctypes.get_errno()
        if code in (errno.EEXIST, errno.ENOTEMPTY):
            raise ValueError('output appeared during collection; existing directory was preserved')
        raise OSError(code, os.strerror(code), str(destination))


def collect(args):
    """从有界ARM根目录收集依赖到新目录；任一校验失败清理临时输出。"""
    install = Path(args.install_tree).resolve()
    target = Path(args.output).absolute()
    if target.exists() or target.is_symlink():
        raise ValueError('output must not exist; no existing install is overwritten')
    report = json.loads(Path(args.ldd_report).read_text())
    if (report.get('schema_version') != SCHEMA or report.get('architecture') != 'aarch64' or
            report.get('ubuntu') != '22.04' or report.get('ros_distro') != 'humble'):
        raise ValueError('unsupported dependency report')
    original_inventory = installed_files(install)
    if original_inventory != report.get('install_sha256'):
        raise ValueError('build install changed after ldd scan')
    guest_install = absolute_guest(report['install_prefix'])
    packages = report.get('packages')
    if not isinstance(packages, dict) or not isinstance(report.get('copyrights'), dict):
        raise ValueError('missing package provenance')
    selected = report.get('package_runtime_files')
    if not isinstance(selected, list) or not isinstance(report.get('elfs'), list):
        raise ValueError('missing runtime/ELF inventory')
    selected_by_guest = {}
    all_elfs = {str(guest_install / name): digest for name, digest in original_inventory.items()
                if is_elf(install / name)}
    for record in selected:
        path, dest = record['path'], record['destination']
        source = checked_record(args.rootfs, record)
        clean_relative(dest)
        mapper = runtime_ros_destination if PurePosixPath(path).is_relative_to(ROS_PREFIX) else runtime_python_destination
        if mapper(path) != dest or record.get('package') not in packages:
            raise ValueError('runtime record destination/provenance invalid: ' + path)
        if path in selected_by_guest:
            raise ValueError('duplicate selected runtime source: ' + path)
        selected_by_guest[path] = record
        if is_elf(source):
            require_arm_elf(source)
            all_elfs[path] = record['sha256']
    seen_elfs = {}
    resolved_deps = {}
    for row in report['elfs']:
        if row.get('returncode') != 0 or row.get('stderr') or row.get('path') in seen_elfs:
            raise ValueError('invalid ldd execution record')
        path = str(absolute_guest(row['path']))
        seen_elfs[path] = row.get('sha256')
        if row.get('sha256') != all_elfs.get(path):
            raise ValueError('ldd ELF not bound to current inputs: ' + path)
        elf_source = (install / PurePosixPath(path).relative_to(guest_install)
                      if PurePosixPath(path).is_relative_to(guest_install)
                      else checked_record(args.rootfs, selected_by_guest[path]))
        for soname, guest in checked_ldd_dependencies(elf_source, row['stdout']).items():
            record = report.get('dependencies', {}).get(soname + '\0' + guest)
            if not record or record.get('path') != guest:
                raise ValueError('ldd dependency lacks bound digest: ' + guest)
            if PurePosixPath(guest).is_relative_to(guest_install):
                rel = PurePosixPath(guest).relative_to(guest_install).as_posix()
                if original_inventory.get(rel) != record.get('sha256'):
                    raise ValueError('ldd resolved stale/foreign build install file: ' + guest)
                continue
            source = checked_record(args.rootfs, record)
            require_arm_elf(source)
            if record.get('package') not in packages:
                raise ValueError('ldd dependency lacks package provenance: ' + guest)
            # 相同SONAME不能按遍历顺序覆盖为另一套库。
            if soname in resolved_deps and resolved_deps[soname]['sha256'] != record['sha256']:
                raise ValueError('ambiguous runtime SONAME bytes: ' + soname)
            resolved_deps[soname] = record
    if seen_elfs != all_elfs:
        raise ValueError('ldd evidence does not exactly cover every runtime ELF')
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.' + target.name + '.collect-', dir=target.parent))
    copied = []
    existing = []
    try:
        for name, digest in original_inventory.items():
            put_file(install / name, temporary / name, digest, executable=os.access(install / name, os.X_OK))
        for record in selected:
            source = checked_record(args.rootfs, record)
            put_file(source, temporary / record['destination'], record['sha256'], executable=record['executable'])
            copied.append(record)
        for soname, record in sorted(resolved_deps.items()):
            guest = PurePosixPath(record['path'])
            if soname in BASE_SONAMES:
                existing.append(dict(soname=soname, reason='Ubuntu base ABI, never vendored', **record))
                continue
            if guest.is_relative_to(ROS_PREFIX) and record['path'] not in selected_by_guest:
                existing.append(dict(soname=soname, reason='existing ROS2 Humble base dependency', **record))
                continue
            source = checked_record(args.rootfs, record)
            # SONAME是加载器要找的文件名；同时保留真实库文件名以便审计，无符号链接。
            for name in dict.fromkeys((soname, source.name)):
                put_file(source, temporary / 'lib' / name, record['sha256'], executable=False)
                copied.append(dict(destination='lib/' + name, soname=soname, **record))
        for name, info in packages.items():
            if not PACKAGE_PATTERN.fullmatch(name) or not isinstance(info, dict) or info.get('name') != name:
                raise ValueError('invalid package provenance name')
            copyright_record = report['copyrights'].get(name)
            source = checked_record(args.rootfs, copyright_record)
            put_file(source, temporary / 'share/fc01_runtime_dependencies/licenses' / name / 'copyright',
                     copyright_record['sha256'])
        # colcon可迁移dsv钩子；不包含构建机绝对路径，不覆盖Ubuntu环境。
        hook = temporary / 'share/fc01_runtime_dependencies/package.dsv'
        hook.parent.mkdir(parents=True, exist_ok=True)
        hook.write_text('prepend-non-duplicate;LD_LIBRARY_PATH;lib\n'
                        'prepend-non-duplicate;PYTHONPATH;lib/python3.10/site-packages\n'
                        'prepend-non-duplicate;PYTHONPATH;local/lib/python3.10/dist-packages\n')
        marker = temporary / 'share/colcon-core/packages/fc01_runtime_dependencies'
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.write_text('')
        resource = temporary / 'share/ament_index/resource_index/packages/fc01_runtime_dependencies'
        resource.parent.mkdir(parents=True, exist_ok=True)
        resource.write_text('')
        registered = register_ros_packages(temporary, selected)
        summary = dict(schema_version=SCHEMA, architecture='aarch64', ubuntu='22.04', ros_distro='humble',
                       input_report_sha256=sha(args.ldd_report), build_install_files=len(original_inventory),
                       selected_ros_packages=report['selected_ros_packages'],
                       selected_python_packages=report['selected_python_packages'], packages=packages,
                       copied=copied, provided_by_target=existing, copyrights=report['copyrights'],
                       generated_ros_colcon_markers=registered, limitations=report['limitations'])
        (temporary / 'RUNTIME_DEPENDENCIES.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        # 新输出只允许普通文件；随后主发行器再次验证ELF和完整manifest。
        inventory = installed_files(temporary)
        if not all(sha(temporary / p) == digest for p, digest in inventory.items()):
            raise ValueError('collected runtime readback mismatch')
        publish_no_replace(temporary, target)
        print(json.dumps(dict(output=str(target), files=len(inventory), copied_records=len(copied),
                              provided_by_target=len(existing)), ensure_ascii=False))
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    scan_parser = commands.add_parser('scan', help='在ARM64构建环境执行ldd并记录包来源')
    scan_parser.add_argument('--install-tree', required=True, type=Path)
    scan_parser.add_argument('--report', required=True, type=Path)
    scan_parser.add_argument('--ros-package', action='append')
    scan_parser.add_argument('--python-package', action='append')
    collect_parser = commands.add_parser('collect', help='在宿主按真实ARM证据生成新可迁移安装树')
    collect_parser.add_argument('--rootfs', required=True, type=Path)
    collect_parser.add_argument('--install-tree', required=True, type=Path)
    collect_parser.add_argument('--output', required=True, type=Path)
    collect_parser.add_argument('--ldd-report', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        (scan if args.action == 'scan' else collect)(args)
    except (ValueError, OSError, KeyError, TypeError, ET.ParseError, subprocess.SubprocessError) as error:
        print('FC01_ARM64_RUNTIME_FAILED: ' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

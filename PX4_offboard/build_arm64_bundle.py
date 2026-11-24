#!/usr/bin/env python3
"""把已构建的 ARM64 安装树和已提交源码封装成离线发行；不编译或激活服务。"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile

from build_navigation_bundle import names
from navigation_prebuilt_install import extract_checked, safe_name, verify_elf

ROOT = Path(__file__).resolve().parent
EXTRA = ('navigation_prebuilt_install.py', 'navigation_prebuilt_service.py',
         'navigation_vision_mode.py', 'prebuilt_boot.sh',
         'docs/FC01_VISION_SITE_TEMPLATE.json', 'docs/FC01_VISION_SITE_CONFIG_zh.md',
         'docs/FC01_ARM64_INSTALL_zh.md')
TARGET_PACKAGES = ('fc01_vision', 'fc01_mission', 'fc01_bringup')
TARGET_ROOT_FILES = ('target/README.md', 'target/UPSTREAM.json', 'target/build.sh')
# ARM64预编译包单独带目标业务源码；旧源码.sh安装器仍保留自己的有界依赖闭包。
# 开发缓存、历史证据和未跟踪UART工具不因此进入运行载荷。
TARGET_CACHE_PARTS = frozenset(('__pycache__', '.pytest_cache', 'build', 'install', 'log'))
TARGET_INSTALL_FILES = frozenset((
    'lib/fc01_vision/fc01_vision_node',
    'lib/fc01_vision/vision_image_node',
    'lib/fc01_mission/mission',
    'lib/fc01_mission/px4_state_bridge',
    'share/fc01_mission/config/mission.yaml',
    'share/fc01_mission/config/platform.yaml',
    'share/fc01_vision/config/vision.yaml',
    'share/fc01_bringup/launch/robot.launch.py',
    'share/fc01_bringup/launch/horizontal_first_test.launch.py',
    'share/jikl_fc01_system/msg/NativeFlightSnapshot.msg',
    'share/jikl_fc01_system/msg/TargetObservation.msg',
    'share/jikl_fc01_system/msg/HorizontalVelocityCommand.msg',
    'lib/libjikl_fc01_system__rosidl_typesupport_cpp.so',
    'lib/libjikl_fc01_system__rosidl_typesupport_c.so',
    *('share/ament_index/resource_index/packages/' + name for name in TARGET_PACKAGES),
))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def target_source_names(root):
    """只选择三个已审查业务包，拒绝链接；调用者只传Git已提交的独立源码树。"""
    root = Path(root)
    selected = set()
    for package in TARGET_PACKAGES:
        package_root = root / 'target/src' / package
        if not package_root.is_dir() or package_root.is_symlink():
            raise ValueError('missing committed target package: ' + package)
        for path in package_root.rglob('*'):
            relative = path.relative_to(root)
            if set(relative.parts) & TARGET_CACHE_PARTS or path.suffix == '.pyc':
                continue
            if any(parent.is_symlink() for parent in (path, *path.parents) if parent != root):
                raise ValueError('target source contains symlink: ' + str(relative))
            if path.is_file():
                selected.add(relative.as_posix())
    return selected


def source_names(root):
    """预编译发行的源码闭包；不放宽原源码自解包器的容量或路径检查。"""
    root = Path(root)
    selected = set(names(root)) | set(EXTRA) | set(TARGET_ROOT_FILES)
    selected.update(target_source_names(root))
    return sorted(selected)


def copy_install(source, target):
    """安装文件成为普通文件，合法链接只可解引用到同一构建工作区。"""
    source = source.resolve()
    for path in sorted(source.rglob('*')):
        rel = path.relative_to(source)
        if '__pycache__' in rel.parts or path.suffix == '.pyc':
            continue
        if path.is_symlink() and path.is_dir():
            raise ValueError('directory symlink is not a portable install item: ' + str(path))
        if path.is_dir():
            continue
        if not path.is_file() or not path.resolve().is_relative_to(source.parent):
            raise ValueError('install item is not a bounded ordinary file: ' + str(path))
        dest = target / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, dest)
        dest.chmod(0o755 if os.access(path, os.X_OK) else 0o644)
        verify_elf(dest)


def validate_evidence(entry, evidence, candidate):
    """构建证据精确覆盖原生/目标源码与安装产物；旧无target发行仍可严格复验。"""
    if (evidence.get('control_candidate_commit') != candidate
            or evidence.get('architecture') != 'aarch64'
            or evidence.get('ubuntu') != '22.04' or evidence.get('ros_distro') != 'humble'):
        raise ValueError('build evidence identity mismatch')
    sources = {p.relative_to(entry).as_posix()
               for p in (entry / 'native/src/jikl_fc01_system').rglob('*') if p.is_file()}
    sources.update('native/tools/' + name for name in
                   ('vision_odometry_adapter.py', 'velocity_command_diagnostic.py',
                    'vision_state_validation.py', 'vision_resource_budget.py'))
    installed = {p.relative_to(entry / 'install').as_posix()
                 for p in (entry / 'install').rglob('*') if p.is_file()}
    if not installed or not {'lib/jikl_fc01_system/fc01_system_node',
                             'lib/jikl_fc01_system/fc01_config_check'} <= installed:
        raise ValueError('build evidence lacks controller/checker')
    checks = [('native_source_sha256', sources, entry),
              ('install_sha256', installed, entry / 'install')]
    if (entry / 'target/src').exists():
        targets = target_source_names(entry)
        checks.append(('target_source_sha256', targets, entry))
        if not TARGET_INSTALL_FILES <= installed:
            raise ValueError('build evidence lacks target runtime: '
                             + repr(sorted(TARGET_INSTALL_FILES - installed)))
        # Python业务模块通常位于lib/python3.10/site-packages；也允许合法merged
        # local/dist-packages布局，但必须唯一且与包内源码同字节，不选旧SDK副本。
        for source in sorted(targets):
            parts = Path(source).parts
            if len(parts) >= 5 and parts[3] == parts[2] and source.endswith('.py'):
                suffix = '/'.join(parts[3:])
                matches = [name for name in installed if name.endswith('/' + suffix)]
                if len(matches) != 1 or sha(entry / source) != sha(entry / 'install' / matches[0]):
                    raise ValueError('installed target Python differs or is ambiguous: ' + source)
        # 源码、现场可复制模板与实际launch/config必须一致，防止只更换ELF却带旧任务配置。
        for source in sorted(targets):
            parts = Path(source).parts
            if len(parts) >= 5 and parts[3] in ('config', 'launch'):
                relative = '/'.join(('share', parts[2], *parts[3:]))
                if relative not in installed or sha(entry / source) != sha(entry / 'install' / relative):
                    raise ValueError('installed target configuration differs: ' + source)
        python_support = [name for name in installed if
                          '/jikl_fc01_system/jikl_fc01_system_s__rosidl_typesupport_c.' in name
                          and name.endswith('-aarch64-linux-gnu.so')]
        if len(python_support) != 1:
            raise ValueError('target runtime lacks unique ARM64 Python message typesupport')
    for key, expected, prefix in checks:
        values = evidence.get(key)
        if not isinstance(values, dict) or set(values) != expected:
            raise ValueError('build evidence coverage differs: ' + key)
        for name, digest in values.items():
            if (not safe_name(name) or not re.fullmatch(r'[0-9a-f]{64}', str(digest))
                    or sha(prefix / name) != digest):
                raise ValueError('build evidence bytes differ: ' + name)


def write_installer(installer_source, archive_sha, output):
    """独立安装器固定包摘要；维护修复可复用已验证的原生包，无须重编飞控。"""
    pin_target = "parser.add_argument('--expected-sha256')"
    if installer_source.count(pin_target) != 1:
        raise ValueError('standalone installer SHA pin contract changed')
    installer_source = installer_source.replace(
        pin_target, "parser.add_argument('--expected-sha256', default=" + repr(archive_sha)
        + ', choices=[' + repr(archive_sha) + '])')
    script = ('#!/usr/bin/env bash\n# 维护安装器：只接受相配的包摘要，以普通用户运行。\n'
              'set -euo pipefail\nexport PYTHONDONTWRITEBYTECODE=1\n'
              'export FC01_PREBUILT_INSTALLER_PATH="$0"\n'
              '/usr/bin/python3 - "$@" <<\'FC01_PREBUILT_INSTALL_PY\'\n'
              + installer_source + '\nFC01_PREBUILT_INSTALL_PY\n')
    output.write_text(script)
    output.chmod(0o755)
    Path(str(output) + '.sha256').write_text(sha(output) + '  ' + output.name + '\n')


def reuse_archive(args):
    """严格读回既有ARM64发行，只交付新安装器，不更动既有tar及其源码身份。"""
    commit = git('rev-parse', 'HEAD').decode().strip()
    archive = args.reuse_archive.resolve()
    archive_sha = sha(archive)
    with tempfile.TemporaryDirectory(prefix='fc01-reuse-readback-') as temporary:
        _, manifest = extract_checked(archive, archive_sha, Path(temporary))
    args.output.mkdir(parents=True, exist_ok=True)
    if not re.fullmatch(r'[A-Za-z0-9_.-]+\.sh', args.installer_name):
        raise ValueError('invalid installer filename')
    output = args.output / args.installer_name
    source = git('show', commit + ':navigation_prebuilt_install.py').decode()
    write_installer(source, archive_sha, output)
    result = dict(archive=str(archive), archive_sha256=archive_sha, archive_bytes=archive.stat().st_size,
                  archive_source_commit=manifest['source_commit'],
                  control_candidate_commit=manifest['control_candidate_commit'],
                  version=manifest['version'], installer=str(output), installer_sha256=sha(output),
                  installer_source_commit=commit, installer_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  payload_files=len(manifest['payload_sha256'])+1, reused_archive_unchanged=True)
    (args.output / 'DELIVERY.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


def build(args):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', args.version):
        raise ValueError('invalid release version')
    commit = git('rev-parse', 'HEAD').decode().strip()
    candidate = git('rev-parse', args.control_candidate + '^{commit}').decode().strip()
    # 只归档提交的字节；用户其他未跟踪文件和历史证据不进入运行载荷。
    with tempfile.TemporaryDirectory(prefix='fc01-arm64-release-') as temporary:
        stage = Path(temporary)
        source = stage / 'source'
        source.mkdir()
        with tarfile.open(fileobj=io.BytesIO(git('archive', commit))) as archive:
            archive.extractall(source)
        top = 'FC01_OrangePi_' + args.version + '_arm64'
        entry = stage / top
        entry.mkdir()
        selected = source_names(source)
        for name in selected:
            original = source / name
            if not original.is_file():
                raise ValueError('missing committed runtime input: ' + name)
            if name.startswith(('native/src/', 'native/tools/', 'target/src/')):
                baseline = git('show', candidate + ':' + name)
                if baseline != original.read_bytes():
                    raise ValueError('control candidate bytes changed: ' + name)
            dest = entry / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original, dest)
        copy_install(args.install_tree, entry / 'install')
        evidence = json.loads(args.build_evidence.read_text())
        validate_evidence(entry, evidence, candidate)
        (entry / 'ARM64_BUILD.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + '\n')
        manifest = dict(version=args.version, source_commit=commit,
                        control_candidate_commit=candidate, architecture='aarch64',
                        ubuntu='22.04', ros_distro='humble',
                        bundled_runtime_dependencies='install/RUNTIME_DEPENDENCIES.json',
                        controller_relative='install/lib/jikl_fc01_system/fc01_system_node',
                        system_dependencies=['ROS 2 Humble rclcpp/rclpy/rmw_fastrtps_cpp',
                                             'python3-yaml', 'psmisc', 'systemd', 'sudo',
                                             'Ubuntu 22.04 ARM64 glibc/libstdc++'],
                        payload_sha256={p.relative_to(entry).as_posix(): sha(p)
                                        for p in sorted(entry.rglob('*')) if p.is_file()})
        (entry / 'ARM64_RELEASE.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        args.output.mkdir(parents=True, exist_ok=True)
        output = args.output / (top + '.tar.gz')
        # 稳定元数据，避免打包用户名/临时目录和时间影响交付身份。
        with output.open('wb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', mtime=0, filename='') as compressed:
            with tarfile.open(fileobj=compressed, mode='w') as archive:
                for p in sorted(entry.rglob('*')):
                    if not p.is_file():
                        continue
                    info = tarfile.TarInfo(top + '/' + p.relative_to(entry).as_posix())
                    info.size = p.stat().st_size
                    info.mode = 0o755 if os.access(p, os.X_OK) else 0o644
                    with p.open('rb') as stream:
                        archive.addfile(info, stream)
        archive_sha = sha(output)
        with tempfile.TemporaryDirectory(prefix='fc01-arm64-readback-') as checkdir:
            _, readback = extract_checked(output, archive_sha, Path(checkdir))
            if readback != manifest:
                raise ValueError('archive manifest readback differs')
        installer_source = (source / 'navigation_prebuilt_install.py').read_text()
        install_script = args.output / 'install_fc01.sh'
        write_installer(installer_source, archive_sha, install_script)
        Path(str(output) + '.sha256').write_text(archive_sha + '  ' + output.name + '\n')
        result = dict(archive=str(output), bytes=output.stat().st_size, sha256=archive_sha,
                      installer=str(install_script), installer_sha256=sha(install_script),
                      source_commit=commit, control_candidate_commit=candidate,
                      files=len(manifest['payload_sha256']) + 1)
        (args.output / 'DELIVERY.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-tree', type=Path)
    parser.add_argument('--build-evidence', type=Path)
    parser.add_argument('--version')
    parser.add_argument('--reuse-archive', type=Path)
    parser.add_argument('--installer-name', default='install_fc01.sh')
    parser.add_argument('--control-candidate', default='adc55d0b7c67e80bf9d77b1082208c898f95b756')
    parser.add_argument('--output', type=Path, default=ROOT / 'dist/arm64')
    args = parser.parse_args()
    if args.reuse_archive:
        if args.install_tree or args.build_evidence or args.version:
            parser.error('--reuse-archive cannot be combined with build inputs')
        reuse_archive(args)
    else:
        if not all((args.install_tree, args.build_evidence, args.version)):
            parser.error('build requires --install-tree, --build-evidence and --version')
        build(args)


if __name__ == '__main__':
    main()

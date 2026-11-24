#!/usr/bin/env python3
"""FC01 预编译 ARM64 发行安装事务；不编译、不下载、不解锁，不强杀控制器。

包内源码/运行文件不可变；deployment 是本机继承配置和事务记录。唯一允许的服务变更
是受管 unit 的专用 drop-in；失败时先等待新控制器安全退出，再还原原配置/运行状态。
维护安装只校验本机软件和进程，不以飞机反馈作为部署成功条件。
现场必须按约定落地、拆桨、飞控断电；飞行准入继续由未修改的原生控制器判定。
"""
from __future__ import annotations

import argparse
import ctypes
from datetime import datetime, timezone
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import pwd
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import time

SERVICE = 'fc01-navigation.service'
MARKER = '# FC01_RC_NAVIGATION_MANUAL_ARM_20261028_V1'
DROPIN = 'zzzz-fc01-prebuilt.conf'
MANIFEST = 'ARM64_RELEASE.json'
MAX_ARCHIVE = 1024 * 1024 * 1024
MAX_TOTAL = 4 * 1024 * 1024 * 1024
MAX_FILES = 30000
# 安装验收需等完整payload/type support/ldd预检；与飞控反馈TTL和控制循环时限无关。
# 完整视觉运行库在ARM64 TCG的单次预检已超过3分22秒；给软件检查有界的
# 10分钟预算。此值不改变飞机反馈时效、原生启动保护或任何控制循环时限。
STARTUP_ACCEPTANCE_SECONDS = 600
ENV_KEYS = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'ROS_DISCOVERY_SERVER', 'ROS_STATIC_PEERS', 'CYCLONEDDS_URI',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
            'RMW_FASTRTPS_USE_QOS_FROM_XML', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'ROS_SECURITY_ENABLE', 'ROS_SECURITY_STRATEGY', 'ROS_SECURITY_KEYSTORE')


class InstallError(RuntimeError):
    """显式失败；不会被转换成“已成功切换”。"""


class StopIncomplete(InstallError):
    """服务所属的旧进程尚未全部退出；禁止继续启动或误报已恢复。"""


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(name, 0o600)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def safe_name(name):
    p = PurePosixPath(name)
    return (bool(name) and not p.is_absolute() and '..' not in p.parts
            and '\\' not in name and name == p.as_posix())


def safe_directory(path, uid):
    """只接受本用户独占的目录祖先，禁止链接把发行写到另一位置。"""
    path = Path(path).absolute()
    for item in (path, *path.parents):
        if item.is_symlink():
            raise InstallError('安装路径包含符号链接：' + str(item))
        if item.exists() and not item.is_dir():
            raise InstallError('安装路径被文件占用：' + str(item))
    if path.exists() and (path.stat().st_uid != uid or path.stat().st_mode & 0o022):
        raise InstallError('安装目录所有者/权限不安全：' + str(path))


def verify_elf(path):
    with Path(path).open('rb') as stream:
        data = stream.read(64)
    if data[:4] != b'\x7fELF':
        return False
    if (len(data) < 64 or data[:6] != b'\x7fELF\x02\x01'
            or int.from_bytes(data[18:20], 'little') != 183):
        raise InstallError('包内存在非 AArch64 ELF：' + str(path))
    return True


def verify_tree(entry, manifest):
    if not isinstance(manifest,dict):
        raise InstallError('ARM64 发行清单不是对象')
    files = manifest.get('payload_sha256')
    if (not isinstance(files, dict) or not files
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', str(manifest.get('version', '')))
            or not re.fullmatch(r'[0-9a-f]{40}', str(manifest.get('source_commit', '')))
            or manifest.get('architecture') != 'aarch64' or manifest.get('ubuntu') != '22.04'
            or manifest.get('ros_distro') != 'humble'):
        raise InstallError('ARM64 发行清单身份不完整')
    actual = set()
    for p in entry.rglob('*'):
        relative = p.relative_to(entry).as_posix()
        if p.is_symlink() or (not p.is_file() and not p.is_dir()):
            raise InstallError('发行树包含特殊/链接文件：' + relative)
        if p.is_file() and relative != MANIFEST and not relative.startswith('deployment/'):
            actual.add(relative)
    if set(files) != actual:
        missing = set(files) - actual
        extra = actual - set(files)
        # 保持精确集合校验；仅给出有界差异，不能忽略可执行的 Python 缓存。
        raise InstallError('发行文件集合与清单不一致；缺少(%d)=%r；多出(%d)=%r' % (
            len(missing), sorted(missing)[:30], len(extra), sorted(extra)[:30]))
    for name, expected in files.items():
        if (not safe_name(name) or name == MANIFEST or name.startswith('deployment/')
                or not re.fullmatch(r'[0-9a-f]{64}', str(expected))
                or digest(entry / name) != expected):
            raise InstallError('发行内容摘要不一致：' + str(name))
        verify_elf(entry / name)
    relative = manifest.get('controller_relative')
    if not isinstance(relative, str) or relative not in files or not verify_elf(entry / relative):
        raise InstallError('缺少清单指定的 AArch64 控制器')
    if not os.access(entry / relative, os.X_OK):
        raise InstallError('控制器缺少执行权限')
    for name in ('prebuilt_boot.sh', 'navigation_prebuilt_service.py'):
        if name not in files:
            raise InstallError('缺少预编译服务入口：' + name)
    return entry / relative


def extract_checked(archive_path, expected, workspace):
    """严格核对压缩包和完整展开文件集；链接一律拒绝，发行器应解引用合法安装链接。"""
    archive_path = Path(archive_path)
    if (archive_path.is_symlink() or not archive_path.is_file()
            or archive_path.stat().st_size > MAX_ARCHIVE
            or not re.fullmatch(r'[0-9a-f]{64}', expected or '')
            or digest(archive_path) != expected):
        raise InstallError('安装包 SHA-256 不匹配或文件无效')
    names = set()
    top = None
    total = 0
    with tarfile.open(archive_path, 'r:gz') as archive:
        for member in archive:
            name = member.name.rstrip('/')
            if (not safe_name(name) or name in names or member.issym() or member.islnk()
                    or not (member.isfile() or member.isdir())):
                raise InstallError('不安全的归档成员：' + name)
            names.add(name)
            if len(names) > MAX_FILES:
                raise InstallError('归档成员数超限')
            parts = PurePosixPath(name).parts
            if top is None:
                top = parts[0]
            if (parts[0] != top or not re.fullmatch(r'FC01_OrangePi_[A-Za-z0-9_.-]+_arm64', top)
                    or (member.isfile() and len(parts) < 2) or 'deployment' in parts[1:2]):
                raise InstallError('归档顶层或本机配置目录无效')
            dest = workspace.joinpath(*parts)
            if member.isdir():
                dest.mkdir(mode=0o700, parents=True, exist_ok=True)
                continue
            total += member.size
            if member.size < 0 or member.size > MAX_ARCHIVE or total > MAX_TOTAL:
                raise InstallError('展开体积超限')
            dest.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            source = archive.extractfile(member)
            if source is None:
                raise InstallError('无法读取归档文件')
            with source, dest.open('xb') as output:
                copied = 0
                while True:
                    chunk = source.read(1024 * 1024)
                    if not chunk:
                        break
                    copied += len(chunk)
                    if copied > member.size:
                        raise InstallError('归档成员长度超限')
                    output.write(chunk)
            if copied != member.size:
                raise InstallError('归档成员长度不符')
            dest.chmod(0o700 if member.mode & 0o111 else 0o600)
    if top is None:
        raise InstallError('安装包为空')
    entry = workspace / top
    for path in [entry, *(p for p in entry.rglob('*') if p.is_dir())]:
        path.chmod(0o700)
    try:
        manifest = json.loads((entry / MANIFEST).read_text())
    except (OSError, ValueError) as exc:
        raise InstallError('缺少有效发行清单') from exc
    verify_tree(entry, manifest)
    return entry, manifest


def publish_directory(source, target):
    """Linux renameat2 NOREPLACE：即使外部并发创建目录也不覆盖。"""
    libc = ctypes.CDLL(None, use_errno=True)
    rename = getattr(libc, 'renameat2', None)
    if rename is None:
        raise InstallError('系统缺少 renameat2，未覆盖任何目录')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    if rename(-100, os.fsencode(source), -100, os.fsencode(target), 1):
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise InstallError('版本目录被并发创建，未覆盖')
        raise OSError(number, os.strerror(number), str(target))


def systemd_quote(value):
    if any(c in str(value) for c in '\n\r\x00'):
        raise InstallError('systemd 路径含控制字符')
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


def systemd_directory(value):
    """WorkingDirectory是单个路径值，不使用ExecStart的shell词法双引号。

    systemd249会把这里的双引号保留为路径字符；百分号仍需转义以免展开specifier。
    拒绝控制字/反斜杠和边缘空白，避免unit续行/转义改变实际目录。
    """
    text=str(value)
    if (not Path(text).is_absolute() or text!=text.strip() or '\\' in text
            or any(ord(c)<32 or ord(c)==127 for c in text)):
        raise InstallError('systemd工作目录必须是无控制字的明确绝对路径')
    return text.replace('%','%%')


class Backend:
    """唯一系统副作用边界；测试替身只替换此对象，不向生产CLI开放跳过开关。"""
    # 本机软件启动后再观察一秒，避免把刚出现即崩溃的进程判为安装成功。
    acceptance_stability_seconds = 1.0

    def __init__(self, home=None):
        self.uid = os.getuid()
        if self.uid == 0:
            raise InstallError('请以 orangepi 普通用户运行，不要 sudo 整个安装器；写服务时会请求 sudo')
        account = pwd.getpwuid(self.uid)
        self.user = account.pw_name
        self.home = Path(home or account.pw_dir).resolve()
        self.units = Path('/etc/systemd/system')
        self.proc = Path('/proc')

    def run(self, argv, **kw):
        return subprocess.run(argv, check=True, text=True, capture_output=True, **kw)

    def run_progress(self, argv, *, timeout):
        # 仅软件检查走继承stdout，进度立即可见；systemd/proc查询仍捕获结构化结果。
        return subprocess.run(argv, check=True, text=True, timeout=timeout)

    def privileged(self, argv):
        return self.run(['sudo', '-n', *map(str, argv)])

    def authenticate(self):
        subprocess.run(['sudo', '-v'], check=True)

    def check_host(self):
        if platform.machine().lower() not in ('aarch64', 'arm64'):
            raise InstallError('仅支持实际 ARM64 主机，未变更服务')
        release = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
        if release.get('ID', '').strip('"') != 'ubuntu' or release.get('VERSION_ID', '').strip('"') != '22.04':
            raise InstallError('仅支持 Ubuntu 22.04，未变更服务')
        for name in ('/opt/ros/humble/setup.bash', '/usr/bin/systemctl', '/usr/bin/python3'):
            if not Path(name).is_file():
                raise InstallError('缺少目标依赖：' + name)
        for name in ('sudo', 'fuser', 'systemd-analyze'):
            if not shutil.which(name):
                raise InstallError('缺少本机工具：' + name)
        import yaml  # noqa: F401

    def service(self):
        props = 'LoadState,FragmentPath,DropInPaths,MainPID,ActiveState,SubState,UnitFileState,WorkingDirectory,User,Environment,EnvironmentFiles,ExecStart,ControlGroup,Result,ExecMainStatus,TimeoutStartUSec'
        text = self.run(['/usr/bin/systemctl', 'show', SERVICE, '--property=' + props]).stdout
        return dict(line.split('=', 1) for line in text.splitlines() if '=' in line)

    def process(self, pid):
        root = self.proc / str(pid)
        # /proc/PID/stat 的 comm 可包含空格和括号；从最后一个右括号之后取字段。
        # start_ticks 绑定同一进程寿命，避免等待停服时误把复用的PID当成旧进程。
        fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=pid, uid=root.stat().st_uid,
                    argv=[x.decode() for x in (root / 'cmdline').read_bytes().split(b'\0') if x],
                    exe=str((root / 'exe').resolve(strict=True)), cwd=str((root / 'cwd').resolve(strict=True)),
                    sha256=digest(root / 'exe'), start_ticks=int(fields[19]))

    def owned_processes(self, service):
        """只跟踪本服务拥有的监督器和FC01节点；复用的Agent不属于停止范围。

        优先用实际祖先链，另用systemd给出的cgroup覆盖父进程先退出后的孤儿。
        不依据模糊名称向全机pkill，也不停止服务外的相机/ROS程序。
        """
        main = int(service.get('MainPID') or 0)
        group = service.get('ControlGroup')
        roles = {'fc01_system_node', 'fc01_vision_node', 'fc01_target_interface_node',
                 'mission', 'px4_state_bridge'}
        found = []
        for root in self.proc.iterdir():
            if not root.name.isdecimal():
                continue
            try:
                pid = int(root.name)
                argv = [x.decode(errors='replace') for x in (root/'cmdline').read_bytes().split(b'\0') if x]
                role = any(Path(arg).name in roles for arg in argv[:2])
                launch = (any(Path(arg).name in ('ros2', 'ros2launch') for arg in argv[:2])
                          and 'launch' in argv and any(arg.endswith('.launch.py') for arg in argv))
                if pid != main and not role and not launch:
                    continue
                in_group = bool(group and group.startswith('/') and group != '/' and any(
                    line.split(':', 2)[-1] == group or line.split(':', 2)[-1].startswith(group+'/')
                    for line in (root/'cgroup').read_text().splitlines()))
                if not self.descendant(pid, main) and not in_group:
                    continue
                uid=root.stat().st_uid
                if uid != self.uid:
                    raise InstallError('服务所属FC01进程用户不一致，未切换')
                fields=(root/'stat').read_text().rsplit(')',1)[1].split()
                if fields[0]!='Z':
                    found.append(dict(pid=pid,uid=uid,argv=argv,
                                      exe=str((root/'exe').resolve(strict=True)),
                                      start_ticks=int(fields[19])))
            except (FileNotFoundError, ProcessLookupError):
                continue
        return found

    def same_process_alive(self, row):
        """只观察捕获的旧进程；僵尸已无可运行代码，不向它或新复用的PID发信号。"""
        try:
            root = self.proc / str(row['pid'])
            fields = (root/'stat').read_text().rsplit(')', 1)[1].split()
            return fields[0] != 'Z' and int(fields[19]) == row['start_ticks']
        except (FileNotFoundError, ProcessLookupError):
            return False

    def software_snapshot(self):
        # 原生console只接受一个客户端，由监督器持有。读取其只读IPC状态桥，
        # 不抢占会话、不发送控制命令；后续验收只查看身份及原生状态的推进。
        return json.loads((self.home/'FC01_navigation/status.json').read_text())

    def controllers(self):
        found = []
        for item in self.proc.iterdir():
            if not item.name.isdecimal():
                continue
            try:
                cmd = (item / 'cmdline').read_bytes().split(b'\0')
                if any(Path(x.decode(errors='replace')).name == 'fc01_system_node' for x in cmd[:2]):
                    found.append(self.process(int(item.name)))
            except (FileNotFoundError, ProcessLookupError):
                continue
        return found

    def descendant(self, pid, main):
        seen = set()
        while pid > 1 and pid not in seen:
            if pid == main:
                return True
            seen.add(pid)
            try:
                pid = int(next(line.split()[1] for line in (self.proc / str(pid) / 'status').read_text().splitlines() if line.startswith('PPid:')))
            except (OSError, ValueError, StopIteration):
                return False
        return False

    def environment(self, pid):
        return dict(item.decode().split('=', 1) for item in (self.proc / str(pid) / 'environ').read_bytes().split(b'\0') if b'=' in item)

    def boot_id(self):
        return (self.proc / 'sys/kernel/random/boot_id').read_text().strip()

    def dependencies(self, entry, manifest, environment):
        """先在目标Humble/SDK环境核对动态依赖和配置检查器，尚不启动控制器。"""
        # 包自带所需px4_msgs，本机旧SDK仅保留为来源信息，不能让旧typesupport抢先加载。
        script = '''set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
exec /usr/bin/python3 - "$1" <<'CHECK'
import hashlib,json,subprocess,sys
from pathlib import Path
from rclpy.type_support import check_for_type_support
from px4_msgs.msg import VehicleStatus,VehicleLocalPosition,VehicleAttitude,ManualControlSwitches
from jikl_fc01_system.msg import VisionState,VisionHealth,NavigationCommand,VelocityCommand,AttitudeThrustPreview
for cls in (VehicleStatus,VehicleLocalPosition,VehicleAttitude,ManualControlSwitches,VisionState,VisionHealth,NavigationCommand,VelocityCommand,AttitudeThrustPreview):check_for_type_support(cls)
entry=Path(sys.argv[1])
# 新视觉包须装齐统一类型和运行模块；旧维护包没有target业务包，仍按原合同验收。
if (entry/'install/share/fc01_mission/package.xml').is_file():
 from px4_msgs.msg import VehicleAngularVelocity
 from jikl_fc01_system.msg import (VelocityCommandV2,TargetObservation,PlatformState,NativeFlightState,NativeFlightSnapshot,HorizontalVelocityCommand)
 for cls in (VehicleAngularVelocity,VelocityCommandV2,TargetObservation,PlatformState,NativeFlightState,NativeFlightSnapshot,HorizontalVelocityCommand):check_for_type_support(cls)
 import numpy,fc01_mission.mission_node,fc01_mission.px4_state_bridge
files=json.loads((entry/'ARM64_RELEASE.json').read_text())['payload_sha256']
dependency_checks=set()
# 本次安装随后必经config_check完整ldd；仅在同一事务显式选择此模式时避免重复。
ldd_required=__import__('os').environ.get('FC01_INSTALL_LOCAL_CHECK')!='elf-and-types'
for path in entry.rglob('*'):
 if not path.is_file():continue
 with path.open('rb') as stream:header=stream.read(64)
 if header[:4]==b'\\x7fELF':
  # 每个路径都验证架构和实际字节；仅相同目录、相同SHA的成功ldd可复用。
  if len(header)<64 or header[:6]!=b'\\x7fELF\\x02\\x01' or int.from_bytes(header[18:20],'little')!=183:raise SystemExit('non-AArch64 ELF '+str(path))
  value=hashlib.sha256()
  with path.open('rb') as stream:
   for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
  actual=value.hexdigest()
  if files.get(path.relative_to(entry).as_posix())!=actual:raise SystemExit('ELF hash mismatch '+str(path))
  if not ldd_required:continue
  key=(path.parent.resolve(),actual)
  if key in dependency_checks:continue
  result=subprocess.run(['ldd',str(path)],capture_output=True,text=True)
  report=result.stdout+result.stderr
  if result.returncode or 'not found' in report:raise SystemExit('unresolved runtime dependency '+str(path)+'\\n'+report)
  dependency_checks.add(key)
print('FC01_PREBUILT_RUNTIME_DEPENDENCIES_OK')
CHECK
'''
        if getattr(self, '_installation_checks_follow', False):
            script = 'export FC01_INSTALL_LOCAL_CHECK=elf-and-types\n' + script
        else:
            script = 'unset FC01_INSTALL_LOCAL_CHECK\n' + script
        print('FC01：检查统一消息、Python模块和全部ELF架构；完整动态依赖随后随配置统一核对。',flush=True)
        result = self.run(['/bin/bash', '-c', script, 'fc01-prebuilt-check', str(entry)]).stdout
        print(result,flush=True)
        return result

    def config_check(self, entry, manifest, runtime, environment):
        binary = entry / manifest['controller_relative']
        checker = binary.with_name('fc01_config_check')
        if not checker.is_file() or not verify_elf(checker):
            raise InstallError('缺少 AArch64 配置检查器')
        print('FC01：开始完整服务软件预检（文件/动态依赖/参数），旧服务此时保持原状态。',flush=True)
        self.run_progress(['/bin/bash', str(entry/'prebuilt_boot.sh'), 'check', '--config', str(runtime)], timeout=600)

    def systemd_verify(self, unit):
        # 验证合成完整unit：drop-in自身没有[Unit]/[Install]，不把它误作独立服务运行。
        self.run(['/usr/bin/systemd-analyze', 'verify', str(unit)])

    def write_root(self, path, data, mode=0o644):
        with tempfile.NamedTemporaryFile() as source:
            source.write(data)
            source.flush()
            self.privileged(['/usr/bin/install', '-D', '-m', format(mode, '04o'), source.name, str(path)])

    def remove_root(self, path):
        self.privileged(['/usr/bin/rm', '-f', '--', str(path)])

    def reload(self):
        self.privileged(['/usr/bin/systemctl', 'daemon-reload'])

    def stop(self):
        self.privileged(['/usr/bin/systemctl', 'stop', '--no-block', SERVICE])

    def start(self):
        # 由verify_active提供有进度、有限时的软件等待；不让systemctl本身无声阻塞。
        self.privileged(['/usr/bin/systemctl', 'start', '--no-block', SERVICE])

    def sleep(self, seconds):
        time.sleep(seconds)


def managed_controller(backend, service=None):
    """只核对本机服务/进程归属；无控制器是旧服务已停止或监督器待机的合法状态。"""
    service = service or backend.service()
    rows = backend.controllers()
    main = int(service.get('MainPID') or 0)
    if main and service.get('User') not in (backend.user, str(backend.uid)):
        raise InstallError('服务用户与安装用户不一致')
    if len(rows) > 1:
        raise InstallError('发现多个控制器，未切换服务')
    if rows:
        controller = rows[0]
        if (not main or service.get('User') not in (backend.user, str(backend.uid))
                or controller['uid'] != backend.uid
                or not backend.descendant(controller['pid'], main)):
            raise InstallError('发现不属于当前服务/用户的控制器，未停止它')
        return controller
    return None


def read_runtime(path):
    """完整保留本机既有参数；维护安装不读取飞机状态，也不把默认参数覆盖到现场。"""
    path = Path(path)
    if not path.is_absolute() or any(p.is_symlink() for p in (path, *path.parents)) or not path.is_file():
        raise InstallError('缺少可验证的本机 runtime.yaml：' + str(path))
    import yaml
    config = yaml.safe_load(path.read_text())
    try:
        params = config['fc01_system']['ros__parameters']
    except (KeyError, TypeError) as exc:
        raise InstallError('本机参数结构不完整') from exc
    if not isinstance(params, dict) or params.get('arm_enabled') is not False:
        raise InstallError('只继承人工RC解锁合同，不扩大程序解锁权限')
    # 视觉开关属于明确的本机配置。安装不因缺相机/飞机反馈卡住，也不擅自打开它；
    # 启用时只接受已配对的生产水平路线，完整数值/限幅由包内原生检查器再校验。
    visual = params.get('visual_navigation_enabled')
    horizontal = params.get('px4_horizontal_enabled', False)
    if type(visual) is not bool or type(horizontal) is not bool or visual != horizontal:
        raise InstallError('视觉与PX4水平路线必须是明确且配对的布尔配置')
    if visual:
        height = params.get('rc_target_height')
        if (type(height) not in (int, float) or not math.isfinite(height)
                or abs(height - 1.5) > 1e-6
                or type(params.get('rc_navigation_mode_slot')) is not int
                or params['rc_navigation_mode_slot'] != 1):
            raise InstallError('视觉首测只继承1.5m目标和人工RC导航槽位1')
    mode = params.get('fc01_runtime_mode')
    if mode != 'flight-test':
        raise InstallError('当前预编译入口需既有人工RC flight-test 配置；未自动提升诊断权限')
    return path, config, mode


def current_runtime(controller):
    values = [controller['argv'][i+1] for i, arg in enumerate(controller['argv'][:-1])
              if arg == '--params-file']
    if len(values) != 1:
        raise InstallError('无法确认唯一生效的 --params-file')
    return read_runtime(values[0])


def persisted_environment(service):
    """读取systemd已解析的Environment和简单EnvironmentFile，不执行shell或展开命令。"""
    env = dict(word.split('=', 1) for word in shlex.split(service.get('Environment', '')) if '=' in word)
    for name in re.findall(r'(/[^ ;{}]+)(?:\s+\(ignore_errors=[^)]*\))?', service.get('EnvironmentFiles', '')):
        path = Path(name)
        if path.is_symlink() or not path.is_file():
            raise InstallError('环境文件来源无效：' + name)
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#') or line.startswith(';'):
                continue
            if '=' not in line or line.endswith('\\'):
                raise InstallError('不支持的环境文件语法，未猜测：' + name)
            key, value = line.split('=', 1)
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key):
                raise InstallError('环境文件键无效：' + name)
            parts = shlex.split(value, comments=False)
            env[key] = ' '.join(parts)
    return env


def capture_environment(backend, service, controller):
    cwd = service.get('WorkingDirectory') or ''
    saved = {}
    if cwd and Path(cwd).is_absolute():
        saved_path = Path(cwd) / 'deployment/environment.json'
        if saved_path.is_file() and not saved_path.is_symlink():
            saved = json.loads(saved_path.read_text())
    env = dict(saved.get('environment', {}))
    env.update(persisted_environment(service))
    main = int(service.get('MainPID') or 0)
    if main:
        env.update(backend.environment(main))
    if controller:
        env.update(backend.environment(controller['pid']))
    inherited = {name: env[name] for name in ENV_KEYS if name in env}
    inherited.setdefault('ROS_DOMAIN_ID', '0')
    inherited.setdefault('ROS_LOCALHOST_ONLY', '0')
    if inherited['ROS_DOMAIN_ID'] != '0' or inherited['ROS_LOCALHOST_ONLY'] != '0':
        raise InstallError('现行非域0/非网络DDS不符合既有服务合同；未悄悄改设置')
    agent = saved.get('agent_path') or str(backend.home / 'fc01_agent_build_offline/MicroXRCEAgent')
    if not Path(agent).is_absolute() or not Path(agent).is_file() or not os.access(agent, os.X_OK):
        raise InstallError('缺少本机已安装的Agent可执行文件：' + str(agent))
    sdk = env.get('FC01_PROJECT_ROOT') or saved.get('sdk_root')
    return dict(environment=inherited, sdk_root=sdk, original_entry=cwd,
                agent_path=agent, service_mode=None)


def old_runtime(backend, controller):
    if controller:
        return current_runtime(controller)
    return read_runtime(backend.home / 'FC01_navigation/runtime.yaml')


def local_service_files(backend, service):
    """支持本机首次建立受管unit；缺少现场runtime仍明确失败，不制造飞行参数。"""
    if service.get('LoadState') == 'not-found' or not service.get('FragmentPath'):
        unit = backend.units / SERVICE
        if any(p.is_symlink() for p in (unit, *unit.parents)):
            raise InstallError('初装服务目录包含符号链接')
        if unit.exists() or unit.is_symlink() or int(service.get('MainPID') or 0):
            raise InstallError('未加载的同名服务文件或进程已存在，未覆盖')
        own = backend.units / (SERVICE + '.d') / DROPIN
        if any(p.is_symlink() for p in (own, *own.parents)):
            raise InstallError('初装专用drop-in路径包含符号链接，未覆盖')
        if own.exists():
            raise InstallError('初装发现残留专用drop-in，未覆盖')
        return [dict(path=str(unit), present=False, data=None),
                dict(path=str(own), present=False, data=None)]
    return capture_files(backend, service)


def first_unit(backend, entry):
    return (MARKER + '\n[Unit]\nDescription=FC01 manual RC navigation controller\nAfter=local-fs.target\n'
            '[Service]\nType=simple\nUser=' + backend.user + '\n'
            'Environment=' + systemd_quote('HOME=' + str(backend.home)) + '\n'
            'Environment=ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1\n'
            'ExecStart=/bin/bash ' + systemd_quote(entry/'prebuilt_boot.sh') + '\n'
            'WorkingDirectory=' + systemd_directory(entry) + '\n'
            'KillMode=process\nKillSignal=SIGTERM\nSendSIGKILL=no\nTimeoutStopSec=infinity\n'
            'TimeoutStartSec='+str(STARTUP_ACCEPTANCE_SECONDS)+'s\n'
            'Restart=on-failure\nRestartSec=30s\nUMask=0077\nNoNewPrivileges=true\n'
            '[Install]\nWantedBy=multi-user.target\n').encode()


def capture_files(backend, service):
    unit = Path(service.get('FragmentPath', ''))
    if unit != backend.units / SERVICE or unit.is_symlink() or not unit.is_file() or not unit.read_text().startswith(MARKER+'\n'):
        raise InstallError('当前服务不是已知受管FC01单元，未覆盖')
    if service.get('User') not in (backend.user, str(backend.uid)):
        raise InstallError('当前服务用户与安装用户不一致')
    paths = [unit]
    paths += [Path(p) for p in shlex.split(service.get('DropInPaths', ''))]
    # 环境文件绝不能被新版本路径替换悄悄遗失；先保留实际字节，且拒绝未知/带链接来源。
    envfiles = service.get('EnvironmentFiles', '')
    for name in re.findall(r'(/[^ ;{}]+)(?:\s+\(ignore_errors=[^)]*\))?', envfiles):
        paths.append(Path(name))
    result = []
    for path in paths:
        if (not path.is_absolute() or any(p.is_symlink() for p in (path,*path.parents))
                or not path.is_file()):
            raise InstallError('服务配置来源无效：' + str(path))
        data = path.read_bytes()
        result.append(dict(path=str(path), mode=path.stat().st_mode & 0o777,
                           sha256=hashlib.sha256(data).hexdigest(), data=data))
    own = backend.units / (SERVICE + '.d') / DROPIN
    if any(p.is_symlink() for p in (own,*own.parents)):
        raise InstallError('事务drop-in路径包含链接，未覆盖')
    if own not in [Path(item['path']) for item in result]:
        result.append(dict(path=str(own), present=False, data=None))
    return result


def snapshot(backend, entry, archive_hash, service, controller, runtime, environment, files, agent=None, agent_owned=False):
    folder = backend.home / 'FC01_navigation/prebuilt-backups'
    safe_directory(folder, backend.uid)
    folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    backup = folder / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '-' + str(os.getpid()))
    backup.mkdir(mode=0o700)
    records=[]
    for i,item in enumerate(files):
        row={key:value for key,value in item.items() if key!='data'}
        if item.get('data') is not None:
            name=f'file-{i:03d}'
            (backup/name).write_bytes(item['data']);(backup/name).chmod(0o600)
            row.update(present=True, backup=name)
        records.append(row)
    original_runtime = Path(runtime)
    (backup/'runtime.yaml').write_bytes(original_runtime.read_bytes())
    manifest=dict(schema=1,entry=str(entry),archive_sha256=archive_hash,service=service,
                  controller=controller,runtime_path=str(original_runtime),
                  runtime_sha256=digest(original_runtime),runtime_mode=original_runtime.stat().st_mode&0o777,
                  environment=environment,files=records,agent=agent,agent_owned=agent_owned,
                  boot_id=backend.boot_id(),phase='prepared')
    atomic_json(backup/'BACKUP.json',manifest)
    return backup,manifest


def write_deployment(entry, config, mode, environment, manifest, backup):
    import yaml
    folder=entry/'deployment'
    if folder.is_symlink():raise InstallError('deployment目录被改为链接，未覆盖')
    folder.mkdir(mode=0o700,exist_ok=True)
    for name in ('runtime.yaml','environment.json','INSTALL_STATE.json'):
        if (folder/name).is_symlink():raise InstallError('本机配置为链接，未覆盖：'+name)
    params=config['fc01_system']['ros__parameters']
    params['fc01_config_path']=str(entry.parents[1]/'FC01_navigation/runtime.yaml')
    params['fc01_config_source_path']=str(folder/'runtime.yaml')
    params['fc01_entry_version']=manifest['version']
    data=yaml.safe_dump(config,sort_keys=False,allow_unicode=True)
    (folder/'runtime.yaml').write_text(data);(folder/'runtime.yaml').chmod(0o600)
    environment=dict(environment, service_mode=mode, mode=mode)
    atomic_json(folder/'environment.json',environment)
    atomic_json(folder/'INSTALL_STATE.json',dict(backup=str(backup),runtime_sha256=digest(folder/'runtime.yaml'),phase='prepared'))



def migrate_vision_configuration(backend, entry, config, old_entry):
    """显式视觉模式升级时搬迁经校验的现场配置；不查询相机、RC或DDS。

    旧目录保留不变，因而回退仍可使用原标定。新目录只接收已验证的文件，
    缺少可追溯现场配置明确在停旧前失败，不能用仓库候选值补造实机标定。
    """
    if config['fc01_system']['ros__parameters'].get('visual_navigation_enabled') is not True:
        return
    tool = entry/'navigation_vision_mode.py'
    if not tool.is_file() or not old_entry or not Path(old_entry).is_absolute():
        raise InstallError('视觉升级缺少可迁移配置工具或原版本目录，未停止旧服务')
    backend.run(['/usr/bin/python3', '-B', str(tool), 'migrate', '--entry', str(entry),
                 '--source-entry', str(old_entry)])


def stop_and_wait(backend, *, timeout=120):
    info=backend.service()
    controllers=backend.controllers()
    owned=backend.owned_processes(info)
    if (info.get('ActiveState') in ('inactive','failed') and int(info.get('MainPID') or 0)==0
            and not controllers and not owned):
        return
    # 维护状态由现场保证。这里只验证软件归属，不要求飞控上电或RC/DDS反馈。
    managed_controller(backend, info)
    if info.get('User') not in (backend.user,str(backend.uid)):
        raise InstallError('无控制器但服务用户未知，未停止它')
    backend.stop()
    began=time.monotonic()
    deadline=began+timeout
    progress=began
    while True:
        info=backend.service()
        # 捕获的身份在父进程退出、cgroup改变后仍有效；新出现的同服务节点也必须退出。
        remaining={row['pid'] for row in owned if backend.same_process_alive(row)}
        remaining.update(row['pid'] for row in backend.owned_processes(info))
        if (info.get('ActiveState') in ('inactive','failed') and int(info.get('MainPID') or 0)==0
                and not backend.controllers() and not remaining):
            print('FC01：旧服务及所属FC01进程已退出（%.1f秒）。'%(time.monotonic()-began),flush=True)
            return
        if time.monotonic()>=deadline:
            raise StopIncomplete('旧服务或所属FC01进程在%.0f秒内尚未退出，未强杀、未启动新版；PID=%s'
                                 %(timeout,sorted(remaining)))
        if time.monotonic()-progress>=10:
            print('FC01：等待旧任务退出，已用%.0f秒；服务=%s，仍存活PID=%s。'
                  %(time.monotonic()-began,info.get('ActiveState'),sorted(remaining)),flush=True)
            progress=time.monotonic()
        backend.sleep(.1)


def software_ipc_stamp(backend, info, controller, *, since):
    """校验监督器收到的当前原生IPC状态，绝不要求飞控、相机或READY。

    10秒是本机软件观察的有界寿命，独立于飞机各路TTL；并要求状态发生在本次
    验收开始之后。调用方再检查源戳持续推进，原样旧文件不能证明软件存活。
    """
    row=backend.software_snapshot()
    state=row.get('state') if isinstance(row,dict) else None
    now=time.monotonic()
    if (not isinstance(state,dict) or row.get('supervisor_pid')!=int(info.get('MainPID') or 0)
            or row.get('boot_id')!=backend.boot_id()
            or any(value!=controller['pid'] for value in
                   (row.get('controller_pid'),row.get('state_controller_pid'),state.get('controller_pid')))
            or row.get('ipc_error') or row.get('controller_returncode') is not None
            or row.get('retry_in_seconds') is not None
            or row.get('status') in ('STARTING','STALE','STOPPING','STOPPED')):
        raise InstallError('等待当前监督器/控制器的只读IPC状态；不要求飞机反馈')
    stamp=state.get('diagnostic_monotonic_s')
    age=row.get('state_age_seconds')
    finite=lambda value:type(value) in (int,float) and math.isfinite(value)
    if (not finite(stamp) or not since<=stamp<=now or now-stamp>10
            or not finite(age) or not 0<=age<=10):
        raise InstallError('等待本次软件验收后推进的原生IPC状态')
    return stamp


def verify_vision_software(backend, entry, info, config, manifest):
    """track安装只验本机辅助软件存活，不要求相机图像或任何飞机反馈。

    launch主进程活着不等于桥/相机/任务都活着。读取实际/proc的可执行路径和
    脚本参数，核实四个子节点均属于本服务的同一个launch进程树；旧JSON不能代替进程。
    """
    params = config['fc01_system']['ros__parameters']
    if (params.get('visual_navigation_enabled') is not True
            or not (entry/'target/src/fc01_mission/package.xml').is_file()):
        return
    value = json.loads((entry/'deployment/vision-runtime.json').read_text())
    if not isinstance(value, dict):
        raise InstallError('track辅助软件状态格式无效')
    stamp = value.get('receive_monotonic_ns')
    now = time.monotonic()*1e9
    main = int(info.get('MainPID') or 0)
    expected_mode = 'position-track' if params.get('px4_takeoff_position_hold') is True else 'track'
    if (type(value.get('schema_version')) is not int or value['schema_version'] != 1
            or value.get('mode') != expected_mode or value.get('enabled') is not True
            or value.get('supervisor_pid') != main or value.get('boot_id') != backend.boot_id()
            or type(stamp) is not int or not 0 < stamp <= now or now-stamp > 3e9
            or value.get('error') or value.get('auxiliary_returncode') is not None):
        raise InstallError('等待本服务新鲜的track辅助软件状态；不需要飞机或相机反馈')
    auxiliary = value.get('auxiliary_pid')
    if type(auxiliary) is not int or auxiliary <= 1 or not backend.descendant(auxiliary, main):
        raise InstallError('track辅助launch不属于本服务的实际进程树')
    launch = backend.process(auxiliary)
    cli = str((entry/'install/bin/ros2').resolve())
    if (launch['uid'] != backend.uid or cli not in launch['argv'][:2]
            or 'horizontal_first_test.launch.py' not in launch['argv']):
        raise InstallError('track辅助launch路径或启动入口不匹配')
    names = ('install/lib/fc01_vision/fc01_vision_node',
             'install/lib/fc01_mission/px4_state_bridge',
             'install/lib/fc01_mission/mission',
             'install/lib/jikl_fc01_system/fc01_target_interface_node')
    expected = {str((entry/name).resolve()): name for name in names}
    found = {name: [] for name in names}
    for root in backend.proc.iterdir():
        if not root.name.isdecimal():
            continue
        try:
            argv = [x.decode(errors='replace') for x in (root/'cmdline').read_bytes().split(b'\0') if x]
            candidates = set(argv[:2]) & expected.keys()
            if not candidates:
                continue
            process = backend.process(int(root.name))
            if process['uid'] != backend.uid:
                raise InstallError('track辅助节点由未知用户运行')
            for path in candidates:
                name = expected[path]
                if not backend.descendant(process['pid'], auxiliary):
                    raise InstallError('track节点存在服务外的重复进程：'+name)
                if digest(entry/name) != manifest['payload_sha256'].get(name):
                    raise InstallError('track节点文件与发行清单不同：'+name)
                if (name.endswith('_node') and
                        (process['exe'] != path or process['sha256'] != digest(entry/name))):
                    raise InstallError('track原生辅助节点实际ELF不匹配：'+name)
                found[name].append(process['pid'])
        except (FileNotFoundError, ProcessLookupError):
            continue
    missing = [name for name, pids in found.items() if len(pids) != 1]
    if missing:
        raise InstallError('等待唯一track辅助软件节点：'+', '.join(missing))


def verify_active(backend, entry, manifest, environment, *, timeout=STARTUP_ACCEPTANCE_SECONDS):
    """软件启动验收：真实unit/唯一进程/路径/摘要/参数/IPC，不读取飞机反馈。"""
    began = time.monotonic()
    deadline = began + timeout
    last = None
    stable_since = None
    stable_identity = None
    first_stamp = None
    progress = began
    binary = entry / manifest['controller_relative']
    binary_hash = digest(binary)
    while True:
        info = {}
        try:
            info = backend.service()
            controller = managed_controller(backend, info)
            if info.get('ActiveState') != 'active' or controller is None:
                raise InstallError('等待新版服务和唯一控制器进程')
            if (Path(controller['exe']) != binary.resolve() or controller['sha256'] != binary_hash
                    or Path(controller['cwd']) != entry.resolve()
                    or str(entry/'prebuilt_boot.sh') not in info.get('ExecStart', '')
                    or Path(info.get('WorkingDirectory', '/')) != entry.resolve()):
                raise InstallError('实际服务/控制器路径或摘要未指向候选版本')
            actual_env = backend.environment(controller['pid'])
            if any(actual_env.get(k) != v for k, v in environment['environment'].items()):
                raise InstallError('实际DDS环境未继承')
            live_path, config, _ = current_runtime(controller)
            expected = __import__('yaml').safe_load((entry/'deployment/runtime.yaml').read_text())
            if config != expected:
                raise InstallError('实际运行参数与继承快照不同')
            if live_path not in (backend.home/'FC01_navigation/runtime.yaml', entry/'deployment/runtime.yaml'):
                raise InstallError('实际控制器使用未知参数路径')
            verify_vision_software(backend, entry, info, config, manifest)
            stamp = software_ipc_stamp(backend, info, controller, since=began)
            if managed_controller(backend, backend.service()) != controller:
                raise InstallError('软件验收期间控制器变化，等待稳定进程')
            identity = (info['MainPID'], controller['pid'], controller['exe'], controller['sha256'],
                        controller.get('start_ticks'))
            if identity != stable_identity:
                stable_identity, stable_since = identity, time.monotonic()
                first_stamp = stamp
            stability = getattr(backend, 'acceptance_stability_seconds', 0.)
            if (time.monotonic()-stable_since >= stability
                    and (not stability or stamp > first_stamp)):
                print('FC01：新版软件路径、唯一进程、运行参数和IPC推进已核对（%.1f秒）；飞机就绪另行显示。'
                      %(time.monotonic()-began),flush=True)
                return dict(controller=controller, service=info)
        except (OSError, ValueError, InstallError, subprocess.SubprocessError) as exc:
            last = exc
            stable_identity = stable_since = None
            first_stamp = None
            if info.get('ActiveState') == 'failed':
                raise InstallError('新版软件启动失败：systemd Result='+str(info.get('Result'))
                                   +' ExecMainStatus='+str(info.get('ExecMainStatus'))+'；'+str(exc)) from exc
        if time.monotonic() >= deadline:
            raise InstallError('新版启动验收失败：' + str(last))
        if time.monotonic()-progress>=10:
            print('FC01：等待新版软件初始化/IPC，已用%.0f/%.0f秒；%s。'
                  %(time.monotonic()-began,timeout,last or '观察软件状态持续推进'),flush=True)
            progress=time.monotonic()
        backend.sleep(.1)


def assert_source_files_unchanged(files):
    for item in files:
        path=Path(item['path'])
        current=digest(path) if path.is_file() and not path.is_symlink() else None
        if item.get('present',True) and current!=item['sha256']:
            raise InstallError('原服务配置被外部修改，未覆盖：'+str(path))
        if not item.get('present',True) and path.exists():
            raise InstallError('事务drop-in被外部创建，未覆盖')


def assert_dropin_pin(path, expected):
    """sudo交互或停服等待期间可能有管理员编辑；再次核对后才能切换/恢复。"""
    if path.is_symlink() or not path.is_file() or digest(path) != expected:
        raise InstallError('事务drop-in被外部修改，未覆盖')


def validate_restore(backend, backup, record):
    """先完整验证回退来源；损坏备份或外部编辑不能先停掉正常运行的候选版本。"""
    own=backend.units/(SERVICE+'.d')/DROPIN
    paths=[item.get('path') for item in record.get('files',[])]
    if (len(paths)!=len(set(paths)) or paths.count(str(own))!=1
            or paths.count(str(backend.units/SERVICE))!=1):
        raise InstallError('回退服务文件清单无效')
    for item in record['files']:
        path=Path(item['path'])
        if not item.get('present', True) and path not in (own, backend.units/SERVICE):
            raise InstallError('不允许通过回退记录删除任意路径')
        if path==own:
            continue
        if not item.get('present', True):
            if path.exists() and (path.is_symlink() or digest(path) != record.get('created_unit_sha256')):
                raise InstallError('初装服务文件被外部修改，保留它：' + str(path))
            continue
        if path.is_symlink() or not path.is_file() or digest(path)!=item.get('sha256'):
            raise InstallError('原服务配置被外部修改，保留它：'+str(path))
    # 备份必须全部验证后才开始恢复，不能恢复一半才发现另一项损坏。
    for item in record['files']:
        if item.get('present'):
            p=backup/item['backup']
            if (not re.fullmatch(r'file-[0-9]{3}',item['backup']) or p.is_symlink()
                    or digest(p)!=item['sha256']):
                raise InstallError('服务备份摘要损坏')
    runtime=backup/'runtime.yaml'
    if runtime.is_symlink() or digest(runtime)!=record['runtime_sha256']:
        raise InstallError('参数备份摘要损坏')
    live=Path(record['runtime_path'])
    if (not live.is_absolute() or not live.is_relative_to(backend.home)
            or any(p.is_symlink() for p in (live,*live.parents))
            or not live.is_file() or live.stat().st_uid!=backend.uid):
        raise InstallError('原参数恢复路径不在本用户可验证的普通文件范围')


def restore(backend, backup, record, *, restart=True):
    if backend.controllers():
        raise InstallError('仍有控制器，不能在其运行期间恢复配置')
    validate_restore(backend,backup,record)
    own=backend.units/(SERVICE+'.d')/DROPIN
    # 主unit、原drop-ins和环境文件从未改写，且上面已确认原字节仍完整；仅恢复本事务的drop-in。
    # 不以用户可写备份清单中的任意路径执行root写入。
    for item in record['files']:
        path=Path(item['path'])
        if path!=own:
            if not item.get('present', True):
                backend.remove_root(path)
            continue
        if item.get('present'):
            backend.write_root(path,(backup/item['backup']).read_bytes(),item['mode'])
        else:
            backend.remove_root(path)
    runtime=backup/'runtime.yaml'
    live=Path(record['runtime_path'])
    if live.is_symlink():raise InstallError('运行参数被改为链接，保留它')
    temp=live.with_name('.'+live.name+'.prebuilt-restore')
    temp.write_bytes(runtime.read_bytes());temp.chmod(record['runtime_mode']);os.replace(temp,live)
    backend.reload()
    # 本事务没有改enable状态；主unit/drop-ins原样还原，不需重新解释开机策略。
    if restart and record['service'].get('ActiveState')=='active':
        backend.start()
        verify_original(backend,backup,record)


def verify_original(backend, backup, record, *, timeout=STARTUP_ACCEPTANCE_SECONDS):
    """恢复验收也只依赖原软件身份、完整参数和DDS设置，不要求飞机就绪。"""
    expected = __import__('yaml').safe_load((backup/'runtime.yaml').read_text())
    began = time.monotonic()
    deadline = began + timeout
    progress = began
    while True:
        try:
            info = backend.service()
            controller = managed_controller(backend, info)
            if info.get('ActiveState') != 'active' or controller is None:
                raise InstallError('等待原版软件启动')
            old = record.get('controller')
            if old and (controller['exe'] != old['exe'] or controller['sha256'] != old['sha256']):
                raise InstallError('回退运行控制器未匹配原版')
            if not old and Path(controller['cwd']) != Path(record['service']['WorkingDirectory']):
                raise InstallError('回退运行控制器目录未匹配原版')
            _, config, _ = current_runtime(controller)
            if config != expected:
                raise InstallError('回退后实际运行参数与原始备份不同')
            actual = backend.environment(controller['pid'])
            if any(actual.get(k) != v for k, v in record['environment']['environment'].items()):
                raise InstallError('回退后实际DDS设置与原始备份不同')
            software_ipc_stamp(backend, info, controller, since=began)
            return
        except (OSError, ValueError, InstallError, subprocess.SubprocessError) as exc:
            last = exc
        if time.monotonic() >= deadline:
            raise InstallError('原配置已恢复，但原服务软件验收失败：' + str(last))
        if time.monotonic()-progress>=10:
            print('FC01：等待原版软件恢复，已用%.0f/%.0f秒；%s。'
                  %(time.monotonic()-began,timeout,last),flush=True)
            progress=time.monotonic()
        backend.sleep(.1)


def install(archive, expected, backend):
    backend.check_host()
    # 生产Backend在同一次事务内将ldd集中到必经的config_check；测试替身仍各自记录调用。
    backend._installation_checks_follow = True
    started = time.monotonic()
    folder = backend.home/'FC01_versions'
    safe_directory(folder, backend.uid)
    folder.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.fc01-unpack-', dir=folder) as tmp:
        print('FC01：核对安装包、完整文件清单和本机软件条件。', flush=True)
        extracted, manifest = extract_checked(archive, expected, Path(tmp))
        print('FC01：安装包SHA、完整清单和ARM64提取通过（%.1f秒）。'%(time.monotonic()-started),flush=True)
        entry = folder/manifest['version']
        safe_directory(entry, backend.uid)
        if entry.exists():
            if json.loads((entry/MANIFEST).read_text()) != manifest:
                raise InstallError('同名版本目录属于不同发布，未覆盖')
            verify_tree(entry, manifest)
        else:
            publish_directory(extracted, entry)
        service = backend.service()
        files = local_service_files(backend, service)
        controller = managed_controller(backend, service)
        installed = entry/'deployment/INSTALL_STATE.json'
        own = backend.units/(SERVICE+'.d')/DROPIN
        # 同版本已切换（包括已停服）：保留原回退点，不把候选误当作原版重新备份。
        if installed.is_file():
            row = json.loads(installed.read_text())
            if own.is_file() and digest(own) == row.get('dropin_sha256'):
                env = json.loads((entry/'deployment/environment.json').read_text())
                if digest(entry/'deployment/runtime.yaml') != row['runtime_sha256']:
                    raise InstallError('继承参数被修改，重复安装不覆盖')
                backend.dependencies(entry, manifest, env)
                backend.config_check(entry, manifest, entry/'deployment/runtime.yaml', env)
                if controller is not None and Path(controller['exe']) != (entry/manifest['controller_relative']).resolve():
                    raise InstallError('重复安装发现服务仍运行其他控制器，未停止它')
                if controller is None:
                    backup = Path(row['backup']).resolve()
                    allowed = (backend.home/'FC01_navigation/prebuilt-backups').resolve()
                    if backup == allowed or not backup.is_relative_to(allowed):
                        raise InstallError('原回退点不在本用户备份目录')
                    record = json.loads((backup/'BACKUP.json').read_text())
                    if Path(record['entry']).resolve() != entry.resolve():
                        raise InstallError('原回退点版本不匹配')
                    validate_restore(backend, backup, record)
                    backend.authenticate()
                    assert_dropin_pin(own, row['dropin_sha256'])
                    try:
                        stop_and_wait(backend)
                        assert_dropin_pin(own, row['dropin_sha256'])
                        if backend.controllers():
                            raise InstallError('旧控制器仍存在，未启动新版')
                        backend.start()
                        actual = verify_active(backend, entry, manifest, env)
                    except BaseException as exc:
                        try:
                            stop_and_wait(backend)
                            if not own.is_file() or digest(own) != row['dropin_sha256']:
                                raise InstallError('事务drop-in被外部修改，未覆盖')
                            restore(backend, backup, record)
                            record.update(phase='failed_restored', error=repr(exc))
                            atomic_json(backup/'BACKUP.json', record)
                        except BaseException as recovery:
                            raise InstallError('重复安装启动失败且恢复未完成；备份='+str(backup)+'；'+str(recovery)) from exc
                        raise InstallError('重复安装启动失败，原版配置和运行状态已恢复。备份='+str(backup)) from exc
                else:
                    actual = verify_active(backend, entry, manifest, env)
                return dict(entry=str(entry), backup=row['backup'], already_active=True,
                            version=manifest['version'], service_pid=int(actual['service']['MainPID']),
                            controller_pid=actual['controller']['pid'])
        if controller and Path(controller['exe']) == (entry/manifest['controller_relative']).resolve():
            raise InstallError('已有候选进程但安装事务记录不匹配，未覆盖回退点')
        environment = capture_environment(backend, service, controller)
        runtime, config, mode = old_runtime(backend, controller)
        runtime_hash = digest(runtime)
        # 绑定解析结果与同一份字节；慢依赖检查/sudo期间的现场编辑必须保留而非覆盖。
        if read_runtime(runtime)[1] != config:
            raise InstallError('本机参数在读取期间变化，请重试')
        environment['service_mode'] = mode
        backend.dependencies(entry, manifest, environment)
        backend.authenticate()
        # sudo可能等待，只重查本机进程/文件，不需要上电取得地面遥测。
        current_service = backend.service()
        current = managed_controller(backend, current_service)
        if current != controller or any(current_service.get(k) != service.get(k) for k in
                ('MainPID', 'ActiveState', 'WorkingDirectory', 'ExecStart', 'DropInPaths')):
            raise InstallError('安装检查期间服务或旧控制器变化，请重试')
        assert_source_files_unchanged(files)
        if digest(runtime) != runtime_hash:
            raise InstallError('原参数在检查期间被修改，保留修改，请重试')
        backup, record = snapshot(backend, entry, expected, service, controller, runtime,
                                  environment, files)
        if record['runtime_sha256'] != runtime_hash:
            raise InstallError('原参数在备份期间变化，未切换')
        write_deployment(entry, config, mode, environment, manifest, backup)
        migrate_vision_configuration(backend, entry, config, service.get('WorkingDirectory'))
        backend.config_check(entry, manifest, entry/'deployment/runtime.yaml', environment)
        # 有限启动预算与实测软件初始化匹配；停服安全链仍保持原TimeoutStopSec/无强杀合同。
        dropin = ('[Service]\nExecStart=\nExecStart=/bin/bash '+systemd_quote(entry/'prebuilt_boot.sh')
                  +'\nWorkingDirectory='+systemd_directory(entry)
                  +'\nTimeoutStartSec='+str(STARTUP_ACCEPTANCE_SECONDS)+'s\n').encode()
        first = not files[0].get('present', True)
        unit_bytes = first_unit(backend, entry) if first else Path(service['FragmentPath']).read_bytes()
        with tempfile.TemporaryDirectory(prefix='fc01-unit-check-') as unit_tmp:
            unit = Path(unit_tmp)/SERVICE
            unit.write_bytes(unit_bytes+b'\n'+dropin)
            backend.systemd_verify(unit)
        if first:
            record['created_unit_sha256'] = hashlib.sha256(unit_bytes).hexdigest()
            atomic_json(backup/'BACKUP.json', record)
        changed = started = False
        try:
            print('FC01：停止旧服务，等待旧控制器实际退出。', flush=True)
            stop_and_wait(backend)
            if digest(runtime) != record['runtime_sha256']:
                raise InstallError('原参数在安装期间被修改，未切换')
            assert_source_files_unchanged(files)
            changed = True
            if first:
                backend.write_root(backend.units/SERVICE, unit_bytes)
            backend.write_root(own, dropin)
            backend.reload()
            pin = hashlib.sha256(dropin).hexdigest()
            atomic_json(installed, dict(backup=str(backup), runtime_sha256=digest(entry/'deployment/runtime.yaml'),
                                       phase='starting', dropin_sha256=pin))
            # stop_and_wait必须证明全机已无旧控制器，才允许启动同一服务的新入口。
            if backend.controllers():
                raise InstallError('旧控制器仍存在，未启动新版')
            started = True
            backend.start()
            actual = verify_active(backend, entry, manifest, environment)
            record.update(phase='active', installed_dropin_sha256=pin)
            atomic_json(backup/'BACKUP.json', record)
            atomic_json(installed, dict(backup=str(backup), runtime_sha256=digest(entry/'deployment/runtime.yaml'),
                                       phase='active', dropin_sha256=pin))
            return dict(entry=str(entry), backup=str(backup), already_active=False,
                        version=manifest['version'], service_pid=int(actual['service']['MainPID']),
                        controller_pid=actual['controller']['pid'])
        except BaseException as exc:
            if not changed and (isinstance(exc,StopIncomplete) or backend.controllers()
                                or int(backend.service().get('MainPID') or 0)):
                record.update(phase='stop_incomplete', error=repr(exc))
                atomic_json(backup/'BACKUP.json', record)
                raise InstallError('旧服务停止未完成；原服务配置未修改，未启动新版、未强杀。备份='
                                   +str(backup)+'；'+str(exc)) from exc
            try:
                if started:
                    stop_and_wait(backend)
                if changed:
                    if own.is_symlink() or (own.exists() and digest(own) != hashlib.sha256(dropin).hexdigest()):
                        raise InstallError('新drop-in被外部修改，未覆盖')
                    restore(backend, backup, record)
                elif not backend.controllers() and service.get('ActiveState') == 'active':
                    backend.start()
                    verify_original(backend, backup, record)
                record.update(phase='failed_restored', error=repr(exc))
                atomic_json(backup/'BACKUP.json', record)
            except BaseException as recovery:
                record.update(phase='manual_recovery_required', error=repr(exc), recovery_error=repr(recovery))
                atomic_json(backup/'BACKUP.json', record)
                raise InstallError('安装失败且软件恢复未完成；未强杀。备份='+str(backup)+'；'+str(recovery)) from exc
            raise InstallError('安装失败，原版配置和运行状态已恢复。备份='+str(backup)+'；'+str(exc)) from exc


def rollback(backup,backend):
    backend.check_host()
    backup=Path(backup).resolve()
    allowed=(backend.home/'FC01_navigation/prebuilt-backups').resolve()
    if not backup.is_relative_to(allowed) or backup==allowed:
        raise InstallError('回退目录不在本用户预编译备份范围')
    record=json.loads((backup/'BACKUP.json').read_text())
    entry=Path(record['entry'])
    if not entry.is_relative_to(backend.home/'FC01_versions'):
        raise InstallError('备份版本目录无效')
    own=backend.units/(SERVICE+'.d')/DROPIN
    if own.is_symlink() or not own.is_file() or digest(own)!=record.get('installed_dropin_sha256'):
        raise InstallError('当前服务切换记录不匹配，拒绝覆盖')
    installed=json.loads((entry/'deployment/INSTALL_STATE.json').read_text())
    if Path(installed.get('backup','')).resolve()!=backup:
        raise InstallError('候选版本与回退目录不匹配')
    validate_restore(backend,backup,record)
    backend.authenticate()
    release=json.loads((entry/MANIFEST).read_text())
    info=backend.service()
    current = managed_controller(backend, info)
    if current is not None and Path(current['exe']) != (entry/release['controller_relative']).resolve():
        raise InstallError('当前运行版本不是本回退记录的候选，未停止它')
    # sudo交互期间备份/服务源仍可能变化；停止前再次验证，恢复前还会再验一次。
    validate_restore(backend,backup,record)
    assert_dropin_pin(own, record['installed_dropin_sha256'])
    stop_and_wait(backend)
    assert_dropin_pin(own, record['installed_dropin_sha256'])
    restore(backend,backup,record)
    record['phase']='rolled_back';atomic_json(backup/'BACKUP.json',record)
    return dict(backup=str(backup),restored_controller=(record.get('controller') or {}).get('exe'), restored_active=record['service'].get('ActiveState')=='active')


def source_freshness(state, elapsed):
    """仅供显示：累计快照存放时间，各路反馈寿命不能被状态刷新续长。"""
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    timings = state.get('source_timing')
    result = {}
    for topic in ('status', 'land', 'local', 'attitude', 'flags', 'battery'):
        item = timings.get(topic) if isinstance(timings, dict) else None
        valid = (finite(elapsed) and elapsed >= 0 and isinstance(item, dict))
        if valid:
            timeout = item.get('timeout_s')
            valid = (finite(timeout) and timeout > 0
                     and item.get('source_reason') == 'VALID'
                     and item.get('receipt_reason') == 'VALID')
        if valid:
            names = ('envelope_age_s', 'receipt_age_s')
            if topic in ('local', 'attitude'):
                names += ('sample_age_s', 'sample_progress_age_s')
            for name in names:
                age = item.get(name)
                if (not finite(age)
                        or (name in ('receipt_age_s', 'sample_progress_age_s') and age < 0)
                        or max(0., age) + elapsed >= timeout):
                    valid = False
                    break
        result[topic] = bool(valid)
    return result



def vision_readiness(backend, info):
    """仅显示同一服务的视觉观察；子消息年龄独立，不借外壳文件续鲜。"""
    result = dict(mode='unknown', current=False, camera_current=False, mission_current=False,
                  camera=None, mission=None)
    try:
        entry = Path(info.get('WorkingDirectory') or '')
        if not entry.is_absolute():
            return result
        value = json.loads((entry/'deployment/vision-runtime.json').read_text())
        now = time.monotonic()
        if not isinstance(value, dict):
            return result
        result['mode'] = value.get('mode', 'unknown')
        stamp = value.get('receive_monotonic_ns')
        current = (type(value.get('schema_version')) is int and value['schema_version'] == 1
                   and info.get('ActiveState') == 'active'
                   and str(value.get('supervisor_pid')) == str(info.get('MainPID'))
                   and value.get('boot_id') == backend.boot_id()
                   and type(stamp) is int and 0 < stamp <= now*1e9 and now*1e9-stamp <= 3e9)
        result['current'] = current
        if current:
            result['calibration_state'] = value.get('calibration_state', 'unknown')
            result['mission_control_enabled'] = value.get('mission_control_enabled')
            observations = value.get('observations')
            for name in ('camera', 'mission'):
                item = observations.get(name) if isinstance(observations, dict) else None
                if (isinstance(item, dict) and isinstance(item.get('value'), dict)
                        and type(item.get('publisher_count')) is int and item['publisher_count'] == 1):
                    stamp = item.get('receive_monotonic_ns')
                    if type(stamp) is int and 0 < stamp <= now*1e9 and now*1e9-stamp <= 1e9:
                        result[name] = item['value']
                        result[name+'_current'] = True
            result['error'] = value.get('error')
    except (OSError, ValueError, TypeError, KeyError):
        pass
    return result


def flight_readiness(backend):
    """仅读现有快照显示就绪；失败/缺失只影响此字段，绝不反向否决安装。"""
    try:
        row = json.loads((backend.home/'FC01_navigation/status.json').read_text())
        if not isinstance(row, dict) or not isinstance(row.get('state'), dict):
            raise ValueError('快照不是有效的状态对象')
        info = backend.service()
        controllers = backend.controllers()
        current = (len(controllers) == 1 and row.get('supervisor_pid') == int(info.get('MainPID') or 0)
                   and row.get('boot_id') == backend.boot_id()
                   and row.get('controller_pid') == controllers[0]['pid']
                   and row.get('state_controller_pid') == controllers[0]['pid'])
        state = row.get('state') or {}
        stamp = state.get('diagnostic_monotonic_s')
        elapsed = time.monotonic()-stamp if type(stamp) in (int, float) and math.isfinite(stamp) else None
        current = current and elapsed is not None and 0 <= elapsed <= 3
        sources = source_freshness(state, elapsed)
        checks = {'DDS': row.get('feedback_current') is True and all(sources.values()),
                  'RC': sources['flags'] and all(state.get(k) is True for k in ('rc_link_ready','rc_source_valid','rc_switches_fresh')),
                  '姿态': sources['attitude'] and all(state.get(k) is True for k in ('attitude_fresh','attitude_sample_fresh')),
                  '位置': sources['local'] and all(state.get(k) is True for k in ('local_fresh','local_sample_fresh','z_valid','vz_valid')),
                  '电池': sources['battery'] and all(state.get(k) is True for k in ('battery_fresh','battery_connected')),
                  'PX4': sources['status'] and sources['land'] and all(state.get(k) is True for k in ('status_fresh','land_fresh','arming_state_known','preflight')),
                  '飞行动作权限': all(state.get(k) is True for k in ('flight_actions_enabled','takeoff_enabled','rc_navigation_enabled'))}
        missing = [k for k, v in checks.items() if not v or not current]
        feedback_current = current and not missing
        command_source = state.get('visual_reason') or 'NATIVE_ALTITUDE'
        if state.get('px4_takeoff_position_hold') is True:
            command_source = ('PX4_POSITION_MISSION' if state.get('position_mission_motion') is True
                              else 'PX4_POSITION_NATIVE')
        return dict(ready_for_manual_rc=feedback_current and row.get('status')=='READY_WAIT_ARM',
                    missing=missing, phase=state.get('phase'), reason=state.get('reason'),
                    status_current=current,
                    target_height_m=state.get('rc_target_height'),
                    native_navigation_allowed=(state.get('navigation_allowed')
                        if feedback_current and type(state.get('navigation_allowed')) is bool else None),
                    current_command_source=command_source if feedback_current else None,
                    vision=vision_readiness(backend, info),
                    note='仅显示；人工RC准入/接管/Kill/失效保护由原生控制器判定')
    except (OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError):
        return dict(ready_for_manual_rc=False, missing=['DDS','RC','姿态','位置','电池','PX4'],
                    reason='尚无当前运行状态，等待飞控上电后按原流程确认')


def status(backend):
    info = backend.service()
    return dict(service=info, controllers=backend.controllers(),
                flight_readiness=flight_readiness(backend), vision=vision_readiness(backend, info))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path,nargs='?')
    parser.add_argument('--expected-sha256')
    parser.add_argument('--rollback',type=Path)
    parser.add_argument('--status',action='store_true')
    args=parser.parse_args(argv)
    try:
        backend=Backend()
        if args.status:
            print(json.dumps(status(backend),ensure_ascii=False,indent=2));return 0
        if bool(args.rollback)==bool(args.archive):
            parser.error('指定安装包或 --rollback，不能同时提供')
        lock=backend.home/'.cache/fc01_navigation/prebuilt-install.lock'
        lock.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
        with lock.open('a') as stream:
            fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
            print('FC01：维护切换适用于落地、拆桨、飞控断电；安装成功不等于飞行就绪。',flush=True)
            result=rollback(args.rollback,backend) if args.rollback else install(args.archive,args.expected_sha256,backend)
        print(json.dumps(result,ensure_ascii=False,indent=2))
        if args.archive:
            print('FC01_PREBUILT_INSTALL_OK entry='+result['entry']+' service_pid='+str(result['service_pid'])+' controller_pid='+str(result['controller_pid']))
            print('FC01_FLIGHT_READINESS='+json.dumps(flight_readiness(backend),ensure_ascii=False))
        else:
            print('FC01_PREBUILT_ROLLBACK_OK')
        if 'backup' in result:
            installer = os.environ.get('FC01_PREBUILT_INSTALLER_PATH', './install_fc01.sh')
            print('FC01_ROLLBACK_COMMAND=bash '+shlex.quote(installer)+' --rollback '+shlex.quote(result['backup']))
        return 0
    except (OSError,ValueError,TypeError,KeyError,RuntimeError,subprocess.SubprocessError) as exc:
        print('FC01_PREBUILT_INSTALL_FAILED: '+str(exc),file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())

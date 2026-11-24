#!/usr/bin/env python3
"""地面维护时选择定高、起飞点保持或非接触track；只修改deployment配置。

必须先停止同一systemd服务并确认实际原生进程退出。缺现场标定可显式选择
--pending-calibration待机；任务控制保持关闭，不会把仓库示例值改成confirmed。
修改前完整备份deployment，配置变更不动RC/推力/DDS。
"""
from __future__ import annotations
import argparse
import copy
import fcntl
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
import yaml

SERVICE = 'fc01-navigation.service'
MODE_FILE = 'vision_mode.json'
MODES = ('altitude', 'position', 'track', 'position-track')
VISION_MODES = ('track', 'position-track')
POSITION_MODES = ('position', 'position-track')
POSITION_LIMITS = (
    ('horizontal_max_radius_m', 'visual_max_radius', 2.),
    ('horizontal_max_speed_mps', 'visual_max_speed', .5),
    ('horizontal_max_yaw_rate_rps', 'visual_max_yaw_rate', .35),
)
NO_POSITION_WAYPOINT = dict(px4_position_waypoint_enabled=False,
                            px4_position_waypoint_origin_id='',
                            px4_position_waypoint_north_m=0.,
                            px4_position_waypoint_east_m=0.)
PENDING_SITE = dict(schema_version=1, calibration_state='pending',
    note='现场设备、内外参、目标模型、采集延迟和安全间距均未确认；仅运行软件诊断，不允许任务控制。')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    fd, temp = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
        os.chmod(temp, 0o600); os.replace(temp, path)
    finally:
        if os.path.exists(temp): os.unlink(temp)


@contextmanager
def deployment_lock(entry):
    """服务与配置工具共用的非阻塞锁；避免停服检查后并发启动读到半份配置。"""
    path=Path(entry)/'deployment/mode.lock'
    fd=os.open(path,os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise ValueError('服务或另一个配置事务正在使用本版本') from exc
        yield


def atomic_text(path, text):
    path=Path(path);fd,temp=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(fd,'w') as stream:
            stream.write(text);stream.flush();os.fsync(stream.fileno())
        os.chmod(temp,0o600);os.replace(temp,path)
    finally:
        if os.path.exists(temp):os.unlink(temp)


def load_parameters(path, node):
    value = yaml.safe_load(Path(path).read_text())
    try:
        params = value[node]['ros__parameters']
    except (TypeError, KeyError) as exc:
        raise ValueError('配置缺少 '+node+'/ros__parameters') from exc
    if not isinstance(params, dict): raise ValueError('ROS参数必须为对象')
    return value, params


def finite_number(value, name, low=None, high=None):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError(name+'必须为有限数值')
    if low is not None and value <= low or high is not None and value > high:
        raise ValueError(name+'超出配置界限')
    return float(value)


def track_endpoints(camera, mission, *, control_enabled):
    """固定实机消息/坐标入口；任务开关不等于原生飞行授权。"""
    camera.update(visualization=False, show_tuning_window=False, publish_debug_image=False,
        state_topic='/fc01/vision/target_observation', status_topic='/fc01/vision/camera_status',
        frame_id='body_frd', body_frame_id='body_frd')
    mission.update(platform_input_mode='px4', engage_mode='track', control_enabled=control_enabled,
        takeoff_altitude_m=1.5, native_flight_topic='/fc01/native/flight_snapshot',
        navigation_command_topic='/fc01/navigation/command',
        vision_control_topic='/fc01/vision/velocity_command_v2',
        horizontal_command_topic='/fc01/vision/horizontal_command',
        platform_state_topic='/fc01/platform/state', vision_state_topic='/fc01/vision/target_observation')
    return camera, mission


def pending_parameters(*, camera, mission):
    """无现场测量的显式待机配置；零值和单位旋转只供软件初始化，不算标定。

    使用不存在的专用设备路径，避免误打开仓库作者的video2。所有确认/来源ID清空，
    任务control_enabled=false，因此即使随后收到其他反馈也不产生有效运动意图。
    填好真实--site后另做一次地面配置事务，才能启用任务控制。
    """
    c=copy.deepcopy(camera); m=copy.deepcopy(mission)
    c.update(device_path='/dev/fc01-camera-unconfigured', device_index=-1,
        frame_width=640, frame_height=480, frame_fps=30, publish_rate_hz=30.0,
        camera_fx=0.0, camera_fy=0.0, camera_cx=0.0, camera_cy=0.0, camera_hfov_deg=60.0,
        camera_to_body_wxyz=[1.0,0.0,0.0,0.0], target_shape='auto',
        target_width_m=0.0, target_height_m=0.0, target_diameter_m=0.0,
        capture_latency_bound_s=0.0)
    for name in ('calibration_confirmed','image_rectified_confirmed','extrinsics_confirmed',
                 'target_model_confirmed','capture_latency_confirmed'):
        c[name]=False
    for name in ('calibration_id','extrinsics_id','target_model_id'):
        c[name]=''
    m.update(camera_extrinsics_confirmed=False, camera_extrinsics_id='',
        standoff_calibration_confirmed=False, standoff_calibration_id='', standoff_m=0.0,
        camera_pos_x=0.0, camera_pos_y=0.0, camera_pos_z=0.0,
        horizontal_navigation_mode='hold', waypoint_alignment_confirmed=False,
        waypoint_alignment_origin_id='', navigation_waypoint_x=0.0,
        navigation_waypoint_y=0.0, navigation_waypoint_yaw_valid=False)
    return track_endpoints(c,m,control_enabled=False)


def site_parameters(site, *, camera, mission):
    """确认配置与未标定待机分开校验；pending不接受夹带现场值或确认字段。"""
    if isinstance(site,dict) and site.get('calibration_state')=='pending':
        if site != PENDING_SITE:
            raise ValueError('未标定待机记录不可夹带现场测量或确认字段')
        return pending_parameters(camera=camera,mission=mission)
    return validate_site(site,camera=camera,mission=mission)


def validate_site(site, *, camera, mission, takeoff_centered=False):
    """输入是现场证据索引及完整ROS参数；返回副本，不补造任何测量或confirmed。"""
    if not isinstance(site, dict) or site.get('schema_version') != 1:
        raise ValueError('现场配置schema_version必须为1')
    provenance=site.get('provenance')
    required=('camera_device','intrinsics','extrinsics','target_model','capture_latency','standoff')
    if not isinstance(provenance, dict) or any(not isinstance(provenance.get(k),str) or not provenance[k].strip() for k in required):
        raise ValueError('现场配置缺少provenance来源记录：'+','.join(required))
    c=copy.deepcopy(camera); m=copy.deepcopy(mission)
    if not isinstance(site.get('camera'),dict) or not isinstance(site.get('mission'),dict):
        raise ValueError('现场配置需要camera和mission参数对象')
    camera_required=('device_path','camera_fx','camera_fy','camera_cx','camera_cy',
                     'frame_width','frame_height','frame_fps','camera_to_body_wxyz',
                     'capture_latency_bound_s','target_shape')
    mission_required=('camera_pos_x','camera_pos_y','camera_pos_z','standoff_m')
    if any(k not in site['camera'] for k in camera_required) or any(k not in site['mission'] for k in mission_required):
        raise ValueError('现场配置必须显式填写设备、内外参、图像尺寸、延时、目标模型和安全间距，不能借用仓库示例值')
    allowed_mission=set(mission_required)|{'camera_extrinsics_confirmed','camera_extrinsics_id',
        'standoff_calibration_confirmed','standoff_calibration_id','horizontal_navigation_mode',
        'horizontal_max_radius_m','horizontal_max_speed_mps','horizontal_max_yaw_rate_rps',
        'waypoint_alignment_confirmed','waypoint_alignment_origin_id',
        'navigation_waypoint_x','navigation_waypoint_y','navigation_waypoint_yaw',
        'navigation_waypoint_yaw_valid'}
    if set(site['mission'])-allowed_mission:
        raise ValueError('现场mission仅接受标定/安全间距/有界航点，不改任务权限、时效或锁定策略')
    if set(site['camera'])-set(c) or set(site['mission'])-set(m):
        raise ValueError('现场配置含未知ROS参数，拒绝拼写错误')
    c.update(site['camera']);m.update(site['mission'])
    for name in ('calibration_confirmed','image_rectified_confirmed','extrinsics_confirmed','target_model_confirmed','capture_latency_confirmed'):
        if c.get(name) is not True:raise ValueError('相机未确认：'+name)
    for name in ('calibration_id','extrinsics_id','target_model_id'):
        if not isinstance(c.get(name),str) or not c[name].strip():raise ValueError('相机缺来源ID：'+name)
    device=c.get('device_path')
    if not isinstance(device,str) or not device.startswith('/dev/') or any(ord(x)<32 for x in device):
        raise ValueError('现场device_path必须明确为/dev/路径（可用by-id）；不要求当前相机插入')
    for name in ('camera_fx','camera_fy'):
        finite_number(c.get(name),name,0)
    for name in ('camera_cx','camera_cy'):
        finite_number(c.get(name),name)
    for name in ('frame_width','frame_height','frame_fps'):
        if type(c.get(name)) is not int or not 1<=c[name]<=4096:raise ValueError(name+'必须为有界正整数')
    finite_number(c.get('capture_latency_bound_s'),'capture_latency_bound_s',0,.2)
    q=c.get('camera_to_body_wxyz')
    if not isinstance(q,list) or len(q)!=4:raise ValueError('camera_to_body_wxyz必须有4个wxyz值')
    q=[finite_number(v,'camera_to_body_wxyz') for v in q]
    if abs(sum(v*v for v in q)-1)>1e-3:raise ValueError('相机安装四元数必须单位化')
    shape=c.get('target_shape')
    if shape not in ('circle','rect'):raise ValueError('首测target_shape需明确circle或rect')
    for name in (('target_diameter_m',) if shape=='circle' else ('target_width_m','target_height_m')):
        if name not in site['camera']:raise ValueError('目标尺寸必须显式来自现场测量：'+name)
        finite_number(c.get(name),name,0)
    for name in ('camera_extrinsics_confirmed','standoff_calibration_confirmed'):
        if m.get(name) is not True:raise ValueError('任务未确认：'+name)
    for name in ('camera_extrinsics_id','standoff_calibration_id'):
        if not isinstance(m.get(name),str) or not m[name].strip():raise ValueError('任务缺来源ID：'+name)
    if m['camera_extrinsics_id']!=c['extrinsics_id']:
        raise ValueError('相机旋转和任务杆臂必须引用同一次外参记录ID')
    for name in ('camera_pos_x','camera_pos_y','camera_pos_z'):finite_number(m.get(name),name)
    finite_number(m.get('standoff_m'),'standoff_m',0,20)
    for name,bound in (('horizontal_max_radius_m',2),('horizontal_max_speed_mps',.5),('horizontal_max_yaw_rate_rps',.35)):
        finite_number(m.get(name),name,0,bound)
    if m.get('horizontal_navigation_mode') not in ('hold','waypoint'):
        raise ValueError('水平模式只可为hold或waypoint')
    if m['horizontal_navigation_mode']=='waypoint':
        if m.get('waypoint_alignment_confirmed') is not True or not m.get('waypoint_alignment_origin_id'):
            raise ValueError('航点需要当前PX4原点确认')
        x=finite_number(m.get('navigation_waypoint_x'),'navigation_waypoint_x')
        y=finite_number(m.get('navigation_waypoint_y'),'navigation_waypoint_y')
        # 位置模式的半径以本次地面起飞点为圆心，由原生在飞行时检查。
        if not takeoff_centered and math.hypot(x,y)>m['horizontal_max_radius_m']:
            raise ValueError('首测航点必须在现场对齐原点的允许半径内')
    # 确保首测入口的权限/端点固定；允许的现场值是测量和有界策略，不是模拟或冲撞模式。
    return track_endpoints(c,m,control_enabled=True)


def selected_mode(params):
    """根据原生控制出口和任务开关，识别部署模式。"""
    visual = params.get('visual_navigation_enabled')
    horizontal = params.get('px4_horizontal_enabled', False)
    if type(visual) is not bool or type(horizontal) is not bool or visual != horizontal:
        raise ValueError('视觉与PX4水平开关必须同时显式启用或关闭')
    position = params.get('px4_takeoff_position_hold', False)
    position_mission = params.get('px4_position_mission_enabled', False)
    waypoint = params.get('px4_position_waypoint_enabled', False)
    if (type(position) is not bool or type(position_mission) is not bool
            or position_mission != (position and visual)
            or type(waypoint) is not bool or (waypoint and not position_mission)):
        raise ValueError('位置跟踪要求同时启用起飞点保持和视觉水平任务')
    if position:
        slot = params.get('rc_navigation_mode_slot')
        if (params.get('rc_navigation_enabled') is not True or params.get('arm_enabled') is not False
                or params.get('rc_target_height') != 1.5 or type(slot) is not int or slot != 1
                or params.get('rc_arm_to_navigation_timeout', 8.) != 8.):
            raise ValueError('位置模式要求既有人工RC、1.5m目标、槽位1及8秒窗口')
        return 'position-track' if visual else 'position'
    return 'track' if visual else 'altitude'


def position_track_parameters(site, *, camera, mission, params, configure=False):
    """核对实机标定，限制任务给出的指令范围。"""
    camera, mission = validate_site(site, camera=camera, mission=mission, takeoff_centered=True)
    mission['position_takeoff_anchor_authority'] = True
    for mission_key, native_key, default in POSITION_LIMITS:
        limit = finite_number(params.get(native_key, default), native_key, 0)
        if mission[mission_key] > limit:
            raise ValueError(f'{mission_key}超过原生包络{native_key}')
        if configure:
            params[native_key] = float(mission[mission_key])
        elif mission[mission_key] != limit:
            raise ValueError(f'原生包络{native_key}与现场{mission_key}不一致')
    if mission['standoff_m'] > mission['horizontal_max_radius_m']:
        raise ValueError('现场安全间距超过水平任务半径')
    waypoint = NO_POSITION_WAYPOINT.copy()
    if mission['horizontal_navigation_mode'] == 'waypoint':
        origin = mission['waypoint_alignment_origin_id']
        if (not isinstance(origin, str)
                or not re.fullmatch(r'px4_local:(?:[0-9]+:){5}[0-9]+', origin)):
            raise ValueError('航点原点必须来自当前PX4局部坐标标识')
        # 现场任务文件使用 ENU 坐标；PX4 轨迹使用 NED。原生独立检查本次起飞半径。
        waypoint.update(px4_position_waypoint_enabled=True,
                        px4_position_waypoint_origin_id=origin,
                        px4_position_waypoint_north_m=float(mission['navigation_waypoint_y']),
                        px4_position_waypoint_east_m=float(mission['navigation_waypoint_x']))
    if configure:
        params.update(waypoint)
    elif any(params.get(key, NO_POSITION_WAYPOINT[key]) != value for key, value in waypoint.items()):
        raise ValueError('原生航点与已核验现场配置不一致')
    return camera, mission


def configured_mode(entry, params):
    """服务只消费完整、哈希一致的显式地面配置；默认false不需要视觉硬件。"""
    entry=Path(entry).resolve();path=entry/'deployment'/MODE_FILE
    mode = selected_mode(params)
    if mode not in VISION_MODES:
        return dict(mode=mode, enabled=False)
    if params.get('rc_target_height')!=1.5 or params.get('rc_navigation_mode_slot')!=1:
        raise ValueError('首测track要求1.5m和导航槽位1；不自动修改RC映射')
    value=json.loads(path.read_text())
    if value.get('schema_version')!=1 or value.get('mode')!=mode:
        raise ValueError('缺少与原生模式一致的视觉配置记录：'+mode)
    for key,name in (('camera','vision_camera.yaml'),('mission','vision_mission.yaml'),('site','vision_site.json')):
        p=entry/'deployment'/name
        if value.get(key+'_sha256')!=sha(p):raise ValueError('视觉配置摘要不匹配：'+name)
        value[key+'_path']=str(p)
    site=json.loads(Path(value['site_path']).read_text())
    _,camera=load_parameters(entry/'target/src/fc01_vision/config/vision.yaml','/**')
    _,mission=load_parameters(entry/'target/src/fc01_mission/config/mission.yaml','fc01_mission')
    if mode == 'position-track':
        camera, mission = position_track_parameters(site, camera=camera, mission=mission, params=params)
    else:
        camera, mission = site_parameters(site, camera=camera, mission=mission)
    state='pending' if site.get('calibration_state')=='pending' else 'confirmed'
    if value.get('calibration_state','confirmed')!=state:
        raise ValueError('视觉模式记录与标定状态不一致')
    _,actual_camera=load_parameters(value['camera_path'],'/**')
    _,actual_mission=load_parameters(value['mission_path'],'fc01_mission')
    if camera!=actual_camera or mission!=actual_mission:
        raise ValueError('视觉配置与来源记录/固定首测入口不一致')
    return value|dict(enabled=True,calibration_state=state,
                      control_enabled=mission['control_enabled'])


def assert_stopped(run=subprocess.run, proc=Path('/proc')):
    state=run(['/usr/bin/systemctl','show',SERVICE,'-p','ActiveState','-p','MainPID'],
              capture_output=True,text=True,check=True,timeout=15).stdout
    fields=dict(line.split('=',1) for line in state.splitlines() if '=' in line)
    if fields.get('ActiveState') not in ('inactive','failed') or fields.get('MainPID')!='0':
        raise ValueError('请先在地面维护状态停止fc01-navigation.service；工具不会在飞行中切配置')
    for p in proc.glob('[0-9]*'):
        try:
            executable=os.readlink(p/'exe')
        except FileNotFoundError:continue
        except PermissionError:
            # 不使用旧status JSON猜测活进程；本用户控制器应可读取/proc/exe。
            continue
        if Path(executable.removesuffix(' (deleted)')).name=='fc01_system_node':
            raise ValueError('仍有实际fc01_system_node进程：'+p.name)


def _configure_locked(entry, mode, site_path=None, *, pending_calibration=False,
                      run=subprocess.run, check_stopped=assert_stopped):
    entry=Path(entry).resolve();dep=entry/'deployment';runtime=dep/'runtime.yaml'
    check_stopped(run=run)
    profile,p=load_parameters(runtime,'fc01_system')
    install_state=dep/'INSTALL_STATE.json'
    receipt=None
    if install_state.exists():
        receipt=json.loads(install_state.read_text())
        if not isinstance(receipt,dict) or receipt.get('runtime_sha256')!=sha(runtime):
            raise ValueError('现用runtime与安装记录不一致；未授权覆盖已有配置修改')
    if p.get('arm_enabled') is not False or p.get('profile')!='attitude_altitude':
        raise ValueError('只支持现有人工RC姿态/高度配置')
    if p.get('rc_target_height')!=1.5 or p.get('rc_navigation_mode_slot')!=1:
        raise ValueError('现用目标/RC不是1.5m、槽位1；保留现场配置，请先核对')
    candidate=copy.deepcopy(profile);cp=candidate['fc01_system']['ros__parameters']
    pending={};record=dict(schema_version=1,mode=mode)
    if mode in VISION_MODES:
        if not pending_calibration and site_path is None:
            raise ValueError('track需--site现场来源/标定JSON，或--pending-calibration仅软件待机')
        site=copy.deepcopy(PENDING_SITE) if pending_calibration else json.loads(Path(site_path).read_text())
        cv,cam=load_parameters(entry/'target/src/fc01_vision/config/vision.yaml','/**')
        mv,mission=load_parameters(entry/'target/src/fc01_mission/config/mission.yaml','fc01_mission')
        cp.update(visual_navigation_enabled=True,px4_horizontal_enabled=True)
        # 新水平路线复用原visual包络，缺省值与已验证原生默认一致且不扩大现场包络。
        limits={'visual_max_speed':.5,'visual_max_acceleration':.6,'visual_max_tilt_deg':5.,
                'visual_max_yaw_rate':.35,'visual_max_radius':2.,'visual_max_vertical_speed':.15}
        for key,limit in limits.items():
            current=cp.get(key,limit);finite_number(current,key,0)
            cp[key]=min(current,limit)
        cp['visual_max_tilt_deg']=min(cp['visual_max_tilt_deg'],finite_number(cp['aa_max_tilt_deg'],'aa_max_tilt_deg',0))
        cp['visual_max_vertical_speed']=min(cp['visual_max_vertical_speed'],
            finite_number(cp['aa_max_climb'],'aa_max_climb',0),finite_number(cp['aa_max_descent'],'aa_max_descent',0))
        cp['visual_max_height']=min(finite_number(cp.get('visual_max_height',cp['aa_max_height']),'visual_max_height',0),
                                    finite_number(cp['aa_max_height'],'aa_max_height',0))
        if mode == 'position-track':
            cam, mission = position_track_parameters(site, camera=cam, mission=mission, params=cp,
                                                     configure=True)
        elif pending_calibration:
            cam, mission = pending_parameters(camera=cam, mission=mission)
        else:
            cam, mission = validate_site(site, camera=cam, mission=mission)
        record['calibration_state']='pending' if pending_calibration else 'confirmed'
        cv['/**']['ros__parameters']=cam;mv['fc01_mission']['ros__parameters']=mission
        pending={'vision_camera.yaml':yaml.safe_dump(cv,allow_unicode=True,sort_keys=False),
                 'vision_mission.yaml':yaml.safe_dump(mv,allow_unicode=True,sort_keys=False),
                 'vision_site.json':json.dumps(site,ensure_ascii=False,indent=2,allow_nan=False)+'\n'}
        for key,name in (('camera','vision_camera.yaml'),('mission','vision_mission.yaml'),('site','vision_site.json')):
            record[key+'_sha256']=hashlib.sha256(pending[name].encode()).hexdigest()
    else:cp.update(visual_navigation_enabled=False,px4_horizontal_enabled=False)
    if mode != 'position-track':
        for key, value in NO_POSITION_WAYPOINT.items():
            if key in cp:
                cp[key] = value
    for key, enabled in (('px4_takeoff_position_hold', mode in POSITION_MODES),
                         ('px4_position_mission_enabled', mode == 'position-track')):
        if enabled or key in cp:
            cp[key] = enabled
    if selected_mode(cp) != mode:
        raise ValueError('模式与原生控制配置不一致')
    backup=dep/('mode-backup-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    backup.mkdir(mode=0o700)
    names=('runtime.yaml',MODE_FILE,'vision_camera.yaml','vision_mission.yaml','vision_site.json',
           'INSTALL_STATE.json')
    for name in names:
        if (dep/name).is_file():shutil.copy2(dep/name,backup/name)
    try:
        check_stopped(run=run)
        for name,text in pending.items():atomic_text(dep/name,text)
        atomic_text(runtime,yaml.safe_dump(candidate,allow_unicode=True,sort_keys=False))
        atomic_json(dep/MODE_FILE,record)
        # 使用与服务相同的配置检查：无串口、无ROS飞行发布、无需飞机/相机反馈。
        # 完整视觉运行库在ARM64仿真环境检查较慢；仅扩大本机软件检查预算。
        run(['/bin/bash',str(entry/'prebuilt_boot.sh'),'check'],check=True,timeout=600)
        if receipt is not None:
            # 地面工具完成受验证配置事务后同步本版安装收据；否则下一次重复
            # 安装会把合法模式切换当成不明改动。旧版本回退备份保持原样。
            receipt['runtime_sha256']=sha(runtime)
            atomic_json(install_state,receipt)
    except BaseException:
        for name in names:
            if (backup/name).exists():shutil.copy2(backup/name,dep/name)
            elif (dep/name).exists():(dep/name).unlink()
        raise
    return dict(mode=mode,entry=str(entry),backup=str(backup),automatic_arm=False,
                calibration_state=record.get('calibration_state','not_required'),
                mission_control_enabled=(mode in VISION_MODES and not pending_calibration),
                note='配置完成；服务尚未启动。启动后原生RC/到高/失效保护仍独立判定。')


def configure(entry, mode, site_path=None, *, pending_calibration=False,
              run=subprocess.run, check_stopped=assert_stopped):
    if mode not in MODES:
        raise ValueError('模式必须为'+', '.join(MODES))
    if pending_calibration and (mode!='track' or site_path is not None):
        raise ValueError('--pending-calibration仅用于track，且不能同时提供--site')
    if mode == 'position-track' and site_path is None:
        raise ValueError('position-track需要--site现场来源和标定')
    if mode in ('altitude', 'position') and site_path is not None:
        raise ValueError('不启用视觉的模式无需--site')
    with deployment_lock(entry):
        return _configure_locked(entry,mode,site_path,pending_calibration=pending_calibration,
                                 run=run,check_stopped=check_stopped)


def migrate(entry, source_entry):
    """安装前校验并复制同一模式的现场证据，保留来源目录和当前服务。"""
    entry,source_entry=Path(entry).resolve(),Path(source_entry).resolve()
    if entry==source_entry:raise ValueError('迁移源和目标目录必须不同')
    _,new_params=load_parameters(entry/'deployment/runtime.yaml','fc01_system')
    _,old_params=load_parameters(source_entry/'deployment/runtime.yaml','fc01_system')
    current=configured_mode(source_entry,old_params)
    mode = selected_mode(new_params)
    if not current.get('enabled') or current['mode'] != mode:
        raise ValueError('只能迁移与目标原生模式一致且已启用的视觉配置')
    with deployment_lock(entry):
        # 先在内存检查同一现场记录在本版默认字段中仍合法，避免旧未知参数被悄悄忽略。
        site=json.loads(Path(current['site_path']).read_text())
        _,camera=load_parameters(entry/'target/src/fc01_vision/config/vision.yaml','/**')
        _,mission=load_parameters(entry/'target/src/fc01_mission/config/mission.yaml','fc01_mission')
        if mode == 'position-track':
            camera, mission = position_track_parameters(site, camera=camera, mission=mission, params=new_params)
        else:
            camera, mission = site_parameters(site, camera=camera, mission=mission)
        payload={'vision_camera.yaml':yaml.safe_dump({'/**':{'ros__parameters':camera}},allow_unicode=True,sort_keys=False),
                 'vision_mission.yaml':yaml.safe_dump({'fc01_mission':{'ros__parameters':mission}},allow_unicode=True,sort_keys=False),
                 'vision_site.json':json.dumps(site,ensure_ascii=False,indent=2,allow_nan=False)+'\n'}
        record=dict(schema_version=1,mode=mode,migrated_from=str(source_entry),
                    calibration_state=current['calibration_state'])
        for key,name in (('camera','vision_camera.yaml'),('mission','vision_mission.yaml'),('site','vision_site.json')):
            record[key+'_sha256']=hashlib.sha256(payload[name].encode()).hexdigest()
        for name,text in payload.items():atomic_text(entry/'deployment'/name,text)
        atomic_json(entry/'deployment'/MODE_FILE,record)
        configured_mode(entry,new_params)
    return dict(mode=mode,entry=str(entry),source_entry=str(source_entry),activated=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=(*MODES,'migrate'))
    parser.add_argument('--entry',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--site',type=Path)
    parser.add_argument('--pending-calibration',action='store_true',
                        help='只启动视觉软件诊断；缺现场标定时保持任务控制关闭')
    parser.add_argument('--source-entry',type=Path)
    parser.add_argument('--maintenance',action='store_true',
                        help='确认已落地、拆桨、飞控断电，并停止控制服务')
    args=parser.parse_args(argv)
    try:
        if args.mode=='migrate':
            if args.pending_calibration or args.site is not None:
                raise ValueError('migrate只使用已校验的来源版本记录')
            if args.source_entry is None:raise ValueError('migrate需要--source-entry')
            result=migrate(args.entry,args.source_entry)
        else:
            if not args.maintenance:raise ValueError('需--maintenance确认地面拆桨断电维护状态')
            result=configure(args.entry,args.mode,args.site,pending_calibration=args.pending_calibration)
        print('FC01_VISION_MODE_CONFIG_OK '+json.dumps(result,ensure_ascii=False))
        return 0
    except (ValueError,OSError,KeyError,TypeError,subprocess.SubprocessError) as exc:
        print('FC01_VISION_MODE_CONFIG_FAILED: '+str(exc),file=__import__('sys').stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())

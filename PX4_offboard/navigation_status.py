# 导航状态窗口的纯显示规则：区分进程/IPC活性、原生状态新鲜度和各飞控话题新鲜度。
# 任何中文提示都不授予准备、恢复或起飞权限，真正决定仍在 C++ 控制器。
"""Pure display rules for native preparation and its actual action gates."""

import math

# 只对这些原生准备阶段解释地面等待；故障阶段由独立恢复解释器处理。
PREPARATION_PHASES = frozenset(
    (
        'IDLE',
        'SELF_CHECK',
        'PRESTREAM',
        'REQUEST_OFFBOARD',
        'RC_PRESTREAM',
        'RC_READY_WAIT_ARM',
        'RC_ARMED_WAIT_NAVIGATION',
    )
)


# 调用方先核对服务/boot身份；这里仅检查已有控制PID的一致性、IPC异常与状态年龄。
# 返回True不表示飞机传感器仍新鲜，也不替代外层服务身份验证。
def controller_state_current(row, *, age=None):
    """IPC progress stays distinct from the freshness of the aircraft topics."""
    state = row.get('state')
    # age可由显示层传入“原年龄+快照存放时间”；只有未传时才使用快照自己的原始年龄。
    if not isinstance(state, dict) or not state:
        return False
    if row.get('status') in ('STARTING', 'STALE', 'STOPPING', 'STOPPED'):
        return False
    if (
        row.get('ipc_error')
        or row.get('controller_returncode') is not None
        or row.get('retry_in_seconds') is not None
    ):
        return False
    controller_pid, state_pid = row.get('controller_pid'), row.get('state_controller_pid')
    if (controller_pid is not None or state_pid is not None) and controller_pid != state_pid:
        return False
    if controller_pid is not None and state.get('controller_pid', controller_pid) != controller_pid:
        return False
    age = row.get('state_age_seconds') if age is None else age
    return type(age) in (int, float) and math.isfinite(age) and 0 <= age <= 3


# 把读取快照后经过的时间加到各话题自身年龄；不能用新显示刷新旧测量的TTL。
def aircraft_feedback_current(state, elapsed):
    """Advance native sample ages without extending their reported lifetimes."""
    finite = lambda value: type(value) in (int, float) and math.isfinite(value)
    # elapsed是监督器接收这条原生状态后累计的显示年龄，单位秒，不是Unix时间戳。
    # 原生明确拒绝位先否决；缺字段也不能用默认0填补，后续必须具备每路完整timing。
    if not finite(elapsed) or elapsed < 0:
        return False
    if any(
        state.get(name) is False
        for name in (
            'status_fresh',
            'land_fresh',
            'local_fresh',
            'attitude_fresh',
            'local_sample_fresh',
            'attitude_sample_fresh',
            'battery_fresh',
            'source_clock_valid',
        )
    ):
        return False
    timings = state.get('source_timing')
    if not isinstance(timings, dict):
        return False
    # 每个关键反馈分别检查 envelope/receipt，局部位置和姿态还检查原样本及样本推进年龄。
    for topic in ('status', 'land', 'local', 'attitude', 'flags', 'battery'):
        item = timings.get(topic)
        if not isinstance(item, dict):
            return False
        timeout = item.get('timeout_s')
        if (
            not finite(timeout)
            or timeout <= 0
            or item.get('source_reason') != 'VALID'
            or item.get('receipt_reason') != 'VALID'
        ):
            return False
        names = ('envelope_age_s', 'receipt_age_s')
        if topic in ('local', 'attitude'):
            names += ('sample_age_s', 'sample_progress_age_s')
        for name in names:
            age = item.get(name)
            if not finite(age) or (name in ('receipt_age_s', 'sample_progress_age_s') and age < 0):
                return False
            # 原生VALID允许源时钟略领先；负源年龄按0消耗显示寿命，不能借此延长TTL。
            if max(0.0, age) + elapsed >= timeout:
                return False
    return True


# 根据明确观测列出等待原因；类型未知时不猜测档位、解锁或地面状态。
def preparation_wait_details(state):
    """Explain observed blockers; native code alone decides admission."""
    if state.get('phase') not in PREPARATION_PHASES:
        return []
    details = []
    # 返回新的中文提示列表，state只读；缺少事实时保留未知，不据推测让用户解锁。
    for name, text in (
        ('status_fresh', '飞控状态反馈过期'),
        ('land_fresh', '接地反馈过期'),
        ('local_fresh', '位置/垂直速度反馈过期'),
        ('attitude_fresh', '姿态反馈过期'),
        ('source_clock_valid', '源消息时间戳过期或无效'),
        ('rc_link_ready', '遥控源/链路未就绪'),
        ('rc_switches_ready', '遥控开关反馈未就绪'),
        ('preflight', '飞控预检未通过'),
    ):
        if state.get(name) is False:
            details.append(text)
    for topic, label in (('local', '位置/垂直速度'), ('attitude', '姿态')):
        if state.get(topic + '_sample_fresh') is False and state.get(topic + '_fresh') is not False:
            details.append(label + '采样过期')
    if state.get('battery_fresh') is False or state.get('battery_connected') is False:
        details.append('电池反馈未就绪')
    if state.get('rc_axes_safe') is False or state.get('rc_sticks_moving') is True:
        details.append('油门/摇杆未归位')
    reason = state.get('reason', '')
    ground_wait = reason in (
        'RC_WAIT_DISARMED_STABILIZED_ARM_OFF_NAV_OFF_GROUND',
        'RC_WAIT_CONTINUOUS_SAFE_GROUND',
        'RC_WAIT_SAFE_GROUND',
        'RC_PRESTREAM_GROUND_CHANGED',
        'RC_READY_GROUND_CHANGED',
    )
    # 仅对已知地面等待原因解释Stabilized、CH5上锁、CH6有效非导航档和连续静止窗口。
    if ground_wait:
        for name, expected, text in (
            ('armed', False, '飞控仍解锁'),
            ('landed', True, '未确认接地'),
            ('at_rest', True, '未确认静止'),
        ):
            if type(state.get(name)) is bool and state[name] is not expected:
                details.append(text)
        if type(state.get('nav_state')) is int and state['nav_state'] != 15:
            details.append('未处于 Stabilized')
        if type(state.get('rc_arm_switch')) is int and state['rc_arm_switch'] != 3:
            details.append('CH5 未复位至上锁')
        slot, navigation_slot = state.get('rc_mode_slot'), state.get('rc_navigation_mode_slot')
        if type(slot) is int and (not 1 <= slot <= 6 or slot == navigation_slot):
            details.append('CH6 未处于有效非导航档')
        elapsed, required = state.get('stationary_seconds'), state.get('required_ground_seconds')
        finite = lambda value: type(value) in (int, float) and math.isfinite(value)
        if finite(elapsed) and finite(required) and elapsed < required:
            details.append(f'持续静止不足（{elapsed:.2f}/{required:.2f} 秒）')
        vz, limit = state.get('ground_vz_ned'), state.get('ground_vz_tolerance')
        if finite(vz) and finite(limit) and abs(vz) > limit:
            details.append(f'垂直速度超出地面阈值（{vz:.3f} / ±{limit:.3f} m/s）')
        if not details:
            details.append('等待原生地面/遥控条件确认；本次状态未指明缺失项')
    elif reason in (
        'RC_PRESTREAM_WAIT_NEW_GROUND_REFERENCE',
        'RC_PRESTREAM_REBUILD_GROUND_REFERENCE',
    ):
        samples, elapsed = state.get('rc_ground_reference_samples'), state.get(
            'rc_ground_reference_seconds'
        )
        seconds = (
            f'{elapsed:.2f} 秒'
            if type(elapsed) in (int, float) and math.isfinite(elapsed)
            else '时间未知'
        )
        details.append(
            f'等待新的地面参考样本（{samples if type(samples) is int else "数量未知"}，{seconds}）'
        )
    return details


# 对故障快照按话题列举过期及独立新鲜落地/状态证据；某路过期不能掩盖另一新鲜不安全观测。
def recovery_wait_details(state):
    """Explain observations in the native snapshot, without granting recovery."""
    if state.get('phase') != 'SAFETY':
        return []
    details = []
    # 只解释传入的那次快照；外层只有native_current时才把这些文字作为当前恢复观察展示。
    finite = lambda value: type(value) in (int, float) and math.isfinite(value) and value >= 0
    timings = state.get('source_timing')
    if isinstance(timings, dict):
        for topic, label in (
            ('local', '位置/垂直速度'),
            ('attitude', '姿态'),
            ('status', '飞控状态'),
            ('land', '接地'),
            ('flags', '失效保护'),
            ('battery', '电池'),
        ):
            item = timings.get(topic)
            if not isinstance(item, dict) or item.get('source_reason') not in (
                'ENVELOPE_EXPIRED',
                'SAMPLE_EXPIRED',
            ):
                continue
            ages = [item.get(name) for name in ('envelope_age_s', 'sample_age_s')]
            ages = [value for value in ages if finite(value)]
            limit = item.get('timeout_s')
            text = label + '消息过期'
            if ages and finite(limit) and limit > 0:
                text += f'（消息年龄 {max(ages):.3f} 秒，阈值 {limit:.3f} 秒）'
            details.append(text)
            queued = item.get('rmw_to_callback_s')
            if finite(queued):
                details.append(f'{label}中间件接收至回调间隔 {queued:.3f} 秒')
    # 分别检查每路新鲜度：local过期也不能掩盖新鲜接地检测器报告的at_rest=false。
    if state.get('land_fresh') is True:
        if state.get('landed') is False:
            details.append('飞控尚未确认接地（landed=false）')
        if state.get('at_rest') is False:
            details.append('飞控尚未确认静止（at_rest=false）')
    if state.get('status_fresh') is True:
        if state.get('armed') is True:
            details.append('飞控仍解锁')
        if state.get('px4_user_takeover_reported') is True:
            details.append('飞控仍报告人工接管有效')
    return details


# 原生准备阶段不等于飞行授权；明确关闭的动作门应显示为诊断禁止起飞。
def preparation_diagnostic_reason(state):
    """A native phase describes feedback progress, not permission to take off."""
    if state.get('phase') not in PREPARATION_PHASES:
        return ''
    closed = [
        name + '=false'
        for name in ('flight_actions_enabled', 'takeoff_enabled')
        if state.get(name) is False
    ]
    return '诊断模式：' + '、'.join(closed) + '；本次禁止起飞和导航。' if closed else ''


# 只根据RC路线三门提示人工步骤；arm_enabled=false是人工解锁合同，不应被误认为故障。
def manual_rc_guidance_allowed(state):
    # arm_enabled=false禁用程序解锁，正是人工RC路线的要求；flight_enabled属于另一控制入口。
    # 返回值只控制提示文案；render_status还须先通过服务、原生状态和飞机反馈三层新鲜度检查。
    return all(
        state.get(name) is True
        for name in ('flight_actions_enabled', 'takeoff_enabled', 'rc_navigation_enabled')
    )


def snapshot_provenance_current(row, info):
    """严格核对状态文件属于当前boot/监督器/控制器；不读取硬件、不授予动作。

    service_info由外层只读systemd取得。缺少任何身份都只能算未知，不能用同PID
    字样、mtime或历史READY补齐。IPC和状态年龄仍由controller_state_current独立检查。
    """
    state = row.get('state')
    if not isinstance(state, dict) or not state:
        return False
    if info.get('ActiveState') not in ('active', 'activating', 'deactivating'):
        return False
    supervisor = row.get('supervisor_pid')
    controller = row.get('controller_pid')
    state_controller = row.get('state_controller_pid')
    native_controller = state.get('controller_pid')
    if not all(type(pid) is int and pid > 0 for pid in
               (supervisor, controller, state_controller, native_controller)):
        return False
    if str(supervisor) != info.get('MainPID') or not controller == state_controller == native_controller:
        return False
    boot = row.get('boot_id')
    return (isinstance(boot, str) and bool(boot) and
            isinstance(info.get('boot_id'), str) and boot == info['boot_id'])


def manual_rc_readiness_reason(state, displayed_status):
    """返回当前人工RC准备提示的拒绝原因；空串也仅表示文案证据一致。

    这是显示交叉核对，不重算或放宽原生准入。READY必须绑定当前原生阶段，
    原生零点/连续地面证据及最终零推力预发送；目标检测与任务计时不参与准备。
    显示层不能由缺字段推断“默认安全”。
    """
    if displayed_status not in ('READY_WAIT_ARM', 'WAIT_NAVIGATION'):
        return ''
    expected_phase = ('RC_READY_WAIT_ARM' if displayed_status == 'READY_WAIT_ARM'
                      else 'RC_ARMED_WAIT_NAVIGATION')
    if state.get('phase') != expected_phase:
        return '准备标签与当前原生阶段不一致'
    if not manual_rc_guidance_allowed(state):
        return '原生飞行动作门未确认'
    for name in ('rc_link_ready', 'rc_switches_ready', 'rc_source_selected',
                 'rc_source_clock_valid', 'rc_switch_clock_valid', 'rc_axes_safe',
                 'source_clock_validated', 'source_clock_valid'):
        if state.get(name) is not True:
            return '人工RC准备证据缺失或无效：' + name
    if state.get('rc_sticks_moving') is not False or state.get('rc_kill_switch') != 3:
        return '摇杆未静止或Kill未明确复位'
    if state.get('rc_kill_latched') is not False or state.get('operator_takeover_latched') is not False:
        return '急停或人工接管状态未明确解除'
    if state.get('height_reference_ready') is not True or state.get('rc_ground_reference_qualified') is not True:
        return '本轮地面零点或连续地面证据尚未确认'
    commits = state.get('rc_prestream_commits')
    if state.get('px4_takeoff_position_hold') is True:
        anchor = state.get('takeoff_position_anchor_ned')
        if (type(commits) is not int or commits <= 0 or state.get('heartbeat_enabled') is not True
                or state.get('attitude_output_enabled') is not False
                or not isinstance(anchor, list) or len(anchor) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in anchor)):
            return '尚无本轮位置模式心跳或有效起飞点证据'
    else:
        thrust = state.get('last_target_local_thrust_z')
        if (type(commits) is not int or commits <= 0 or type(thrust) not in (int, float)
                or not math.isfinite(thrust) or thrust != 0.0):
            return '尚无本轮零推力预发送提交证据'
    slot = state.get('rc_mode_slot')
    navigation_slot = state.get('rc_navigation_mode_slot')
    if type(navigation_slot) is not int or navigation_slot != 1:
        return '当前人工RC导航槽位未确认为1'
    if type(slot) is not int or not 1 <= slot <= 6:
        return '当前RC档位无效'
    if displayed_status == 'READY_WAIT_ARM':
        if (state.get('reason') != 'RC_READY_WAIT_CH5_MANUAL_ARM' or
                state.get('armed') is not False or state.get('nav_state') != 15 or
                state.get('landed') is not True or state.get('at_rest') is not True or
                state.get('rc_arm_switch') != 3 or slot == navigation_slot):
            return '当前尚未满足未解锁Stabilized非导航档和接地静止条件'
        if 'ready_wait_arm' in state and state.get('ready_wait_arm') is not True:
            return '原生当前READY证据未成立'
    else:
        # 原生负责8s固定窗口以及异步解锁/Offboard配对；显示不凭墙钟重建窗口。
        # 新版明确给出剩余窗口时才解释其数值，未知不补成8s。
        if (state.get('armed') is not True or state.get('rc_arm_switch') != 1 or
                state.get('rc_manual_arm_proven') is not True or
                state.get('landed') is not True or state.get('nav_state') not in (14, 15)):
            return '人工解锁或本轮地面导航等待证据尚未确认'
        remaining = state.get('rc_arm_to_navigation_remaining_s')
        if remaining is not None and (type(remaining) not in (int, float) or
                                      not math.isfinite(remaining) or not 0 <= remaining <= 8):
            return '原生解锁到导航窗口已过期或无效'
        if state.get('reason') in ('RC_NAVIGATION_SELECTION_TIMEOUT', 'RC_MANUAL_ARM_EVIDENCE_TIMEOUT'):
            return '原生解锁到导航窗口已过期或无效'
    return ''


def vision_runtime_current(value, info, *, now, monotonic):
    """观察必须属于当前服务和本次开机，单调时钟年龄不超过3秒。

    同一boot的单调时钟不受NTP或系统时间调整影响。相机/任务的原接收时刻独立保留，
    外壳重写不能续鲜子项。这个文件只用于显示，不能授予飞行动作。
    """
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value['schema_version'] != 1):
        return False
    if (info.get('ActiveState') != 'active' or
            str(value.get('supervisor_pid')) != str(info.get('MainPID')) or
            not value.get('boot_id') or value.get('boot_id') != info.get('boot_id')):
        return False
    stamp = value.get('receive_monotonic_ns')
    return (type(stamp) is int and type(monotonic) in (int, float)
            and math.isfinite(monotonic) and 0 < stamp <= monotonic * 1e9
            and monotonic * 1e9 - stamp <= 3e9)


def vision_observation(value, name, *, monotonic, timeout=1.0):
    """原接收TTL内且只有一个发布者的诊断才显示；缺失/多源/过期保持未知。"""
    observations = value.get('observations') if isinstance(value, dict) else None
    item = observations.get(name) if isinstance(observations, dict) else None
    if (not isinstance(item, dict) or not isinstance(item.get('value'), dict)
            or type(item.get('publisher_count')) is not int or item['publisher_count'] != 1):
        return None
    stamp = item.get('receive_monotonic_ns')
    if (type(stamp) is not int or type(monotonic) not in (int, float)
            or not math.isfinite(monotonic) or not 0 < stamp <= monotonic * 1e9
            or monotonic * 1e9 - stamp > timeout * 1e9):
        return None
    return item['value']


def vision_display_lines(value, info, *, now, monotonic):
    """只读视觉显示；相机ready、连续目标锁定和原生到高权限分别表达。"""
    if not isinstance(value, dict) or not value:
        return ['视觉观察：尚无当前服务状态；相机/目标锁定未知']
    current = vision_runtime_current(value, info, now=now, monotonic=monotonic)
    mode = value.get('mode', '未知')
    if not current:
        return [f'视觉观察（历史/STALE）：模式 {mode}；相机和目标锁定当前未知']
    if mode == 'altitude':
        return ['视觉模式：altitude（原生1.5m定高；相机/任务未启用）']
    lines = [f'视觉模式：{mode}；观察文件当前，不能代替原生控制授权',
             '视觉标定：'+str(value.get('calibration_state','未知'))+
             '；mission control_enabled='+str(value.get('mission_control_enabled','未知'))]
    camera = vision_observation(value, 'camera', monotonic=monotonic)
    mission = vision_observation(value, 'mission', monotonic=monotonic)
    if camera is None:
        lines.append('相机：无新鲜诊断（未连接/尚未接收）；标定与图像有效性未知')
    else:
        ready = str(camera['ready']).lower() if type(camera.get('ready')) is bool else '未知'
        lines.append('相机：ready=' + ready + '；原因=' + str(camera.get('reason', '未知'))
                     + '；配置=' + str(camera.get('configuration', '未知')))
        missing = camera.get('missing')
        if isinstance(missing, list) and missing:
            lines.append('相机缺项：' + '；'.join(str(x) for x in missing))
    if mission is None:
        lines.append('目标锁定：未知（任务诊断过期或未接收）')
    else:
        locked = str(mission['target_locked']).lower() if type(mission.get('target_locked')) is bool else '未知'
        lines.append('目标锁定：' + locked + '；连续新帧=' + str(mission.get('continuous_new_frames', '未知'))
                     + '；原因=' + str(mission.get('target_reason', '未知')))
        missing = mission.get('missing')
        if isinstance(missing, list) and missing:
            lines.append('视觉等待：' + '；'.join(str(x) for x in missing))
    if value.get('error'):
        lines.append('视觉软件：' + str(value['error']))
    return lines

"""原生飞行阶段的只读接续门；不创建ROS、请求解锁或生成高度。

原生控制器是离地/到高证据的唯一生产者。这里复核来源、时间和声明的一致性，
不能以任务计时、目标检测、历史READY字符串或local绝对z代替原生授权。
"""
import math
from .sensor_contract import stamp_ns


class NativeFlightGate:
    def __init__(self, boot_id, timeout=.3):
        if not boot_id or not math.isfinite(timeout) or not .05 <= timeout <= 1.:
            raise ValueError('NATIVE_GATE_POLICY_INVALID')
        self.boot_id, self.timeout = boot_id, timeout
        self.identity = None
        self.retired = set()
        self.sequence = 0
        self.stamp = 0
        self.received = 0.
        self.local_sample = 0
        self.local_progress = 0.
        self.latest = None
        self.reason = 'NATIVE_FLIGHT_STATE_NOT_RECEIVED'
        self.generation = 0
        self.now_ros = 0
        self.now_mono = 0.
        self.clock_fault = False

    def invalidate(self, reason):
        self.latest = None
        self.reason = reason
        return False

    def clock(self, ros_ns, mono):
        if ros_ns < self.now_ros or mono < self.now_mono:
            self.clock_fault = True
        self.now_ros,self.now_mono = ros_ns,mono
        if self.clock_fault:
            return self.invalidate('NATIVE_HOST_CLOCK_REGRESSED_RESTART_REQUIRED')
        return True

    def accept(self, msg, ros_ns, mono, publisher_count=1):
        if not self.clock(ros_ns,mono):
            return False
        if publisher_count != 1:
            return self.invalidate('NATIVE_PUBLISHER_NOT_UNIQUE')
        if (msg.schema_version != 1 or msg.boot_id != self.boot_id or
                not msg.source_id or len(msg.source_id.encode('utf-8')) > 256 or
                int(msg.controller_pid) <= 0 or msg.profile != 'attitude_altitude'):
            return self.invalidate('NATIVE_PROVENANCE_INVALID')
        try:
            source=stamp_ns(msg.stamp)
        except ValueError:
            return self.invalidate('NATIVE_STAMP_INVALID')
        seq=int(msg.sequence)
        if seq <= 0 or source <= 0:
            return self.invalidate('NATIVE_MEASUREMENT_ID_INVALID')
        local=int(msg.local_sample_stamp_us)
        age_ns=ros_ns-source
        receive_ns=round(mono*1e9)-int(msg.native_monotonic_ns)
        local_receive_ns=round(mono*1e9)-int(msg.local_receive_monotonic_ns)
        if (not -20_000_000 <= age_ns <= self.timeout*1e9 or
                not 0 <= receive_ns <= self.timeout*1e9 or
                not 0 <= local_receive_ns <= self.timeout*1e9 or
                local <= 0 or not -20_000 <= ros_ns//1000-local <= self.timeout*1e6):
            return self.invalidate('NATIVE_SOURCE_OR_RECEIVE_STALE')
        # 先核验原始时间再退休旧身份：迟到的未知会话只能撤销本次授权，
        # 不能把仍在工作的真实会话永久加入retired，导致后续新反馈无法恢复。
        key=(msg.source_id,msg.boot_id,int(msg.controller_pid),int(msg.task_epoch),
             int(msg.z_reset_counter),int(msg.vz_reset_counter),int(msg.quat_reset_counter))
        if key in self.retired:
            return self.invalidate('NATIVE_RETIRED_SESSION')
        if key != self.identity:
            if len(self.retired) >= 64:
                return self.invalidate('NATIVE_RETIRED_SESSION')
            changed=self.identity is not None
            if changed:self.retired.add(self.identity)
            self.identity=key
            self.sequence=self.stamp=self.local_sample=0
            self.received=self.local_progress=0.
            self.generation+=1
            self.latest=None
        else:
            changed=False
        if seq < self.sequence or source < self.stamp:
            # 同一进程/任务的时钟或序号回退不得靠下一条稍新的消息自行恢复。
            self.retired.add(key)
            return self.invalidate('NATIVE_SOURCE_REGRESSED')
        if seq == self.sequence or source == self.stamp:
            # 真正的DDS重复必须保持身份、时间和安全内容一致。不能让同seq/stamp
            # 却已上锁、离开Offboard或高度变化的矛盾消息继续借用旧latest的true。
            # 完全相同的重复只保留原TTL，绝不更新source/receive/local_progress。
            fields=('native_monotonic_ns','local_receive_monotonic_ns','local_sample_stamp_us',
                    'feedback_current','publication_current','navigation_allowed',
                    'target_height_settled','kill','failsafe','operator_takeover',
                    'rc_navigation_enabled','rc_manual_arm_proven','ground_reference_valid',
                    'ground_reference_qualified','armed','offboard','landed','liftoff_confirmed',
                    'rc_navigation_mode_slot','rc_mode_slot','phase','reference_z_ned','z_ned',
                    'vz_ned','relative_height_m','target_height_m','height_tolerance_m',
                    'speed_tolerance_mps','settle_seconds')
            if (self.latest is None or seq != self.sequence or source != self.stamp or
                    any(getattr(msg,name) != getattr(self.latest,name) for name in fields)):
                return self.invalidate('NATIVE_DUPLICATE_INCONSISTENT')
            return self.current(ros_ns,mono)
        self.sequence,self.stamp,self.received=seq,source,mono
        if local < self.local_sample:
            self.retired.add(key)
            return self.invalidate('NATIVE_LOCAL_SAMPLE_REGRESSED')
        if local > self.local_sample:
            self.local_sample,self.local_progress=local,mono
        if changed:
            return self.invalidate('NATIVE_SESSION_CHANGED_WAIT_NEW_SNAPSHOT')
        self.latest=msg
        return self.current(ros_ns,mono)

    def current(self, ros_ns, mono):
        if not self.clock(ros_ns,mono):
            return False
        if self.latest is None:
            return False
        if (not 0 <= mono-self.received <= self.timeout or
                not -20_000_000 <= ros_ns-self.stamp <= self.timeout*1e9 or
                not 0 <= mono-self.local_progress <= self.timeout):
            return self.invalidate('NATIVE_STATE_STALE')
        m=self.latest
        # 每次使用授权都重新检查底层测量，而不只检查上次accept时的年龄。
        # 否则到达时已0.29s的样本可能再借receive TTL延长0.3s。
        local_age_us=ros_ns//1000-int(m.local_sample_stamp_us)
        native_age_ns=round(mono*1e9)-int(m.native_monotonic_ns)
        local_receive_age_ns=round(mono*1e9)-int(m.local_receive_monotonic_ns)
        if (not -20_000 <= local_age_us <= self.timeout*1e6 or
                not 0 <= native_age_ns <= self.timeout*1e9 or
                not 0 <= local_receive_age_ns <= self.timeout*1e9):
            return self.invalidate('NATIVE_UNDERLYING_MEASUREMENT_STALE')
        # Native reset/reference/phase判断仍是准入依据；数值核对只会收紧，不会自造授权。
        values=(m.reference_z_ned,m.z_ned,m.vz_ned,m.relative_height_m,m.target_height_m,
                m.height_tolerance_m,m.speed_tolerance_mps,m.settle_seconds)
        if not all(math.isfinite(float(x)) for x in values):
            self.reason='NATIVE_HEIGHT_FEEDBACK_INVALID'
            return False
        if (abs(float(m.target_height_m)-1.5)>1e-6 or
                not 0 < m.height_tolerance_m <= .5 or not 0 < m.speed_tolerance_mps <= 1. or
                m.settle_seconds <= 0 or
                abs((m.reference_z_ned-m.z_ned)-m.relative_height_m)>1e-5 or
                abs(m.relative_height_m-m.target_height_m)>m.height_tolerance_m or
                abs(m.vz_ned)>m.speed_tolerance_mps):
            self.reason='NATIVE_HEIGHT_TARGET_OR_STABILITY_MISMATCH'
            return False
        required=(m.feedback_current,m.publication_current,m.rc_navigation_enabled,
                  m.rc_manual_arm_proven,m.ground_reference_valid,m.ground_reference_qualified,
                  m.armed,m.offboard,m.liftoff_confirmed,m.target_height_settled,m.navigation_allowed)
        if (not all(required) or m.landed or m.kill or m.operator_takeover or m.failsafe or
                m.rc_navigation_mode_slot != 1 or m.rc_mode_slot != 1 or
                m.phase not in ('HEIGHT_HOLD','HeightHold')):
            self.reason='NATIVE_NAVIGATION_NOT_ALLOWED:'+str(m.reason)
            return False
        self.reason='NATIVE_NAVIGATION_CURRENT'
        return True

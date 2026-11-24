"""PX4实际反馈的纯校验/坐标模块；不导入ROS、不产生飞行命令。

同一条DDS外壳的新timestamp不能给重复timestamp_sample续鲜。每路同时检查
源时钟年龄和本机monotonic进展年龄；局部原点/姿态reset会产生显式撤销快照。
"""

from dataclasses import dataclass, field
import math

POSITION = 1
VELOCITY = 2
ATTITUDE = 4
ANGULAR_VELOCITY = 8
NAN3 = (math.nan,) * 3
NAN4 = (math.nan,) * 4


def finite_vector(values, count):
    """严格要求完整有限向量，缺测不补0。"""
    return len(values) == count and all(math.isfinite(float(x)) for x in values)


def ned_to_enu(values):
    """NED米/米每秒转ENU，保留PX4原local原点，不做起飞地面归零。"""
    if not finite_vector(values, 3):
        raise ValueError('nonfinite NED vector')
    return (float(values[1]), float(values[0]), -float(values[2]))


def frd_to_flu(values):
    """机体前右下转前左上，适用于实际角速度rad/s。"""
    if not finite_vector(values, 3):
        raise ValueError('nonfinite FRD vector')
    return (float(values[0]), -float(values[1]), -float(values[2]))


def multiply_wxyz(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (
        w * v - x * i - y * j - z * k,
        w * i + x * v + y * k - z * j,
        w * j - x * k + y * v + z * i,
        w * k + x * j - y * i + z * v,
    )


def quaternion_ned_frd_to_enu_flu(values):
    """PX4 Hamilton wxyz(FRD->NED)转ROS xyzw(FLU->ENU)。

    R_enu_flu = R_enu_ned * R_ned_frd * R_frd_flu；不是只交换四元数数组。
    接受接近单位长度的PX4量并归一化；零/NaN或明显非单位量直接拒绝。
    """
    if not finite_vector(values, 4):
        raise ValueError('nonfinite quaternion')
    norm = math.sqrt(sum(float(x) ** 2 for x in values))
    if not 0.95 <= norm <= 1.05:
        raise ValueError('quaternion norm invalid')
    q = tuple(float(x) / norm for x in values)
    root = math.sqrt(0.5)
    # NED->ENU为绕(1,1,0)转pi；FLU->FRD为绕x转pi。
    result = multiply_wxyz(multiply_wxyz((0.0, root, root, 0.0), q), (0.0, 1.0, 0.0, 0.0))
    return (result[1], result[2], result[3], result[0])


@dataclass
class Stream:
    envelope_ns: int = 0
    sample_ns: int = 0
    received_ns: int = 0
    publisher_id: str = ''
    payload: dict = field(default_factory=dict)
    valid: bool = False
    reason: str = 'MISSING'
    regressed: bool = False

    def observe(
        self,
        envelope_us,
        sample_us,
        host_ns,
        monotonic_ns,
        publisher_id,
        payload,
        valid,
        timeout_ns,
        future_ns,
    ):
        """只在raw样本推进时更新接收TTL；invalid与回退立即撤销旧载荷。"""
        envelope = int(envelope_us) * 1000
        sample = int(sample_us) * 1000
        self.publisher_id = publisher_id
        if envelope <= 0 or sample <= 0 or sample > envelope or monotonic_ns <= 0 or host_ns <= 0:
            self.valid = False
            self.reason = 'INVALID_TIMESTAMP'
            return False
        if (
            sample > host_ns + future_ns
            or envelope > host_ns + future_ns
            or host_ns - sample > timeout_ns
            or host_ns - envelope > timeout_ns
        ):
            self.valid = False
            self.reason = 'SOURCE_TIME_STALE_OR_FUTURE'
            return False
        if self.sample_ns and (sample < self.sample_ns or envelope < self.envelope_ns):
            self.regressed = True
        if self.regressed:
            self.valid = False
            self.reason = 'SOURCE_TIME_REGRESSED'
            return False
        advanced = sample > self.sample_ns
        self.envelope_ns = max(self.envelope_ns, envelope)
        if advanced:
            # 无效新测量也推进水位；不能随后拿同一采样戳的“好载荷”复活/续鲜。
            self.sample_ns = sample
            self.received_ns = monotonic_ns
            self.payload = payload
        if not valid:
            self.valid = False
            self.reason = 'INVALID_PAYLOAD'
            return False
        if not advanced:
            # 外壳可以进展，原始样本不进展时仍使用最初接收时刻，禁止重复续鲜。
            return False
        self.valid = True
        self.reason = 'OK'
        return True

    def fresh(self, host_ns, monotonic_ns, timeout_ns, future_ns):
        return (
            self.valid
            and not self.regressed
            and self.received_ns > 0
            and monotonic_ns >= self.received_ns
            and monotonic_ns - self.received_ns <= timeout_ns
            and host_ns > 0
            and self.sample_ns <= host_ns + future_ns
            and host_ns - self.sample_ns <= timeout_ns
            and self.envelope_ns <= host_ns + future_ns
            and host_ns - self.envelope_ns <= timeout_ns
        )


class PlatformFeedback:
    """持有三路实际状态；snapshot返回可直接映射PlatformState的独立dict。"""

    def __init__(
        self,
        source_id,
        *,
        timeout_s=0.25,
        alignment_s=0.08,
        future_s=0.02,
        require_angular_velocity=True,
    ):
        if (
            not source_id
            or len(source_id.encode('utf-8')) > 256
            or not all(math.isfinite(x) and x > 0 for x in (timeout_s, alignment_s))
            or not math.isfinite(future_s)
            or not 0 <= future_s <= 0.1
        ):
            raise ValueError('invalid platform feedback policy')
        self.source_id = source_id
        self.timeout_ns = int(timeout_s * 1e9)
        self.alignment_ns = int(alignment_s * 1e9)
        self.future_ns = int(future_s * 1e9)
        self.require_angular_velocity = bool(require_angular_velocity)
        self.streams = {name: Stream() for name in ('local', 'attitude', 'angular_velocity')}
        self.identities = {}
        self.retired_publishers = {name: set() for name in self.streams}
        self.local_reset = None
        self.attitude_reset = None
        self.reset_counter = 0
        self.sequence = 0
        self.pending_reset = ''
        self.last_host_ns = 0
        self.last_monotonic_ns = 0
        self.clock_fault = False

    def _reset(self, reason):
        self.reset_counter += 1
        self.streams = {name: Stream() for name in self.streams}
        self.pending_reset = reason

    def _source(self, name, publisher_id, publisher_count):
        if publisher_count != 1 or not publisher_id:
            self.streams[name].valid = False
            self.streams[name].reason = 'PUBLISHER_COUNT_OR_ID_INVALID'
            return False
        previous = self.identities.get(name)
        if previous is not None and previous != publisher_id:
            if (
                publisher_id in self.retired_publishers[name]
                or len(self.retired_publishers[name]) >= 32
            ):
                self.streams[name].valid = False
                self.streams[name].reason = 'RETIRED_OR_TOO_MANY_PUBLISHERS'
                return False
            self.retired_publishers[name].add(previous)
            self._reset('PX4_SOURCE_SESSION_CHANGED')
        self.identities[name] = publisher_id
        return True

    def local(self, message, host_ns, monotonic_ns, publisher_id, publisher_count=1):
        """接收VehicleLocalPosition；位置/速度有效标记分别保留，不假定独立测量。"""
        if not self._source('local', publisher_id, publisher_count):
            return False
        reset = tuple(
            int(getattr(message, name))
            for name in (
                'xy_reset_counter',
                'z_reset_counter',
                'vxy_reset_counter',
                'vz_reset_counter',
                'heading_reset_counter',
                'ref_timestamp',
            )
        )
        if self.local_reset is not None and reset != self.local_reset:
            self._reset('PX4_LOCAL_COORDINATE_RESET')
        self.local_reset = reset
        pos = (message.x, message.y, message.z)
        vel = (message.vx, message.vy, message.vz)
        position_valid = bool(message.xy_valid and message.z_valid and finite_vector(pos, 3))
        velocity_valid = bool(message.v_xy_valid and message.v_z_valid and finite_vector(vel, 3))
        data = dict(
            position=ned_to_enu(pos) if position_valid else NAN3,
            velocity=ned_to_enu(vel) if velocity_valid else NAN3,
            position_valid=position_valid,
            velocity_valid=velocity_valid,
            resets=reset,
            dead_reckoning=bool(message.dead_reckoning),
            timestamp_us=int(message.timestamp),
            timestamp_sample_us=int(message.timestamp_sample),
        )
        # 一个字段无效不掩盖另一个，但dead_reckoning不能冒充融合定位有效。
        return self.streams['local'].observe(
            message.timestamp,
            message.timestamp_sample,
            host_ns,
            monotonic_ns,
            publisher_id,
            data,
            (position_valid or velocity_valid) and not data['dead_reckoning'],
            self.timeout_ns,
            self.future_ns,
        )

    def attitude(self, message, host_ns, monotonic_ns, publisher_id, publisher_count=1):
        if not self._source('attitude', publisher_id, publisher_count):
            return False
        reset = int(message.quat_reset_counter)
        if self.attitude_reset is not None and reset != self.attitude_reset:
            self._reset('PX4_ATTITUDE_RESET')
        self.attitude_reset = reset
        try:
            quat = quaternion_ned_frd_to_enu_flu(message.q)
            valid = True
        except (ValueError, OverflowError):
            quat, valid = NAN4, False
        return self.streams['attitude'].observe(
            message.timestamp,
            message.timestamp_sample,
            host_ns,
            monotonic_ns,
            publisher_id,
            dict(
                attitude_xyzw=quat,
                quat_reset_counter=reset,
                timestamp_us=int(message.timestamp),
                timestamp_sample_us=int(message.timestamp_sample),
            ),
            valid,
            self.timeout_ns,
            self.future_ns,
        )

    def angular_velocity(self, message, host_ns, monotonic_ns, publisher_id, publisher_count=1):
        if not self._source('angular_velocity', publisher_id, publisher_count):
            return False
        valid = finite_vector(message.xyz, 3)
        data = dict(
            angular_velocity=frd_to_flu(message.xyz) if valid else NAN3,
            timestamp_us=int(message.timestamp),
            timestamp_sample_us=int(message.timestamp_sample),
        )
        return self.streams['angular_velocity'].observe(
            message.timestamp,
            message.timestamp_sample,
            host_ns,
            monotonic_ns,
            publisher_id,
            data,
            valid,
            self.timeout_ns,
            self.future_ns,
        )

    def snapshot(self, host_ns, monotonic_ns, publisher_counts=None):
        """每次发布都重审源/接收TTL和字段对齐；不是把最后好值一直标为ready。"""
        self.sequence += 1
        if (
            self.last_host_ns
            and host_ns < self.last_host_ns
            or self.last_monotonic_ns
            and monotonic_ns < self.last_monotonic_ns
        ):
            self.clock_fault = True
        self.last_host_ns, self.last_monotonic_ns = host_ns, monotonic_ns
        counts = publisher_counts or {}
        # 图中来源消失/争抢立即撤销；数量恢复不能让旧缓存自动复活。
        for name, stream in self.streams.items():
            if counts.get(name, 1) != 1:
                stream.valid = False
                stream.reason = 'PUBLISHER_COUNT_OR_ID_INVALID'
        active = {
            name: s.fresh(host_ns, monotonic_ns, self.timeout_ns, self.future_ns)
            and counts.get(name, 1) == 1
            for name, s in self.streams.items()
        }
        local, att, omega = (self.streams[n] for n in ('local', 'attitude', 'angular_velocity'))
        reasons = []
        required = ['local', 'attitude'] + (
            ['angular_velocity'] if self.require_angular_velocity else []
        )
        for name in required:
            if not active[name]:
                stream = self.streams[name]
                reasons.append(
                    name.upper() + ':' + (stream.reason if not stream.valid else 'STALE')
                )
        # 以实际local测量为基准；姿态/角速度独自新鲜但时刻不匹配时不能混为一帧。
        if active['local']:
            for name in ('attitude', 'angular_velocity'):
                if (
                    active[name]
                    and abs(self.streams[name].sample_ns - local.sample_ns) > self.alignment_ns
                ):
                    active[name] = False
                    if name in required:
                        reasons.append(name.upper() + ':TIME_ALIGNMENT_REQUIRED')
        if self.clock_fault:
            active = dict.fromkeys(active, False)
            reasons.append('HOST_CLOCK_REGRESSED_RESTART_REQUIRED')
        if self.pending_reset:
            active = dict.fromkeys(active, False)
            reasons.append(self.pending_reset)
            self.pending_reset = ''
        mask = 0
        if active['local'] and local.payload.get('position_valid'):
            mask |= POSITION
        else:
            reasons.append('POSITION_INVALID')
        if active['local'] and local.payload.get('velocity_valid'):
            mask |= VELOCITY
        else:
            reasons.append('VELOCITY_INVALID')
        if active['attitude']:
            mask |= ATTITUDE
        if active['angular_velocity']:
            mask |= ANGULAR_VELOCITY
        required_mask = (
            POSITION
            | VELOCITY
            | ATTITUDE
            | (ANGULAR_VELOCITY if self.require_angular_velocity else 0)
        )
        reset = local.payload.get('resets', self.local_reset or (0,) * 6)
        result = dict(
            schema_version=1,
            sequence=self.sequence,
            source_id=self.source_id,
            reset_counter=self.reset_counter,
            origin_id='px4_local:' + ':'.join(str(x) for x in (reset[5], *reset[:5])),
            frame_id='px4_local_enu',
            body_frame_id='base_link_flu',
            ready=(mask & required_mask == required_mask),
            require_angular_velocity=self.require_angular_velocity,
            valid_fields=mask,
            reason=';'.join(dict.fromkeys(reasons)) or 'OK',
            position=local.payload.get('position', NAN3) if mask & POSITION else NAN3,
            velocity=local.payload.get('velocity', NAN3) if mask & VELOCITY else NAN3,
            attitude_xyzw=att.payload.get('attitude_xyzw', NAN4) if mask & ATTITUDE else NAN4,
            angular_velocity=(
                omega.payload.get('angular_velocity', NAN3) if mask & ANGULAR_VELOCITY else NAN3
            ),
            position_stamp_ns=local.sample_ns if mask & POSITION else 0,
            velocity_stamp_ns=local.sample_ns if mask & VELOCITY else 0,
            attitude_stamp_ns=att.sample_ns if mask & ATTITUDE else 0,
            angular_velocity_stamp_ns=omega.sample_ns if mask & ANGULAR_VELOCITY else 0,
            xy_reset_counter=reset[0],
            z_reset_counter=reset[1],
            vxy_reset_counter=reset[2],
            vz_reset_counter=reset[3],
            heading_reset_counter=reset[4],
            quat_reset_counter=att.payload.get('quat_reset_counter', self.attitude_reset or 0),
            ref_timestamp_us=reset[5],
            dead_reckoning=local.payload.get('dead_reckoning', False),
        )
        for name, stream in self.streams.items():
            result[name + '_timestamp_us'] = stream.payload.get('timestamp_us', 0)
            result[name + '_timestamp_sample_us'] = stream.payload.get('timestamp_sample_us', 0)
            result[name + '_receive_monotonic_ns'] = stream.received_ns
            result[name + '_publisher_id'] = stream.publisher_id
        return result

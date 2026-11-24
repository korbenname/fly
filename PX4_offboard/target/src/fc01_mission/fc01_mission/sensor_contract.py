"""制导传感器的纯Python边界：真实时间、身份、插值；不创建ROS端点或飞行权限。

ROS接收器把源戳和本机monotonic时间传进来。重复报文不刷新最后新测量时刻；
无效、重定位或来源更换立即撤销目标，不能以本周期仍收到报文证明测量新鲜。
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


def finite(values, count):
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError('NONFINITE_OR_BAD_SHAPE')
    return result


def quaternion_xyzw(values):
    q = finite(values, 4)
    norm = float(np.linalg.norm(q))
    if not .99 <= norm <= 1.01:
        raise ValueError('ATTITUDE_QUATERNION_INVALID')
    return q/norm


def rotation_xyzw(values):
    x, y, z, w = quaternion_xyzw(values)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def slerp(a, b, ratio):
    a, b = quaternion_xyzw(a), quaternion_xyzw(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = a+ratio*(b-a)
        return q/np.linalg.norm(q)
    theta = math.acos(float(np.clip(dot, -1., 1.)))
    return (math.sin((1-ratio)*theta)*a+math.sin(ratio*theta)*b)/math.sin(theta)


def stamp_ns(stamp):
    if not 0 <= int(stamp.nanosec) < 1_000_000_000:
        raise ValueError('STAMP_NANOSECOND_INVALID')
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def fresh(stamp, now_ns, limit_s, future_s=.02):
    age = (now_ns-stamp)*1e-9
    return stamp > 0 and -future_s <= age <= limit_s


class MeasurementIdentity:
    """同一会话只接受递增序号及源时刻；回退锁存至显式reset或新会话。

    重复帧只返回DUPLICATE，不改水位/接收时间。已退休来源不会重新占回当前会话。
    来源更换本帧用于建立新水位，下一新帧才参与连续锁定，避免跨重启拼接。
    """
    def __init__(self):
        self.identity = None
        self.sequence = 0
        self.stamp = 0
        self.received = 0.
        self.retired = set()
        self.fault = ''

    def observe(self, source, reset, sequence, stamp, received):
        if not isinstance(source, str) or not source or len(source.encode('utf-8')) > 256:
            return 'SOURCE_ID_INVALID'
        key = (source, int(reset))
        if int(sequence) <= 0 or stamp <= 0:
            return 'MEASUREMENT_ID_INVALID'
        if key != self.identity:
            if key in self.retired or len(self.retired) >= 32:
                return 'RETIRED_OR_TOO_MANY_SOURCES'
            changed = self.identity is not None
            if self.identity:
                self.retired.add(self.identity)
            self.identity, self.sequence, self.stamp = key, int(sequence), stamp
            self.received, self.fault = received, ''
            return 'SOURCE_RESET' if changed else 'OK'
        if self.fault:
            return self.fault
        if received < self.received:
            self.fault = 'RECEIVE_CLOCK_REGRESSED'
        elif int(sequence) < self.sequence or stamp < self.stamp:
            self.fault = 'SOURCE_REGRESSED'
        elif int(sequence) == self.sequence or stamp == self.stamp:
            return 'DUPLICATE'
        if self.fault:
            return self.fault
        self.sequence, self.stamp, self.received = int(sequence), stamp, received
        return 'OK'


@dataclass
class AlignedPlatform:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    omega: object
    source_stamp_ns: int


class PlatformHistory:
    """有界保存各真实字段，按图像源时刻插值，不以最新姿态猜测过去姿态。

    保留PX4原local原点，只做桥声明的ENU/FLU转换。origin/session/reset变化清空历史；
    不自动减第一条位置，所以用户航点原点必须另行确认。没有角速度时保持None。
    """
    def __init__(self, timeout=.3, skew=.08, require_omega=True):
        if not 0 < timeout <= 2 or not 0 < skew <= timeout:
            raise ValueError('PLATFORM_TIME_LIMIT_INVALID')
        self.timeout, self.skew, self.require_omega = timeout, skew, require_omega
        self.identity = None
        self.retired = set()
        self.fields = {k:deque(maxlen=160) for k in ('position','velocity','attitude','angular_velocity')}
        self.received = {}
        self.watermarks = {}
        self.latest = None
        self.reason = 'PLATFORM_NOT_RECEIVED'
        self.generation = 0
        self.now_ros = 0
        self.now_mono = 0.
        self.clock_fault = False

    def clear(self, reason):
        for history in self.fields.values():
            history.clear()
        self.received.clear()
        self.latest = None
        self.reason = reason
        self.generation += 1

    def clock(self, now_ns, received):
        if now_ns < self.now_ros or received < self.now_mono:
            self.clock_fault = True
            self.clear('HOST_CLOCK_REGRESSED_RESTART_REQUIRED')
        self.now_ros, self.now_mono = now_ns, received
        return not self.clock_fault

    def accept(self, msg, now_ns, received, publisher_count=1):
        if not self.clock(now_ns, received):
            return False
        if publisher_count != 1:
            self.clear('PLATFORM_PUBLISHER_NOT_UNIQUE')
            return False
        if msg.schema_version != 1 or msg.frame_id != 'px4_local_enu' or msg.body_frame_id != 'base_link_flu':
            self.clear('PLATFORM_SCHEMA_OR_FRAME_INVALID')
            return False
        key = (msg.source_id, int(msg.reset_counter), msg.origin_id)
        if not all(isinstance(v,str) and v and len(v.encode('utf-8'))<=256 for v in (key[0],key[2])):
            self.clear('PLATFORM_ID_INVALID')
            return False
        if key != self.identity:
            if key in self.retired or len(self.retired) >= 32:
                self.clear('PLATFORM_RETIRED_SOURCE')
                return False
            changed = self.identity is not None
            if self.identity:
                self.retired.add(self.identity)
            self.identity = key
            self.watermarks.clear()
            self.clear('PLATFORM_ORIGIN_CHANGED' if changed else 'PLATFORM_START')
        required = 1|2|4|(8 if self.require_omega else 0)
        if not msg.ready or msg.valid_fields & required != required:
            # invalid快照也保留原始测量水位；不能随后重标同一旧测量为有效。
            for name, raw in (('position',msg.local_timestamp_sample_us),
                              ('velocity',msg.local_timestamp_sample_us),
                              ('attitude',msg.attitude_timestamp_sample_us),
                              ('angular_velocity',msg.angular_velocity_timestamp_sample_us)):
                self.watermarks[name] = max(self.watermarks.get(name,0),int(raw)*1000)
            self.clear('PLATFORM_'+str(msg.reason))
            return False
        # 每字段源时刻及其桥接收到原报文的monotonic时间独立检查。
        values = {'position':msg.position, 'velocity':msg.velocity,
                  'attitude':msg.attitude_xyzw, 'angular_velocity':msg.angular_velocity}
        receives = {'position':msg.local_receive_monotonic_ns,
                    'velocity':msg.local_receive_monotonic_ns,
                    'attitude':msg.attitude_receive_monotonic_ns,
                    'angular_velocity':msg.angular_velocity_receive_monotonic_ns}
        # 先事务性消耗整条快照的测量水位，再验证载荷。任何一项NaN/非法四元数
        # 都不能留下“这条新测量尚未见过”的空隙，随后同戳改好载荷不得复活。
        previous = self.watermarks.copy()
        samples = {name:int(getattr(msg,name+'_stamp_ns')) for name in values}
        for name, t in samples.items():
            self.watermarks[name] = max(previous.get(name,0),t)
        staged = []
        try:
            for bit, name in enumerate(values):
                if not msg.valid_fields & (1<<bit):
                    continue
                t = samples[name]
                if not fresh(t, now_ns, self.timeout):
                    raise ValueError(name.upper()+'_SOURCE_STALE')
                # 原始monotonic纳秒用整数作差，避免浮点转换造成负的亚纳秒“未来”。
                bridge_age_ns = round(received*1e9)-int(receives[name])
                if not 0 <= bridge_age_ns <= self.timeout*1e9:
                    raise ValueError(name.upper()+'_RECEIVE_STALE')
                v = quaternion_xyzw(values[name]) if name=='attitude' else finite(values[name],3)
                if t < previous.get(name,0):
                    raise ValueError(name.upper()+'_SOURCE_REGRESSED')
                if t > previous.get(name,0):
                    staged.append((name,t,v.copy()))
            # 所有字段校验结束才写入历史；不能让前几个好字段进入半条快照。
            for bit,name in enumerate(values):
                if not msg.valid_fields & (1<<bit):
                    self.fields[name].clear()
                    self.received.pop(name,None)
            for name,t,v in staged:
                self.fields[name].append((t,v))
                self.received[name] = received
            if not self.current(now_ns,received):
                return False
            self.latest = AlignedPlatform(self.fields['position'][-1][1],self.fields['velocity'][-1][1],
                rotation_xyzw(self.fields['attitude'][-1][1]),
                self.fields['angular_velocity'][-1][1] if self.fields['angular_velocity'] else None,
                self.fields['position'][-1][0])
            self.reason = 'OK'
            return True
        except ValueError as error:
            self.clear(str(error))
            return False

    def current(self, now_ns, received):
        required = ('position','velocity','attitude')+(('angular_velocity',) if self.require_omega else ())
        for name in required:
            h = self.fields[name]
            if not h or not fresh(h[-1][0],now_ns,self.timeout):
                self.reason = name.upper()+'_SOURCE_STALE'
                return False
            if not 0 <= received-self.received.get(name,-1e9) <= self.timeout:
                self.reason = name.upper()+'_RECEIVE_STALE'
                return False
        if max(self.fields[n][-1][0] for n in required)-min(self.fields[n][-1][0] for n in required)>self.skew*1e9:
            self.reason = 'PLATFORM_FIELDS_NOT_ALIGNED'
            return False
        return True

    def aligned(self, stamp):
        def interpolate(name):
            history = self.fields[name]
            for t,v in history:
                if t==stamp:return v.copy()
            for (a,x),(b,y) in zip(history,list(history)[1:]):
                if a<stamp<b and (b-a)*1e-9<=self.skew:
                    k=(stamp-a)/(b-a)
                    return slerp(x,y,k) if name=='attitude' else x+k*(y-x)
            raise ValueError(name.upper()+'_IMAGE_TIME_NOT_BRACKETED')
        return AlignedPlatform(interpolate('position'),interpolate('velocity'),
            rotation_xyzw(interpolate('attitude')),
            interpolate('angular_velocity') if self.require_omega else None,stamp)

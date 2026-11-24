#!/usr/bin/env python3
"""Build and exercise the pinned PX4 position-control code against a point mass.

The optional ROS plant imports the same library adapter so the controller can
consume actual FC01 wire setpoints. No firmware process or device is started.
"""
import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
AUDIT = HERE.parent / 'docs/investigations/20261118_vision_step2/evidence/EKF_AUDIT.json'
PX4_COMMIT = '07983bfb894b9372d7520d43ac10dc792d0b4b9f'
PARAMETERS = (
    'COM_SPOOLUP_TIME', 'COM_THROW_EN', 'MPC_ACC_DECOUPLE', 'MPC_TKO_RAMP_T',
    'MPC_THR_HOVER', 'MPC_THR_MAX', 'MPC_THR_MIN', 'MPC_THR_XY_MARG',
    'MPC_TILTMAX_AIR', 'MPC_TILTMAX_LND', 'MPC_XY_P', 'MPC_XY_VEL_D_ACC',
    'MPC_XY_VEL_I_ACC', 'MPC_XY_VEL_MAX', 'MPC_XY_VEL_P_ACC', 'MPC_Z_P',
    'MPC_Z_VEL_D_ACC', 'MPC_Z_VEL_I_ACC', 'MPC_Z_VEL_MAX_DN',
    'MPC_Z_VEL_MAX_UP', 'MPC_Z_VEL_P_ACC',
)
SOURCE_DIR = Path('src/modules/mc_pos_control')
SOURCES = (
    SOURCE_DIR / 'PositionControl/PositionControl.cpp',
    SOURCE_DIR / 'PositionControl/ControlMath.cpp',
    SOURCE_DIR / 'Takeoff/Takeoff.cpp',
    Path('src/lib/hysteresis/hysteresis.cpp'),
)
TOPICS = ('trajectory_setpoint', 'vehicle_attitude_setpoint',
          'vehicle_local_position_setpoint', 'takeoff_status')
G = 9.80665
NAN = math.nan
TRIPLE = ctypes.c_float * 3


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Px4PositionControl:
    """ctypes adapter for the real PX4 position and takeoff control-law library."""

    def __init__(self, library):
        self.library_path = Path(library).resolve()
        self.parameter_info = json.loads(
            self.library_path.with_suffix('.json').read_text())
        if self.parameter_info['library_sha256'] != sha(self.library_path):
            raise ValueError('PX4 model library SHA256 does not match its receipt')
        self.hover_thrust = self.parameter_info['parameters']['MPC_THR_HOVER']
        self._lib = ctypes.CDLL(str(self.library_path))
        self._lib.fc01_test_px4_create.restype = ctypes.c_void_p
        self._lib.fc01_test_px4_create.argtypes = []
        self._lib.fc01_test_px4_destroy.argtypes = [ctypes.c_void_p]
        self._lib.fc01_test_px4_destroy.restype = None
        self._lib.fc01_test_px4_step.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.c_float, ctypes.POINTER(ctypes.c_float),
            ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.POINTER(ctypes.c_float),
        ]
        self._lib.fc01_test_px4_step.restype = ctypes.c_int
        self._handle = self._lib.fc01_test_px4_create()

    def close(self):
        if self._handle:
            self._lib.fc01_test_px4_destroy(self._handle)
            self._handle = None

    def update(self, dt, position, velocity, acceleration, yaw, *,
               target_position, target_velocity, target_acceleration,
               target_yaw=NAN, target_yaw_speed=NAN, airborne=True,
               armed=True, offboard=True):
        fields = (*target_position, *target_velocity, *target_acceleration,
                  target_yaw, target_yaw_speed)
        target = (ctypes.c_float * 11)(*(NAN if value is None else value for value in fields))
        thrust = TRIPLE()
        result = self._lib.fc01_test_px4_step(
            self._handle, dt, TRIPLE(*position), TRIPLE(*velocity), TRIPLE(*acceleration), yaw,
            target, armed, offboard, airborne, thrust)
        if result:
            raise RuntimeError(f'PX4 PositionControl update failed ({result})')
        return tuple(thrust)


def compile_library(firmware, sitl_build, output):
    commit = subprocess.check_output(
        ['git', '-C', str(firmware), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != PX4_COMMIT:
        raise ValueError(f'PX4 source must be {PX4_COMMIT}, got {commit}')
    # 拒绝改写过的生产求解器及其矩阵、数学、平台依赖。
    checked_paths = ('src/modules/mc_pos_control', 'src/lib/mathlib', 'src/lib/matrix',
                     'src/lib/hysteresis', 'src/lib/geo', 'src/drivers/drv_hrt.h',
                     'src/modules/uORB', 'msg', 'platforms/common', 'platforms/posix')
    changed = subprocess.check_output(
        ['git', '-C', str(firmware), 'status', '--porcelain', '--untracked-files=all',
         '--', *checked_paths], text=True)
    if changed:
        raise ValueError(f'PX4 model source inputs have local changes: {changed}')

    record = next(row for row in json.loads(AUDIT.read_text())['records']
                  if row.get('source', '').endswith('/log_82_UnknownDate.ulg'))
    params = {name: record['all_initial_parameters'][name] for name in PARAMETERS}
    output.mkdir(parents=True, exist_ok=False)
    header = output / 'px4_model_parameters.hpp'
    header.write_text(
        '#pragma once\nnamespace px4_model_parameters {\n' +
        ''.join(f'inline constexpr float {name} = {float(value)!r}f;\n'
                for name, value in params.items()) + '}\n')
    # POSIX 板配置来自本机 SITL 构建，消息布局则取定制板构建，避免 v1.18 消息混入。
    board_build = firmware / 'build/damiao_dm-fc01_default'
    include_dirs = [
        output,
        firmware / 'boards/px4/sitl/src',
        firmware / 'platforms/posix/src/px4/common/include',
        firmware / 'platforms/posix/src/px4/generic/generic/include',
        firmware / 'platforms/posix/include',
        firmware / 'platforms/common',
        firmware / 'platforms/common/include',
        firmware / 'src',
        firmware / 'src/include',
        firmware / 'src/lib',
        firmware / 'src/lib/matrix',
        firmware / 'src/modules',
        firmware / SOURCE_DIR / 'PositionControl',
        firmware / SOURCE_DIR / 'Takeoff',
        sitl_build,
    ]
    generated = [sitl_build / 'px4_boardconfig.h']
    for topic in TOPICS:
        source = board_build / f'uORB/topics/{topic}.h'
        destination = output / f'uORB/topics/{topic}.h'
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not source.is_file():
            raise ValueError(f'missing generated FC01 build header: {source}')
        destination.write_bytes(source.read_bytes())
        generated.extend((source, destination))
    for path in generated:
        if not path.is_file():
            raise ValueError(f'missing generated PX4 build header: {path}')
    wrapper = HERE / 'px4_takeoff_position_model.cpp'
    library = output / 'px4_takeoff_position_model.so'
    command = [
        'c++', '-std=gnu++17', '-O2', '-Wall', '-Wextra', '-Wno-unused-parameter',
        '-fPIC', '-shared', '-Wl,--no-undefined', '-D__PX4_POSIX', '-D__PX4_LINUX',
        '-DCONFIG_ARCH_BOARD_PX4_SITL', '-include', 'visibility.h',
        *(f'-I{path}' for path in include_dirs), '-o', str(library),
        str(wrapper), *(str(firmware / src) for src in SOURCES),
    ]
    subprocess.run(command, check=True)
    receipt = dict(
        px4_commit=commit, firmware=str(firmware), command=command,
        px4_test_scope='PositionControl, ControlMath, Takeoff and Hysteresis as POSIX code',
        parameters=params, parameter_ulog_name=Path(record['source']).name,
        parameter_ulog_sha256=record['sha256'], evidence_sha256=sha(AUDIT),
        library_sha256=sha(library),
        inputs_sha256={str(path): sha(path) for path in [
            header, wrapper, *generated, *(firmware / src for src in SOURCES),
            Path(__file__),
        ]},
    )
    library.with_suffix('.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return library


def standalone(library, output):
    """Numerical smoke check; the ROS test also exercises the real FC01 gates."""
    model = Px4PositionControl(library)
    anchor = (2.25, -1.75, 2.)
    position = list(anchor)
    velocity = [0.] * 3
    acceleration = [0.] * 3
    dt = .01
    records = []
    airborne = False
    rows = 1801
    try:
        for step in range(rows):
            time = step * dt
            yaw = math.pi / 2 if time >= 10. else 0.
            if step == 300:
                # 起飞后注入向北、向西的速度扰动，航向改变后再次注入相同扰动。
                velocity[0] += .45
                velocity[1] -= .35
            if step == 1000:
                velocity[0] += .35
                velocity[1] -= .20
            height_command = min(1.5, max(0., time - 1.) * .30)
            horizontal = airborne and anchor[2] - position[2] >= .12
            target = (*anchor[:2], anchor[2] - height_command) if horizontal else (
                NAN, NAN, anchor[2] - height_command if height_command else NAN)
            target_acceleration = (NAN, NAN, NAN) if horizontal else (
                0., 0., NAN if height_command else G)
            thrust = model.update(
                dt, position, velocity, acceleration, yaw, target_position=target,
                target_velocity=(NAN,) * 3, target_acceleration=target_acceleration,
                target_yaw=0., airborne=airborne, armed=time >= .25, offboard=time >= 1.)
            # PX4 给出 NED 归一化推力。以 hover 为重量对应的标度，并加入线性阻尼。
            for axis in range(3):
                acceleration[axis] = (G * (thrust[axis] / model.hover_thrust + (axis == 2))
                                      - .45 * velocity[axis])
                velocity[axis] += acceleration[axis] * dt
                position[axis] += velocity[axis] * dt
            if position[2] > anchor[2]:
                position[2] = anchor[2]
                velocity[2] = acceleration[2] = 0.
            airborne = airborne or position[2] < anchor[2] - .02
            records.append(dict(
                t_s=time, anchor_ned_m=anchor, position_ned_m=list(position),
                velocity_ned_mps=list(velocity), height_m=anchor[2] - position[2],
                thrust_ned=thrust, yaw_rad=yaw,
                target_ned_m=[value if math.isfinite(value) else None for value in target],
                xy_error_m=math.hypot(position[0] - anchor[0], position[1] - anchor[1]),
            ))
    finally:
        model.close()
    correction = next(row for row in records[300:1000]
                      if row['target_ned_m'][0] is not None and row['thrust_ned'][0] < -.005)
    yaw_correction = records[1000]
    peak = max(row['xy_error_m'] for row in records)
    final = max(row['xy_error_m'] for row in records[-100:])
    overshoot = max(0., max(row['height_m'] for row in records) - 1.5)
    result = dict(
        model='exact PX4 PositionControl + Takeoff; simplified NED point mass',
        fixed_step_s=dt, first_disturbance_s=3., yaw_disturbance_s=10.,
        first_correction=correction, yaw_correction=yaw_correction,
        peak_xy_error_m=peak, last_second_max_xy_error_m=final,
        height_overshoot_m=overshoot, final_height_m=records[-1]['height_m'],
        passed=correction['height_m'] < 1.5 and yaw_correction['thrust_ned'][0] < -.005
        and yaw_correction['target_ned_m'][:2] == list(anchor[:2])
        and final < .05 and abs(records[-1]['height_m'] - 1.5) < .05 and overshoot < .25,
    )
    (output / 'standalone.jsonl').write_text(
        ''.join(json.dumps(row, allow_nan=False) + '\n' for row in records))
    (output / 'standalone.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, required=True)
    parser.add_argument('--sitl-build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-only', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    library = compile_library(args.firmware.resolve(), args.sitl_build.resolve(), output)
    return 0 if args.build_only else standalone(library, output)


if __name__ == '__main__':
    raise SystemExit(main())

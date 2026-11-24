#!/usr/bin/env python3
"""隔离ROS测试监督器：真实standby生命周期，仅替代ARM/UART预检与传输。

不属于安装入口，不打包。HOME必须由测试指定为新的私有目录，domain必须非零
localhost；不接systemd、不接Agent/串口。实际锁、配置写入、spawn、日志泵、IPC
monitor、状态身份和退出清理仍执行生产standby代码，替代项明确写入结果。
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (
        os.environ.get('ROS_LOCALHOST_ONLY') != '1'
        or not 1 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
    ):
        raise RuntimeError('isolated localhost domain required')
    private = Path(os.environ.get('FC01_STEP5_PRIVATE_HOME', '')).resolve()
    if (
        Path.home().resolve() != private
        or 'fc01-step5-' not in str(private)
        or not private.is_dir()
    ):
        raise RuntimeError('explicit isolated temporary HOME required')
    import yaml
    import standby
    from navigation_service import describe_row

    profile = yaml.safe_load(args.config.read_text())
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())

    class LocalDDSTransport:
        """仅说明合成PX4经localhost DDS连接；不伪称真实Agent已连接。"""

        def __init__(self, *unused):
            pass

        def start(self):
            pass

        def snapshot(self):
            return dict(
                agent_pid=None,
                agent_owned=False,
                agent_transport_status='SYNTHETIC_LOCAL_DDS',
                agent_restart_count=0,
            )

        def close(self):
            return None

    replaced = ['ARM64/SDK/UART preflight', 'global hardware process discovery', 'AgentTransport']
    args.output.mkdir(mode=0o700, parents=True, exist_ok=True)
    (args.output / 'boundary.json').write_text(
        json.dumps(
            dict(
                real_components=[
                    'standby.supervise',
                    'launcher lock',
                    'runtime YAML write',
                    '_start_supervised_controller',
                    'LogPump',
                    'IPC monitor',
                    'StatusWriter',
                    'native safe shutdown',
                    'resource cleanup',
                ],
                test_instrumentation='transparent single-owned IPC record tee and bounded command relay',
                substituted_hardware_boundaries=replaced,
                systemd=False,
                uart=False,
                agent=False,
                node=str(args.node.resolve()),
            ),
            indent=2,
        )
        + '\n'
    )
    # 原生只允许一个IPC客户端。保留监督器的真实连接，在测试侧透明记录收到的行；
    # 少量LAND/拒绝自动起飞探针也复用这个连接，绝不新增第二个原生客户端。
    original_connect = standby.connect_controller
    requests = args.output / 'test-requests.jsonl'
    requests.touch(mode=0o600)
    incoming = requests.open()
    observed = (args.output / 'test-ipc.jsonl').open('w')

    class ObservedChannel:
        def __init__(self, channel, pid):
            self.channel, self.pid = channel, pid
            self.request_ids = {}
            self.count = 0

        def __getattr__(self, key):
            return getattr(self.channel, key)

        def read(self):
            # 仅三个测试命令；限制长度/数量，不能形成无界控制队列。
            while True:
                line = incoming.readline(1025)
                if not line:
                    break
                if len(line) > 1024 or not line.endswith('\n') or self.count >= 8:
                    raise RuntimeError('invalid test command queue')
                req = json.loads(line)
                if req['op'] not in ('TAKEOFF', 'LAND'):
                    raise RuntimeError('unsupported test operation')
                seq = self.channel.send(req['op'], req['args'])
                self.request_ids[seq] = req['id']
                self.count += 1
            rows = self.channel.read()
            for row in rows:
                copy = dict(row, _test_controller_pid=self.pid, _test_received=time.monotonic())
                if row.get('kind') == 'reply':
                    copy['_test_request_id'] = self.request_ids.get(row.get('seq'))
                observed.write(json.dumps(copy, allow_nan=False) + '\n')
            observed.flush()
            return rows

    def connect(*values, **kwargs):
        channel = original_connect(*values, **kwargs)
        return ObservedChannel(channel, values[1].pid)

    # 模型在另一进程持有真正DDS发布者。没有新增控制输出或绕过原生飞行门。
    try:
        with (
            patch.object(standby, 'preflight', return_value=(args.node.resolve(), object())),
            patch.object(
                standby, 'wait_for_previous_controller', side_effect=lambda *_: not stopped.is_set()
            ),
            patch.object(standby, 'AgentTransport', LocalDDSTransport),
            patch.object(standby, 'connect_controller', side_effect=connect),
        ):
            standby.supervise(
                ROOT,
                private / 'NO_AGENT',
                args.output,
                stopped,
                controller=args.node.resolve(),
                config_factory=lambda *_: profile,
                status_metadata=dict(
                    runtime_mode='flight-test',
                    entry_version='step5-synthetic',
                    visual_navigation_enabled=profile['fc01_system']['ros__parameters'].get(
                        'visual_navigation_enabled', False
                    ),
                ),
                config_path=args.config,
                row_formatter=describe_row,
            )
    finally:
        incoming.close()
        observed.close()


if __name__ == '__main__':
    main()

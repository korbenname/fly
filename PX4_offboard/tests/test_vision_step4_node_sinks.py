"""本次视觉第4步的真实节点差分：所有控制sink字段逐字节一致。

使用本步开始的2948bfe，驱动原生SystemNode，ROS与时钟为本机内存替身。
新增只读NativeFlightState与稳定窗统计不属于旧版JSON协议字节相等声明；
它们另由真实Humble和门控回归核对，本测试完整保留原始状态以供审查。
"""
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import test_stage2_powered_node_equivalence as powered

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '2948bfe4e6a26b5ee58bf02e4e9988e484fad81e'


class VisionStep4NodeSinkTests(unittest.TestCase):
    def test_manual_rc_lifecycle_preserves_exact_control_sink_sequence(self):
        sys.path.insert(0,str(ROOT/'native/tests'))
        self.addCleanup(lambda:sys.path.remove(str(ROOT/'native/tests')))
        recovery=importlib.import_module('test_rc_recovery_adapter')
        evidence=Path(os.environ.get('FC01_VISION_STEP4_NODE_EVIDENCE','/tmp/fc01-vision-step4-node-sinks'))
        evidence.mkdir(parents=True,exist_ok=True)
        sink_records=[]
        states=[]
        with tempfile.TemporaryDirectory(prefix='fc01_vision_step4_node_') as directory:
            folder=Path(directory)
            baseline=folder/'baseline';baseline.mkdir()
            archive=subprocess.check_output(['git','archive',BASELINE,'native'],cwd=ROOT)
            with tarfile.open(fileobj=io.BytesIO(archive)) as source:
                for member in source.getmembers():
                    self.assertTrue((member.isfile() or member.isdir()) and '..' not in Path(member.name).parts)
                source.extractall(baseline)
            harness=powered.powered_harness(recovery)
            for label,tree in [('baseline',baseline),('candidate',ROOT)]:
                build=folder/('build-'+label);build.mkdir(mode=0o700)
                with patch.object(recovery.fixture,'ROOT',tree/'native'):
                    recovery.write_fixture(build)
                header=build/'rclcpp/rclcpp.hpp'
                header.write_text(powered.replace_once(header.read_text(),
                    'unsigned calls{};void publish(const T&){++calls;};',
                    'unsigned calls{};std::function<void(const T&)> record;'
                    'void publish(const T& value){++calls;if(record)record(value);};'))
                command=build/'px4_msgs/msg/vehicle_command.hpp'
                command.write_text(powered.replace_once(command.read_text(),
                    'float param1{},param2{};',
                    'float param1{},param2{},param3{},param4{},param7{};'
                    'double param5{},param6{};std::uint8_t confirmation{};'))
                driver=build/'driver.cpp';driver.write_text(harness)
                binary=build/'trace'
                argv=['g++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-pthread',
                      '-I',str(build),'-I',str(tree/'native/src/jikl_fc01_system/include'),
                      '-I',str(tree/'native/tests'),str(driver),'-o',str(binary)]
                compiled=subprocess.run(argv,capture_output=True,text=True,timeout=120)
                (evidence/(label+'-compile.log')).write_text(compiled.stdout+compiled.stderr)
                self.assertEqual(compiled.returncode,0,compiled.stdout+compiled.stderr)
                result=subprocess.run([str(binary)],env=dict(os.environ,FC01_SOCKET=str(build/'control.sock')),
                                      capture_output=True,timeout=30)
                (evidence/(label+'-full.log')).write_bytes(result.stdout)
                (evidence/(label+'-stderr.log')).write_bytes(result.stderr)
                self.assertEqual(result.returncode,0,result.stderr.decode(errors='replace'))
                # 只投影明确新增的观察消息；每条实际动力/心跳/离散命令的所有字段及顺序全部保留。
                sinks=b'\n'.join(line for line in result.stdout.splitlines() if line.startswith(
                    (b'POWERED TARGET ',b'POWERED HEARTBEAT ',b'POWERED COMMAND ',b'POWERED COMPLETE ')))
                (evidence/(label+'-sinks.log')).write_bytes(sinks)
                sink_records.append(sinks)
                states.append([json.loads(line.split(b':',1)[1]) for line in result.stdout.splitlines()
                               if line.startswith(b'POWERED STATE ')])
            self.assertTrue(sink_records[0])
            self.assertEqual(sink_records[0],sink_records[1],'actual control sink field/order regression')
            # 兼容字段的控制事实也逐样本比较，不能用只比较publish次数掩盖状态机变化。
            fields=('phase','reason','armed','nav_state','relative_height_m','height_reference_z0',
                    'liftoff_confirmed','target_height_settled','navigation_allowed','rc_manual_arm_proven',
                    'last_target_local_thrust_z','target_local_commits','heartbeat_local_commits')
            projection=lambda rows:[{key:row.get(key) for key in fields} for row in rows]
            self.assertEqual(projection(states[0]),projection(states[1]),'control-state projection regression')
            summary=dict(baseline=BASELINE,comparison='all control sink bytes exact; explicit control-state fields exact',
                lines=len(sink_records[1].splitlines()),state_samples=len(states[1]),
                sink_sha256=hashlib.sha256(sink_records[1]).hexdigest(),state_fields=list(fields),
                driver_sha256=hashlib.sha256(harness.encode()).hexdigest(),
                excluded_observations='additive NativeFlightState and height stable-sample metadata; full logs preserved',
                hardware=False)
            (evidence/'RESULT.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':unittest.main()

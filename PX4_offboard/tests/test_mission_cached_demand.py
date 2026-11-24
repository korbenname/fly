"""复核发现的冻结需求回归：执行实际方法，避免依赖ROS或启动飞控。"""
import ast
from pathlib import Path
import unittest
import numpy as np

SOURCE=Path(__file__).resolve().parents[1]/'target/src/fc01_mission/fc01_mission/mission_node.py'

def implementation():
    tree=ast.parse(SOURCE.read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
    methods=[x for x in cls.body if isinstance(x,ast.FunctionDef) and
             x.name in ('_clear_target','_update_mission','_publish_command','_publish_navigation')]
    module=ast.Module(body=[ast.ClassDef(name='ActualMethods',bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[])
    ast.fix_missing_locations(module)
    scope={'np':np}
    exec(compile(module,str(SOURCE),'exec'),scope)
    return scope['ActualMethods']


class CachedDemandTests(unittest.TestCase):
    def fixture(self,phase,terminal=False):
        n=implementation()()
        n.target_vel_est=np.ones(3)
        n.terminal_cmd=np.array([1.2,-.3,.1])
        n.last_v_cmd=np.ones(3)
        n.terminal_start=10. if terminal else None
        n.flight_state=phase
        n.platform_input_mode='simulation'
        n.guidance_invalidated=False
        n.platform_pos=np.array([10.,20.,2.])
        n.platform_vel=np.zeros(3)
        n.push_aim=np.ones(3)
        n.yaw_rate_ff=1.
        return n

    def test_invalid_target_clears_terminal_and_frozen_phase_demands(self):
        for phase,terminal in [('seek',True),('strike',False),('push',False),('retreat',False)]:
            with self.subTest(phase=phase):
                n=self.fixture(phase,terminal)
                n._clear_target('PLATFORM_ORIGIN_CHANGED')
                self.assertTrue(n.guidance_invalidated)
                self.assertIsNone(n.terminal_start)
                self.assertIsNone(n.push_aim)
                self.assertIsNone(n.est_pos)
                np.testing.assert_array_equal(n.terminal_cmd,[0.,0.,0.])
                np.testing.assert_array_equal(n.last_v_cmd,[0.,0.,0.])
                # 即使平台有效性恢复，旧阶段仍不能自动恢复非零需求。
                for now in (10.2,11.,50.):
                    v,valid=n._update_mission(now,.01)
                    self.assertFalse(valid)
                    np.testing.assert_array_equal(v,[0.,0.,0.])

    def test_initial_missing_data_does_not_latch_unused_mission(self):
        n=self.fixture('takeoff')
        n._clear_target('PLATFORM_NOT_RECEIVED')
        self.assertFalse(n.guidance_invalidated)
        self.assertFalse(n.target_locked)


if __name__=='__main__':unittest.main()

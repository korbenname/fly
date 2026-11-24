"""Read-only: execute selected exact current mission methods without creating ROS endpoints."""
import ast
import json
from pathlib import Path
import numpy as np

path=Path('/home/jikl62/FC01_OrangePi_Navigation_20261028/target/src/fc01_mission/fc01_mission/mission_node.py')
tree=ast.parse(path.read_text())
cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
names={'_clear_target','_update_mission'}
selected=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in names]
module=ast.Module(body=[ast.ClassDef(name='UnderReview',bases=[],keywords=[],body=selected,decorator_list=[])],type_ignores=[])
ast.fix_missing_locations(module)
namespace={'np':np}
exec(compile(module,str(path),'exec'),namespace)
Node=namespace['UnderReview']
node=Node()
node.target_vel_est=np.ones(3)
node.platform_pos=np.array([10.,20.,2.])
node.platform_vel=np.zeros(3)
node.engage_mode='impact'
node.control_enabled=True
node.flight_state='seek'
node.impacted=False
node.terminal_start=10.
node.terminal_timeout_s=3.
node.terminal_cmd=np.array([1.2,-.3,.1])
node.min_altitude_m=.5
node._clear_target('PLATFORM_ORIGIN_CHANGED')
v,valid=node._update_mission(10.2,.01)
result={
 'source':str(path),'scenario':'platform reset invalidates target while terminal command is active',
 'vision_rel_after_clear':node.vision_rel,'target_locked_after_clear':node.target_locked,
 'terminal_start_after_clear':node.terminal_start,'flight_state_after_clear':node.flight_state,
 'next_guidance_velocity':v.tolist(),'next_guidance_valid':valid,
 'reproduced':bool(valid and np.linalg.norm(v)>0),
 'physical_control_exercised':False,
 'production_default_control_enabled':False,
}
print(json.dumps(result,ensure_ascii=False,indent=2))
assert result['reproduced'], 'reviewed behavior changed; rerun review'

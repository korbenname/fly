import ast,json
from pathlib import Path
import numpy as np
p=Path('target/src/fc01_mission/fc01_mission/mission_node.py');tree=ast.parse(p.read_text());c=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='MissionNode');m=next(x for x in c.body if isinstance(x,ast.FunctionDef) and x.name=='_update_mission');mod=ast.Module(body=[ast.ClassDef(name='Original',bases=[],keywords=[],body=[m],decorator_list=[])],type_ignores=[]);ast.fix_missing_locations(mod);ns={'np':np};exec(compile(mod,str(p),'exec'),ns);o=ns['Original']();o.platform_pos=np.array([0.,0.,0.]);o.platform_vel=np.zeros(3);o.guidance_invalidated=False;o.engage_mode='track';o.impacted=False;o.terminal_start=None;o.control_enabled=True;o.flight_state='takeoff';o.takeoff_altitude_m=2.78;o.takeoff_climb_mps=.7;o.min_altitude_m=.2
v,valid=o._update_mission(100.,.01)
print(json.dumps({'production_native_state_received':False,'platform_local_z_enu':0.,'takeoff_altitude_m':o.takeoff_altitude_m,'generated_world_velocity':v.tolist(),'valid':valid,'reproduced_native_authority_bypass':bool(valid and v[2]>.1)}))

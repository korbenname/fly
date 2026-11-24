"""Run exact publishing methods with small observable message doubles, no ROS endpoints."""
import ast,json,math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
source=Path('/home/jikl62/FC01_OrangePi_Navigation_20261028/target/src/fc01_mission/fc01_mission/mission_node.py')
cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='MissionNode')
names={'_publish_command','_publish_navigation','_push_altitude'}
module=ast.Module(body=[ast.ClassDef(name='Actual',bases=[],keywords=[],body=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in names],decorator_list=[])],type_ignores=[])
ast.fix_missing_locations(module)
class Msg(NS):
    CRUISE=1;HOLD=2
class Sink:
    def __init__(self):self.items=[]
    def publish(self,msg):self.items.append(msg)
scope=dict(np=np,math=math,VelocityCommandMsg=Msg,NavigationCommandMsg=Msg)
exec(compile(module,str(source),'exec'),scope)
n=scope['Actual']();n.vision_control_hz=50.;n.last_vision_control_time=0.;n.yaw_cmd=None;n.vision_control_pub=Sink();n.platform_rot=np.eye(3)
n.control_enabled=True;n.guidance_invalidated=True;n.max_vx_mps=n.max_vy_mps=n.max_vz_mps=0.;n.vision_control_seq=1;n.reset_counter=0;n.vision_control_source_id='review'
n.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=100,nanosec=0)))
n._publish_command([1.,2.,3.],100.,True)
v=n.vision_control_pub.items[-1]
assert not v.valid and (v.vx,v.vy,v.vz,v.wz)==(0.,0.,0.,0.)
n.navigation_command_pub=Sink();n.last_navigation_time=0.;n.navigation_command_hz=10.;n.vision_rel=None;n.vision_time=0.;n.command_timeout_s=.5;n.impacted=False;n.flight_state='seek';n.platform_history=NS(identity=('bridge',0,'origin'))
n.waypoint_alignment_confirmed=True;n.waypoint_alignment_origin_id='origin';n.navigation_seq=1;n.target_x=1.;n.target_y=2.;n.push_altitude_m=1.5;n.yaw_cmd=0.
n._publish_navigation(100.)
a=n.navigation_command_pub.items[-1]
assert not a.valid and not a.position_valid
print(json.dumps({'exact_current_methods':True,'physical_control':False,'velocity_invalid_zero':not v.valid,'navigation_invalid_even_origin_confirmed':not a.valid}))

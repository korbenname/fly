import json
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/jikl62/FC01_OrangePi_Navigation_20261028/target/src/fc01_mission')
from fc01_mission.platform_feedback import quaternion_ned_frd_to_enu_flu
from fc01_mission.sensor_contract import rotation_xyzw
rng=np.random.default_rng(7289)
E=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
F=np.diag([1.,-1.,-1.])
error=0.
for _ in range(1000):
    q=rng.normal(size=4);q/=np.linalg.norm(q)
    w=q[0]; v=q[1:]
    skew=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
    # Quaternion independent Rodrigues-style matrix identity.
    expected=E@(np.eye(3)*(w*w-np.dot(v,v))+2*np.outer(v,v)+2*w*skew)@F
    actual=rotation_xyzw(quaternion_ned_frd_to_enu_flu(q))
    error=max(error,float(np.max(np.abs(expected-actual))))
assert error<1e-12,error
print(json.dumps({'cases':1000,'seed':7289,'maximum_matrix_error':error,'passed':True}))

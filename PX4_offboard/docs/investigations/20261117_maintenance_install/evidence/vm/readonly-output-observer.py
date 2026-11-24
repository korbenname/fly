import json,time,rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from px4_msgs.msg import VehicleAttitudeSetpoint,VehicleCommand,OffboardControlMode
rclpy.init(args=[]);n=Node('fc01_maintenance_readonly_output_observer');q=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
f=open('/home/orangepi/maintenance-observed-output.jsonl','a',buffering=1);counts={'setpoint':0,'command':0,'heartbeat':0}
def observe(kind,msg):
 counts[kind]+=1;d={'kind':kind,'timestamp':msg.timestamp,'monotonic':time.monotonic()}
 if kind=='setpoint':d['thrust_body']=[float(v) for v in msg.thrust_body]
 if kind=='command':d.update(command=int(msg.command),param1=float(msg.param1),param2=float(msg.param2))
 f.write(json.dumps(d)+'\n')
subs=[n.create_subscription(VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint',lambda m:observe('setpoint',m),q),n.create_subscription(VehicleCommand,'/fmu/in/vehicle_command',lambda m:observe('command',m),q),n.create_subscription(OffboardControlMode,'/fmu/in/offboard_control_mode',lambda m:observe('heartbeat',m),q)]
start=time.monotonic();last=0
while True:
 rclpy.spin_once(n,timeout_sec=.05)
 if time.monotonic()-last>2:
  topics=('/fmu/out/vehicle_status_v1','/fmu/out/vehicle_local_position','/fmu/out/manual_control_setpoint','/fmu/out/manual_control_switches','/fmu/out/vehicle_attitude','/fmu/out/vehicle_land_detected')
  row={'elapsed_s':time.monotonic()-start,'counts':counts,'input_publishers':{t:n.count_publishers(t) for t in topics}}
  Path('/home/orangepi/maintenance-observer.json').write_text(json.dumps(row));last=time.monotonic()

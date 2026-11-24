#!/usr/bin/env python3
"""Read-only ARM VM ROS output witness. Never constructs a publisher."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from px4_msgs.msg import VehicleAttitudeSetpoint,VehicleCommand,OffboardControlMode

def main():
 p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=300);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 rclpy.init(args=[]);node=Node('fc01_step6_readonly_output_witness',enable_rosout=False,start_parameter_services=False)
 qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
 counts={'setpoint':0,'command':0,'heartbeat':0};violation=[];records=[]
 def receive(kind,m):
  counts[kind]+=1;row={'kind':kind,'timestamp':int(m.timestamp),'receive_monotonic':time.monotonic()}
  if kind=='setpoint':
   row['thrust_body']=[float(x) for x in m.thrust_body]
   if any(x!=0 for x in row['thrust_body']):violation.append(row)
  if kind=='command':
   row.update(command=int(m.command),param1=float(m.param1),param2=float(m.param2))
   # No aircraft feedback: neither arming nor takeoff request may be submitted.
   if row['command']==22 or row['command']==400 and row['param1']>0:violation.append(row)
  if len(records)<1000:records.append(row)
 subscriptions=[node.create_subscription(t,topic,lambda m,k=kind:receive(k,m),qos) for t,topic,kind in (
  (VehicleAttitudeSetpoint,'/fmu/in/vehicle_attitude_setpoint','setpoint'),
  (VehicleCommand,'/fmu/in/vehicle_command','command'),
  (OffboardControlMode,'/fmu/in/offboard_control_mode','heartbeat'))]
 topics=('/fmu/out/vehicle_status_v1','/fmu/out/vehicle_local_position','/fmu/out/vehicle_attitude',
         '/fmu/out/manual_control_setpoint','/fmu/out/manual_control_switches','/fmu/out/vehicle_land_detected',
         '/fmu/out/battery_status','/fmu/out/failsafe_flags','/fmu/out/vehicle_command_ack')
 samples=[];start=time.monotonic();last=0.
 try:
  while time.monotonic()-start<a.seconds:
   rclpy.spin_once(node,timeout_sec=.1)
   now=time.monotonic()
   if now-last>=2:
    sample={'receive_monotonic':now,'input_publishers':{t:node.count_publishers(t) for t in topics}}
    samples.append(sample);last=now
    a.output.write_text(json.dumps({'schema_version':1,'counts':counts,'violations':violation,'records':records,'input_samples':samples,'elapsed_seconds':now-start},indent=2)+'\n')
 finally:node.destroy_node();rclpy.shutdown()
 assert not violation,'NO_FEEDBACK_FLIGHT_OUTPUT_VIOLATION'
 assert samples and all(all(n==0 for n in s['input_publishers'].values()) for s in samples),'Unexpected synthetic/aircraft input publisher'
 print('FC01_VM_READONLY_NO_FEEDBACK_OUTPUT_PASS '+json.dumps(counts))
if __name__=='__main__':main()

import rclpy,subprocess,time,signal,json
from rclpy.node import Node
from std_msgs.msg import String
from jikl_fc01_system.msg import TargetObservation
from rclpy.qos import QoSProfile,ReliabilityPolicy
rclpy.init();n=Node('camera_missing_fixture');statuses=[];obs=[]
n.create_subscription(String,'/fc01/vision/camera_status',lambda m:statuses.append(json.loads(m.data)),10)
n.create_subscription(TargetObservation,'/fc01/vision/target_observation',obs.append,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
b='/home/jikl62/.cache/fc01-vision-step2-20261118/install/fc01_vision/lib/fc01_vision/fc01_vision_node'
results=[]
for i in range(3):
 log=open(f'/tmp/fc01-step2-camera-missing-{i}.log','w');p=subprocess.Popen([b,'--ros-args','-p','device_index:=99999'],stdout=log,stderr=subprocess.STDOUT)
 end=time.monotonic()+1.5
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
 assert statuses and all(not s['ready'] for s in statuses)
 assert set(statuses[-1]['missing'])=={'CAMERA_CALIBRATION','CAMERA_EXTRINSICS','TARGET_MODEL','CAPTURE_LATENCY','CAMERA_RECTIFICATION'}
 assert obs and all(not m.tracking and m.valid_fields==0 for m in obs)
 p.send_signal(signal.SIGINT);code=p.wait(timeout=4);assert code==0;results.append({'round':i,'exit':code,'source':statuses[-1]['source_id']});log.close()
print(json.dumps({'passed':True,'missing_config_visible':True,'no_valid_measurement':True,'restart_distinct_source':len({x['source'] for x in results})==3,'rounds':results}))
n.destroy_node();rclpy.shutdown()

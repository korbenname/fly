#!/usr/bin/env python3
"""VM-only test driver. No ROS publishers and no native socket clients."""
import argparse,importlib.util,json,sys,time,os
from pathlib import Path
ENTRY=Path('/home/orangepi/FC01_versions/20261118_vision_step6_fix1')
sys.path.insert(0,str(ENTRY))
from navigation_prebuilt_install import Backend,verify_active,flight_readiness
import yaml
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--mode',required=True);p.add_argument('--calibration');args=p.parse_args()
b=Backend();manifest=json.loads((ENTRY/'ARM64_RELEASE.json').read_text());env=json.loads((ENTRY/'deployment/environment.json').read_text())
actual=verify_active(b,ENTRY,manifest,env)
fr=flight_readiness(b)
assert fr['ready_for_manual_rc'] is False,fr
result=dict(label=args.label,service_pid=actual['service']['MainPID'],controller=actual['controller'],readiness=fr)
profile=yaml.safe_load((ENTRY/'deployment/runtime.yaml').read_text())['fc01_system']['ros__parameters']
assert profile['rc_target_height']==1.5
if args.mode=='track':
 end=time.monotonic()+150
 while True:
  v=json.loads((ENTRY/'deployment/vision-runtime.json').read_text())
  obs=v.get('observations',{})
  if all(isinstance(obs.get(k,{}).get('value'),dict) for k in ('camera','mission')):break
  if time.monotonic()>end:raise AssertionError(('missing observations',v))
  time.sleep(1)
 assert v['calibration_state']==args.calibration,v
 assert v['mission_control_enabled']==(args.calibration=='confirmed'),v
 c=obs['camera']['value'];m=obs['mission']['value']
 assert c['ready'] is False,c
 assert m['target_locked'] is False and m['native_navigation_current'] is False,m
 assert m.get('control_authority') is not True,m
 if args.calibration=='pending':
  assert c['configuration']=='CAMERA_CALIBRATION_UNCONFIRMED',c
  assert {'CAMERA_CALIBRATION','CAMERA_RECTIFICATION','CAMERA_EXTRINSICS','TARGET_MODEL','CAPTURE_LATENCY'}<=set(c['missing']),c
 result['vision']=v
else:
 assert profile['visual_navigation_enabled'] is False and profile['px4_horizontal_enabled'] is False
result['processes']=os.popen('ps -eo pid,ppid,args').read()
Path('/home/orangepi/step6-evidence/'+args.label+'.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('FINAL_FIX1_'+args.label.upper().replace('-','_')+'_PASS',flush=True)

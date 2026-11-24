#!/usr/bin/env python3
"""VM-only independent read-only evidence helper; never publishes ROS or controls services."""
import argparse, hashlib, json, os, signal, subprocess, time
from pathlib import Path

SERVICE='fc01-navigation.service'

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def proc_rows():
    rows=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal(): continue
        try:
            argv=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
            if not argv: continue
            executable=os.readlink(p/'exe')
            if Path(executable.removesuffix(' (deleted)')).name!='fc01_system_node': continue
            status=dict(line.split(':',1) for line in (p/'status').read_text().splitlines() if ':' in line)
            rows.append(dict(pid=int(p.name),exe=executable,argv=argv,ppid=int(status['PPid']),
                             uid=int(status['Uid'].split()[0])))
        except (FileNotFoundError, ProcessLookupError): continue
    return sorted(rows,key=lambda r:r['pid'])
def service():
    out=subprocess.check_output(['systemctl','show',SERVICE,'-p','ActiveState','-p','MainPID',
        '-p','WorkingDirectory','-p','ExecStart','-p','DropInPaths'],text=True)
    return dict(line.split('=',1) for line in out.splitlines() if '=' in line)
def snapshot(home):
    result=dict(unix=time.time(),monotonic=time.monotonic(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                service=service(),controllers=proc_rows())
    for key,path in [('status',home/'FC01_navigation/status.json'),('runtime',home/'FC01_navigation/runtime.yaml')]:
        if path.is_file():
            result[key+'_sha256']=sha(path)
            result[key]=json.loads(path.read_text()) if key=='status' else path.read_text()
    entry=Path(result['service'].get('WorkingDirectory','/'))
    for key,name in [('vision','vision-runtime.json'),('install','INSTALL_STATE.json')]:
        path=entry/'deployment'/name
        if path.is_file():result[key]=json.loads(path.read_text())
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['snapshot','monitor','assert-no-feedback','assert-transitions','assert-repeat','make-site'])
    p.add_argument('--home',type=Path,default=Path('/home/orangepi'));p.add_argument('--output',type=Path)
    p.add_argument('--seconds',type=float,default=600);p.add_argument('--input',type=Path);p.add_argument('--before',type=Path);p.add_argument('--after',type=Path)
    a=p.parse_args()
    if a.action=='snapshot':result=snapshot(a.home)
    elif a.action=='monitor':
        assert a.output
        stop=False
        def done(*unused):nonlocal stop;stop=True
        signal.signal(signal.SIGTERM,done);signal.signal(signal.SIGINT,done)
        previous=None;end=time.monotonic()+a.seconds
        with a.output.open('a',buffering=1) as f:
            while not stop and time.monotonic()<end:
                current=proc_rows()
                if current!=previous:
                    f.write(json.dumps(dict(unix=time.time(),monotonic=time.monotonic(),controllers=current))+'\n');previous=current
                time.sleep(.25)
        return
    elif a.action=='assert-no-feedback':
        result=snapshot(a.home);s=result.get('status',{}).get('state') or {}
        assert len(result['controllers'])==1,result['controllers']
        assert result['service']['ActiveState']=='active',result['service']
        timing=s.get('source_timing');assert isinstance(timing,dict) and timing,'missing source timing evidence'
        seen={k:timing.get(k,{}).get('callbacks_total') for k in ('status','land','local','attitude','flags','battery')}
        assert all(v==0 for v in seen.values()),seen
        assert s.get('navigation_allowed') is not True,s
        assert s.get('active') is not True,s
        assert s.get('phase') not in ('THRUST_RAMP','CLIMB','HEIGHT_HOLD'),s.get('phase')
        assert all(v==0 for v in (s.get('thrust_body') or [0,0,0])),s.get('thrust_body')
        result['assertions']=dict(no_feedback_callbacks=seen,no_native_motion_authority=True,unique_controller=True)
    elif a.action=='assert-transitions':
        rows=[json.loads(line) for line in a.input.read_text().splitlines() if line]
        assert rows,'no process samples'
        assert all(len(r['controllers'])<=1 for r in rows),'overlapping native processes'
        previous=None;transitions=[]
        for r in rows:
            current=r['controllers']
            if current and previous and previous['controllers'] and current[0]['pid']!=previous['controllers'][0]['pid']:
                raise AssertionError('PID switch without sampled empty interval: sampling inconclusive, do not claim strict order')
            if current!= (previous or {}).get('controllers'):transitions.append(r)
            previous=r
        result=dict(sampled_unique=True,empty_interval_between_observed_switches=True,transitions=transitions,
                    limitation='0.25s sampling; installer stop-and-wait is stronger transaction ordering evidence')
    elif a.action=='assert-repeat':
        before=json.loads(a.before.read_text());after=json.loads(a.after.read_text())
        assert before['service']['MainPID']==after['service']['MainPID'],'supervisor restarted'
        assert before['controllers']==after['controllers'],'native restarted or paths changed'
        result=dict(repeat_keeps_supervisor_and_controller=True,service_pid=after['service']['MainPID'],controller=after['controllers'])
    else:
        result=dict(schema_version=1,provenance={k:'SYNTHETIC ARM VM SOFTWARE-ONLY TEST, NOT FIELD CALIBRATION: '+k for k in
            ('camera_device','intrinsics','extrinsics','target_model','capture_latency','standoff')},
            camera=dict(device_path='/dev/v4l/by-id/FC01_VM_NO_PHYSICAL_CAMERA',camera_fx=466.7,camera_fy=466.7,
                camera_cx=320.,camera_cy=240.,frame_width=640,frame_height=480,frame_fps=30,
                camera_to_body_wxyz=[1.,0.,0.,0.],target_diameter_m=.2,calibration_confirmed=True,
                calibration_id='SYNTHETIC-VM-K',image_rectified_confirmed=True,extrinsics_confirmed=True,
                extrinsics_id='SYNTHETIC-VM-extrinsics',target_model_confirmed=True,target_model_id='SYNTHETIC-VM-circle',
                capture_latency_confirmed=True,capture_latency_bound_s=.01,target_shape='circle'),
            mission=dict(camera_extrinsics_confirmed=True,camera_extrinsics_id='SYNTHETIC-VM-extrinsics',
                standoff_calibration_confirmed=True,standoff_calibration_id='SYNTHETIC-VM-distance',
                camera_pos_x=.1,camera_pos_y=0.,camera_pos_z=.02,standoff_m=1.2))
    output=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if a.output:a.output.write_text(output)
    else:print(output,end='')
if __name__=='__main__':main()

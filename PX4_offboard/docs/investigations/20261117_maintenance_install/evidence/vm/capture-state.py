import json,time,hashlib,subprocess,pathlib,yaml
r=pathlib.Path('/home/orangepi/FC01_navigation');status={}
try:status=json.loads((r/'status.json').read_text())
except (OSError,ValueError):pass
s=status.get('state') or {};controllers=[];agents=[]
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
  if not args:continue
  name=pathlib.Path(args[0]).name
  if name not in ('fc01_system_node','MicroXRCEAgent'):continue
  row={'pid':int(p.name),'exe':str((p/'exe').resolve()),'argv':args,'sha256':hashlib.sha256((p/'exe').read_bytes()).hexdigest()}
  (controllers if name=='fc01_system_node' else agents).append(row)
 except (OSError,ValueError):pass
props=subprocess.check_output(['systemctl','show','fc01-navigation.service','-p','MainPID','-p','ActiveState','-p','WorkingDirectory','-p','ExecStart','-p','DropInPaths'],text=True)
result={'unix':time.time(),'monotonic':time.monotonic(),'service':props,'controllers':controllers,'controller_count':len(controllers),'agents':agents,'cached_status':status.get('status'),'cached_state':{k:s.get(k) for k in ('phase','reason','active','heartbeat_enabled','armed','arming_state_known','status_fresh','local_fresh','thrust_body','rc_target_height','rc_navigation_mode_slot')},'source_callbacks':{k:v.get('callbacks_total') for k,v in s.get('source_timing',{}).items()}}
if (r/'runtime.yaml').is_file():result.update(runtime=yaml.safe_load((r/'runtime.yaml').read_text()),runtime_sha256=hashlib.sha256((r/'runtime.yaml').read_bytes()).hexdigest())
obs=pathlib.Path('/home/orangepi/maintenance-observer.json')
if obs.is_file():result['observer']=json.loads(obs.read_text())
print(json.dumps(result,indent=2))

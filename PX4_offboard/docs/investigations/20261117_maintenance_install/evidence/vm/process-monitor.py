import json,time
from pathlib import Path
f=open('/root/maintenance-process-transitions.jsonl','a',buffering=1);previous=None
while True:
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   args=[x.decode() for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   if args and Path(args[0]).name=='fc01_system_node':
    rows.append({'pid':int(p.name),'exe':str((p/'exe').resolve()),'argv':args})
  except OSError:pass
 rows.sort(key=lambda v:v['pid'])
 if rows!=previous:
  f.write(json.dumps({'monotonic':time.monotonic(),'unix':time.time(),'controllers':rows})+'\n');previous=rows
 time.sleep(1.)

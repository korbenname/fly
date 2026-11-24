#!/bin/bash
set -euo pipefail
VM=/home/jikl62/.cache/fc01-arm64-vm-20261116/ssh-vm
"$VM" 'set -e
systemctl stop fc01-navigation.service
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-inactive-original-1_23.json
python3 - <<'\''PY'\''
import hashlib,json,yaml
from pathlib import Path
entry=Path("/home/orangepi/FC01_versions/20261117_stage3_fix1")
p=entry/"deployment/runtime.yaml"; profile=yaml.safe_load(p.read_text())
profile["fc01_system"]["ros__parameters"]["rc_target_height"]=1.5
p.write_text(yaml.safe_dump(profile,sort_keys=False,allow_unicode=True))
s=entry/"deployment/INSTALL_STATE.json";j=json.loads(s.read_text());j["runtime_sha256"]=hashlib.sha256(p.read_bytes()).hexdigest();s.write_text(json.dumps(j,indent=2)+"\n")
live=Path("/home/orangepi/FC01_navigation/runtime.yaml");live.write_text(p.read_text())
Path("/root/step6-evidence/VM_ONLY_1_5_BASELINE.json").write_text(json.dumps(dict(note="Explicit VM-only software test fixture; not field configuration or calibration",target_height_m=1.5,prior_height_m=1.23),indent=2))
PY
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-before-inactive-install.json
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh /home/orangepi/Desktop/FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-after-inactive-install.json
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-no-feedback --output /root/step6-evidence/fix1-inactive-install-no-feedback.json
python3 - <<'\''PY'\''
import json
from pathlib import Path
p=Path("/root/step6-evidence");a=json.loads((p/"fix1-before-inactive-install.json").read_text());b=json.loads((p/"fix1-after-inactive-install.json").read_text())
assert a["service"]["ActiveState"]=="inactive" and not a["controllers"]
assert b["service"]["ActiveState"]=="active" and len(b["controllers"])==1
assert b["controllers"][0]["exe"].startswith("/home/orangepi/FC01_versions/20261118_vision_step6_fix1/")
print("FINAL_FIX1_INACTIVE_BASELINE_INSTALL_PASS")
PY
systemctl stop fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/FC01_versions/20261118_vision_step6_fix1/navigation_vision_mode.py track --pending-calibration --entry /home/orangepi/FC01_versions/20261118_vision_step6_fix1 --maintenance
systemctl start fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/step6-evidence/vm-mode-accept.py --label arm64-pending-track --mode track --calibration pending
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh --status
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-no-feedback --output /root/step6-evidence/fix1-pending-no-feedback.json
python3 /root/step6-evidence/fc01-step6-vm-probe.py make-site --output /home/orangepi/vm-synthetic-site.json
chown orangepi:orangepi /home/orangepi/vm-synthetic-site.json
systemctl stop fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/FC01_versions/20261118_vision_step6_fix1/navigation_vision_mode.py track --site /home/orangepi/vm-synthetic-site.json --entry /home/orangepi/FC01_versions/20261118_vision_step6_fix1 --maintenance
systemctl start fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/step6-evidence/vm-mode-accept.py --label arm64-confirmed-fixture-track --mode track --calibration confirmed
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh --status
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-track-before-repeat.json
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh /home/orangepi/Desktop/FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-track-after-repeat.json
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-repeat --before /root/step6-evidence/fix1-track-before-repeat.json --after /root/step6-evidence/fix1-track-after-repeat.json
runuser -u orangepi -- python3 -B /home/orangepi/step6-evidence/vm-mode-accept.py --label arm64-track-repeat --mode track --calibration confirmed
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/orangepi/FC01_versions/20261118_vision_step6_fix1/install/local_setup.bash
python3 /root/step6-evidence/fc01-step6-vm-observe.py --seconds 30 --output /root/step6-evidence/track-no-feedback-ros-final.json
systemctl stop fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/FC01_versions/20261118_vision_step6_fix1/navigation_vision_mode.py altitude --entry /home/orangepi/FC01_versions/20261118_vision_step6_fix1 --maintenance
systemctl start fc01-navigation.service
runuser -u orangepi -- python3 -B /home/orangepi/step6-evidence/vm-mode-accept.py --label arm64-return-altitude --mode altitude
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-no-feedback --output /root/step6-evidence/fix1-altitude-return-no-feedback.json
python3 - <<'\''PY'\''
import json
from pathlib import Path
rows=[]
for p in Path("/proc").glob("[0-9]*"):
 try:
  args=[x.decode(errors="replace") for x in (p/"cmdline").read_bytes().split(b"\0") if x]
  if any(Path(x).name in ("fc01_vision_node","mission","px4_state_bridge","fc01_target_interface_node") for x in args[:2]):rows.append((p.name,args))
 except (FileNotFoundError,ProcessLookupError):continue
assert not rows,rows
print("FINAL_FIX1_TRACK_AUXILIARIES_EXITED_ALTITUDE_PASS")
PY
backup=$(python3 -c '\''import json;print(json.load(open("/home/orangepi/FC01_versions/20261118_vision_step6_fix1/deployment/INSTALL_STATE.json"))["backup"])'\'')
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh --rollback "$backup"
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-after-inactive-rollback.json
python3 - <<'\''PY'\''
import json
from pathlib import Path
p=Path("/root/step6-evidence");a=json.loads((p/"fix1-before-inactive-install.json").read_text());b=json.loads((p/"fix1-after-inactive-rollback.json").read_text())
assert b["service"]["ActiveState"]=="inactive" and b["service"]["MainPID"]=="0" and not b["controllers"],b["service"]
assert b["runtime"]==a["runtime"]
assert b["service"]["WorkingDirectory"]==a["service"]["WorkingDirectory"]
print("FINAL_FIX1_INACTIVE_ROLLBACK_RUNTIME_BYTE_IDENTICAL_PASS")
PY
systemctl stop fc01-step6-process-monitor.service
python3 /root/step6-evidence/fc01-step6-vm-probe.py assert-transitions --input /root/step6-evidence/process-transitions-final.jsonl --output /root/step6-evidence/fix1-transition-assertions.json
printf "FINAL_FIX1_ARM64_MODE_LIFECYCLE_SUITE_PASS\n"'

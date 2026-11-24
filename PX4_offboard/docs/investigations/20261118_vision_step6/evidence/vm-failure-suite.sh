#!/bin/bash
set -euo pipefail
VM=/home/jikl62/.cache/fc01-arm64-vm-20261116/ssh-vm
"$VM" 'cat > /usr/local/bin/fc01-vm-fail-step6 <<'\''EOF'\''
#!/bin/sh
case "$PWD" in
 /home/orangepi/FC01_versions/20261118_vision_step6_fix1)
   echo VM_TEST_CANDIDATE_START_FAILURE >&2; exit 23 ;;
esac
exit 0
EOF
chmod 755 /usr/local/bin/fc01-vm-fail-step6
mkdir -p /etc/systemd/system/fc01-navigation.service.d
printf "[Service]\nExecStartPre=/usr/local/bin/fc01-vm-fail-step6\n" > /etc/systemd/system/fc01-navigation.service.d/yy-vm-fail-step6.conf
systemctl daemon-reload
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-before-failure.json
set +e
runuser -u orangepi -- bash /home/orangepi/Desktop/install_fc01.sh /home/orangepi/Desktop/FC01_OrangePi_20261118_vision_step6_fix1_arm64.tar.gz
result=$?
set -e
printf "INJECTED_INSTALL_EXIT=%s\n" "$result"
test "$result" -ne 0
python3 /root/step6-evidence/fc01-step6-vm-probe.py snapshot --output /root/step6-evidence/fix1-after-failure.json
python3 - <<'\''PY'\''
import json
from pathlib import Path
p=Path("/root/step6-evidence")
a=json.loads((p/"fix1-before-failure.json").read_text());b=json.loads((p/"fix1-after-failure.json").read_text())
assert b["service"]["ActiveState"]=="active", b["service"]
assert b["service"]["WorkingDirectory"]==a["service"]["WorkingDirectory"]
assert b["runtime"]==a["runtime"]
assert len(b["controllers"])==1
assert b["controllers"][0]["exe"]==a["controllers"][0]["exe"]
print("FINAL_FIX1_START_FAILURE_AUTOMATIC_RECOVERY_PASS")
PY
journalctl -u fc01-navigation.service --no-pager -n 40
rm /etc/systemd/system/fc01-navigation.service.d/yy-vm-fail-step6.conf
systemctl daemon-reload'

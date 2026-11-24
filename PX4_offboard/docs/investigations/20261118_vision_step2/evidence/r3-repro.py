import json,sys
from pathlib import Path
root=Path('/home/jikl62/FC01_OrangePi_Navigation_20261028')
sys.path.insert(0,str(root/'tests'))
from test_mission_sensor_contract import platform
from fc01_mission.sensor_contract import PlatformHistory
h=PlatformHistory()
a=h.accept(platform(),100_000_000_000,10.)
bad=platform(100_010_000_000,10.01,position=[float('nan'),0.,0.])
b=h.accept(bad,100_010_000_000,10.01)
reason=h.reason
c=h.accept(platform(100_010_000_000,10.01),100_020_000_000,10.02)
print(json.dumps({'scenario':'ready=true NaN position followed by valid same raw sample','first_good':a,'bad_rejected':not b,'reason':reason,'same_sample_revalidated':c,'raw_stamp_ns':100010000000,'path':'PlatformHistory.accept'}))

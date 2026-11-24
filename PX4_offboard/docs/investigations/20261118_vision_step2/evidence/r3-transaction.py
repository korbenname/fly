import json,sys
from pathlib import Path
sys.path.insert(0,str(Path.cwd()/'tests'))
from test_mission_sensor_contract import platform
from fc01_mission.sensor_contract import PlatformHistory
h=PlatformHistory();assert h.accept(platform(),100_000_000_000,10.)
assert not h.accept(platform(100_010_000_000,10.01,attitude_xyzw=[0.,0.,0.,0.]),100_010_000_000,10.01)
assert all(not x for x in h.fields.values())
# Only local position/velocity advance; previous invalid attitude/omega stamps remain retired.
m=platform(100_020_000_000,10.02,attitude_stamp_ns=100_010_000_000,angular_velocity_stamp_ns=100_010_000_000)
partial=h.accept(m,100_020_000_000,10.02)
assert not partial
assert not h.fields['attitude'] and not h.fields['angular_velocity']
# All required fields now advance; normal recovery supported, no permanent poisoning for finite invalid sample.
recovered=h.accept(platform(100_030_000_000,10.03),100_030_000_000,10.03)
assert recovered
print(json.dumps({'partial_recovery_rejected':not partial,'all_required_fields_advanced_recovered':recovered,'transactional_no_partial_append_on_bad_attitude':True}))

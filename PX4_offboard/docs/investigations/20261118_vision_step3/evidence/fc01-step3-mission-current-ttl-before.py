import sys,json
sys.path.insert(0,'tests')
from test_mission_native_gate import state,NativeFlightGate
n=100_000_000_000
m=state(local_sample_stamp_us=(n-290_000_000)//1000,local_receive_monotonic_ns=9_710_000_000)
g=NativeFlightGate('boot');first=g.accept(m,n,10.)
print(json.dumps({'accept_local_age_0_29':first,'current_local_age_0_49':g.current(n+200_000_000,10.2),'reason':g.reason}))

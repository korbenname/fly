import json
from tests.test_navigation_feedback_freshness import ready_state
import navigation
state=ready_state()
base=dict(status='READY_WAIT_ARM',reason='ready',supervisor_pid=10,boot_id='b',updated_unix=1000.,state_age_seconds=0.,state=state,feedback_current=True)
info=dict(ActiveState='active',MainPID='10',boot_id='b')
views={}
for name,row,svc in (
 ('missing_controller_identity',base,info),
 ('restarted_supervisor',base,dict(info,MainPID='11')),
 ('changed_boot',base,dict(info,boot_id='c')),
 ('stale_ready_label_current_ascend',dict(base,state=dict(state,phase='ASCEND',reason='ASCEND',armed=True,nav_state=14)),info),
 ('ready_ground_not_at_rest',dict(base,state=dict(state,at_rest=False)),info),
 ('ready_no_ground_zero',dict(base,state=dict(state,height_reference_ready=False)),info),
 ('ready_no_prestream_commits',dict(base,state=dict(state,rc_prestream_commits=0)),info),
 ):
 v=navigation.render_status(row,svc,now=1000.)
 views[name]={'first_line':v.splitlines()[0],'manual_arm_prompt':'已准备：CH5 手动解锁' in v,'actual_mode':'实际模式：' in v}
print(json.dumps(views,ensure_ascii=False,indent=2))

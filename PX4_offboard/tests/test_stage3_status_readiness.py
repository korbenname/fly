"""准备文案的证据交叉核对：显示不重建原生授权，也不操作服务或飞控。"""
import copy
import math
import unittest

import navigation
from navigation_status import (manual_rc_readiness_reason, snapshot_provenance_current,
                               aircraft_feedback_current)
from tests.test_navigation_feedback_freshness import ready_state


def current_ready():
    state=ready_state()
    state.update(controller_pid=22,rc_source_selected=True,rc_link_ready=True,
        rc_switches_ready=True,rc_source_clock_valid=True,rc_switch_clock_valid=True,
        rc_axes_safe=True,source_clock_validated=True,source_clock_valid=True,
        rc_sticks_moving=False,rc_kill_switch=3,rc_kill_latched=False,
        operator_takeover_latched=False,height_reference_ready=True,
        rc_ground_reference_qualified=True,rc_prestream_commits=25,
        last_target_local_thrust_z=0.,rc_mode_slot=4,rc_navigation_mode_slot=1,
        rc_arm_switch=3,ready_wait_arm=True)
    return state


def current_row():
    return dict(state=current_ready(),supervisor_pid=20,controller_pid=22,
                state_controller_pid=22,boot_id='same-boot',status='READY_WAIT_ARM',
                reason='RC_READY_WAIT_CH5_MANUAL_ARM',updated_unix=1000.0,
                state_age_seconds=0.0,feedback_current=True)


SERVICE = dict(ActiveState='active', MainPID='20', boot_id='same-boot')


class Stage3StatusReadinessTests(unittest.TestCase):
    def assert_stale_without_arm_prompt(self, row, info=None):
        """调用真实显示入口；STALE 是显示结论，不向控制器发送任何指令。"""
        view = navigation.render_status(row, SERVICE if info is None else info, now=1000.0)
        self.assertIn('[STALE]', view)
        self.assertNotIn('已准备：CH5 手动解锁', view)
        self.assertNotIn('实际模式：', view)
        self.assertIn('历史', view)

    def test_render_historical_ready_is_stale_after_process_boot_or_identity_change(self):
        row = current_row()
        valid = navigation.render_status(row, SERVICE, now=1000.0)
        self.assertIn('[READY_WAIT_ARM]', valid)
        self.assertIn('已准备：CH5 手动解锁', valid)
        for changed in (dict(MainPID='21'), dict(boot_id='next-boot'), dict(boot_id='')):
            with self.subTest(service=changed):
                self.assert_stale_without_arm_prompt(row, dict(SERVICE, **changed))
        for key in ('supervisor_pid', 'controller_pid', 'state_controller_pid', 'boot_id'):
            with self.subTest(missing=key):
                broken = copy.deepcopy(row)
                broken.pop(key)
                self.assert_stale_without_arm_prompt(broken)
        for key in ('controller_pid', 'state_controller_pid'):
            self.assert_stale_without_arm_prompt(dict(row, **{key: 23}))
        broken = copy.deepcopy(row)
        broken['state'].pop('controller_pid')
        self.assert_stale_without_arm_prompt(broken)

    def test_render_ready_label_cannot_outlive_native_phase_or_ground_zero_stream(self):
        row = current_row()
        for changed in (dict(phase='ASCEND', armed=True, nav_state=14),
                        dict(phase='RC_PRESTREAM'), dict(at_rest=False),
                        dict(height_reference_ready=False),
                        dict(rc_ground_reference_qualified=False), dict(rc_prestream_commits=0),
                        dict(last_target_local_thrust_z=-0.12), dict(ready_wait_arm=False),
                        dict(rc_mode_slot=1), dict(nav_state=14)):
            with self.subTest(state=changed):
                self.assert_stale_without_arm_prompt(dict(row, state=dict(row['state'], **changed)))

    def test_render_dds_loss_and_expired_raw_samples_revoke_ready_display(self):
        row = current_row()
        for changed in (dict(feedback_current=False), dict(ipc_error=dict(kind='clean_eof')),
                        dict(state_age_seconds=0.5)):
            self.assert_stale_without_arm_prompt(dict(row, **changed))
        for key in ('status_fresh', 'local_fresh', 'attitude_sample_fresh'):
            self.assert_stale_without_arm_prompt(dict(row, state=dict(row['state'], **{key: False})))
        broken = copy.deepcopy(row)
        broken['state']['source_timing']['local']['sample_progress_age_s'] = 0.5
        self.assert_stale_without_arm_prompt(broken)

    def test_render_native_arm_window_and_wrong_slot_evidence_are_not_invented(self):
        row = current_row()
        row.update(status='WAIT_NAVIGATION')
        row['state'].update(phase='RC_ARMED_WAIT_NAVIGATION', reason='RC_WAIT_NAVIGATION',
                            armed=True, rc_arm_switch=1, rc_manual_arm_proven=True,
                            rc_arm_to_navigation_remaining_s=0.01)
        view = navigation.render_status(row, SERVICE, now=1000.0)
        self.assertIn('[WAIT_NAVIGATION]', view)
        self.assertNotIn('已准备：CH5 手动解锁', view)
        for changed in (dict(rc_arm_to_navigation_remaining_s=-0.01),
                        dict(reason='RC_NAVIGATION_SELECTION_TIMEOUT'),
                        dict(rc_manual_arm_proven=False), dict(rc_mode_slot=0)):
            self.assert_stale_without_arm_prompt(dict(row, state=dict(row['state'], **changed)))

    def test_current_supervisor_boot_controller_and_ipc_state_identity_required(self):
        info=dict(ActiveState='active',MainPID='20',boot_id='same-boot')
        row=current_row()
        self.assertTrue(snapshot_provenance_current(row,info))
        for key,bad in [('supervisor_pid',21),('controller_pid',23),('state_controller_pid',23),
                        ('boot_id','old-boot')]:
            with self.subTest(key=key):
                self.assertFalse(snapshot_provenance_current(dict(row,**{key:bad}),info))
        for key in ('supervisor_pid','controller_pid','state_controller_pid','boot_id'):
            broken=copy.deepcopy(row);broken.pop(key)
            self.assertFalse(snapshot_provenance_current(broken,info))
        for pid in (None,False,0,-1,'22'):
            broken=copy.deepcopy(row);broken['state']['controller_pid']=pid
            self.assertFalse(snapshot_provenance_current(broken,info))
        self.assertFalse(snapshot_provenance_current(row,dict(info,boot_id='')))
        self.assertFalse(snapshot_provenance_current(row,dict(info,ActiveState='inactive')))

    def test_ready_requires_current_native_phase_and_ground_zero_committed_zero_stream(self):
        state=current_ready()
        self.assertEqual(manual_rc_readiness_reason(state,'READY_WAIT_ARM'),'')
        changes=[dict(phase='ASCEND',armed=True,nav_state=14),dict(phase='IDLE'),
                 dict(armed=True),dict(nav_state=14),dict(at_rest=False),dict(landed=False),
                 dict(rc_arm_switch=1),dict(rc_mode_slot=1),dict(rc_navigation_mode_slot=6),
                 dict(height_reference_ready=False),dict(rc_ground_reference_qualified=False),
                 dict(rc_prestream_commits=0),dict(last_target_local_thrust_z=-.12),
                 dict(last_target_local_thrust_z=math.nan),dict(ready_wait_arm=False),
                 dict(rc_source_selected=False),dict(rc_axes_safe=False),dict(rc_sticks_moving=True),
                 dict(rc_kill_switch=1),dict(operator_takeover_latched=True)]
        for change in changes:
            with self.subTest(change=change):
                self.assertTrue(manual_rc_readiness_reason(dict(state,**change),'READY_WAIT_ARM'))
        for key in ('height_reference_ready','rc_ground_reference_qualified','last_target_local_thrust_z',
                    'rc_prestream_commits','rc_source_clock_valid','rc_switches_ready'):
            broken=dict(state);broken.pop(key)
            self.assertTrue(manual_rc_readiness_reason(broken,'READY_WAIT_ARM'))

    def test_feedback_cutoff_remains_separate_from_current_process_and_readiness_facts(self):
        state=current_ready()
        self.assertEqual(manual_rc_readiness_reason(state,'READY_WAIT_ARM'),'')
        self.assertTrue(aircraft_feedback_current(state,.1))
        self.assertFalse(aircraft_feedback_current(state,.5))
        state['local_fresh']=False
        self.assertFalse(aircraft_feedback_current(state,.01))

    def test_position_mode_ready_requires_current_heartbeat_and_finite_ground_anchor(self):
        state = dict(current_ready(), px4_takeoff_position_hold=True, heartbeat_enabled=True,
                     attitude_output_enabled=False,
                     takeoff_position_anchor_ned=[2.0, -1.0, .3], last_target_local_thrust_z=None)
        self.assertEqual(manual_rc_readiness_reason(state, 'READY_WAIT_ARM'), '')
        for invalid in (dict(heartbeat_enabled=False), dict(attitude_output_enabled=True),
                        dict(takeoff_position_anchor_ned=None),
                        dict(takeoff_position_anchor_ned=[2.0, math.nan, .3]),
                        dict(rc_prestream_commits=0), dict(ready_wait_arm=False)):
            with self.subTest(invalid=invalid):
                self.assertTrue(manual_rc_readiness_reason(dict(state, **invalid), 'READY_WAIT_ARM'))

    def test_navigation_window_display_does_not_recreate_arm_deadline(self):
        state=current_ready()
        state.update(phase='RC_ARMED_WAIT_NAVIGATION',reason='RC_WAIT_NAVIGATION',armed=True,
                     rc_arm_switch=1,rc_manual_arm_proven=True,rc_arm_to_navigation_remaining_s=.01)
        self.assertEqual(manual_rc_readiness_reason(state,'WAIT_NAVIGATION'),'')
        for missing in ('armed','rc_manual_arm_proven','landed'):
            broken=dict(state);broken.pop(missing)
            self.assertTrue(manual_rc_readiness_reason(broken,'WAIT_NAVIGATION'))
        for remaining in (-.001,8.1,math.nan,'unknown',True):
            with self.subTest(remaining=remaining):
                self.assertTrue(manual_rc_readiness_reason(dict(state,rc_arm_to_navigation_remaining_s=remaining),
                                                         'WAIT_NAVIGATION'))
        self.assertTrue(manual_rc_readiness_reason(dict(state,reason='RC_NAVIGATION_SELECTION_TIMEOUT'),
                                                 'WAIT_NAVIGATION'))


if __name__=='__main__':unittest.main()

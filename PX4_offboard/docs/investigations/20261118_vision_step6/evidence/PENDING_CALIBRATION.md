# Pending calibration task handoff
Changes applied (no commit; parent owns commit):
- navigation_vision_mode.py adds `track --pending-calibration --maintenance`, no --site allowed at same time; strict complete --site remains.
- Canonical pending site record and re-derived exact pending camera/mission configs, hash pins and strict comparison prohibit accidental confirmation/source injection. Pending migration stays pending.
- Pending camera uses `/dev/fc01-camera-unconfigured`; frame geometry/rate are structural placeholders only; all measured dimensions/IDs/confirmations cleared. Camera rotation is identity but extrinsics_confirmed=false. All five camera confirmation fields false.
- Pending mission control_enabled=false, camera lever arm and standoff confirmations false, no waypoint alignment, hold selection, 0 standoff placeholder. Full site later selects control_enabled=true.
- Shared track endpoint helper keeps actual production topics/px4/track/fixed native1.5 semantics.
- VisionAuxiliary parent/root added calibration_state + mission_control_enabled and launch control_enabled from actual mode. I de-duplicated overlapping edit; no syntax error remains.
- tests/test_navigation_vision_mode.py adds five regressions including config/migration, no fake calibration, strict tamper check, pending disabled launch argument.

Tests latest: `python3 -m unittest tests.test_navigation_vision_mode tests.test_navigation_prebuilt_service tests.test_prebuilt_vision_status -v` PASS37 (19 mode + service/status). Log `/tmp/fc01-step6-pending-tests.log`. `py_compile` and `git diff --check` pass.

Native/target are unchanged. Camera implementation already accepts unconfirmed calibration and stays alive (no-frame status every publication). Mission runtime has explicit missing camera lever arm/safe distance and rejects tracking commands without calibration. Pending disables task output including HOLD by actual launch override. Ground configuration does not start service or make flight calls. No actual hardware operations.

Real x86 Humble test launched below; append result when complete. Files `/tmp/fc01-step6-pending-live-test.py`, `/tmp/fc01-step6-pending-live.log`, `/tmp/fc01-step6-pending-live/result.json`.

Real x86 Humble test PASS `PENDING_CALIBRATION_NO_CAMERA_NO_DDS_WAIT_AND_CLEANUP_PASS` at ROS_DOMAIN_ID193 / localhost, actual production ROS typesupport and horizontal_first_test.launch.py. No camera or native/PX4/DDS producer. Observed camera ready=false, all five calibration missing fields; mission target_locked=false, no current native gate, missing lever arm and standoff. Four auxiliary nodes stay alive until orderly cleanup; launch returncode0. Result JSON records pending + mission_control_enabled=false. No real board measurement claimed.

Command:
```
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash; source /home/jikl62/ros2_px4_ws/install/local_setup.bash; source /home/jikl62/.cache/fc01-vision-step4-20261118/install/local_setup.bash; export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1; /usr/bin/python3 /tmp/fc01-step6-pending-live-test.py'
```
Recommended docs mode selection:
- altitude: original native height, no auxiliary nodes.
- track --pending-calibration: auxiliary software diagnostically running, control_enabled=false, no flight test tracking; camera path unconfigured until real --site.
- track --site "$HOME/fc01_site.json": all actual site sources checked; task enabled but native RC/Offboard/airborne/stable1.5 and actual runtime feedback gates remain.

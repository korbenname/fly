# Lifecycle maintenance continuation

Implemented by /root/step6_maintenance_lifecycle, no commits and no host service operations.

## Actual changes

- `navigation_prebuilt_install.py`: current `Backend.process` includes `/proc/PID/stat` start_ticks; parsing accounts for spaces/parentheses in comm. No volatile process state in the snapshot identity.
- `Backend.owned_processes` captures only the actual MainPID and explicit FC01 native/camera/mission/bridge/read-only/ros2-launch identities under the real service ancestor chain or systemd cgroup. Captures PID+start_ticks; excludes reused Agent and unrelated service-external nodes. Zombie is treated as exited. All process helpers are read-only.
- `stop_and_wait` tracks captured old identities after parent exit, plus newly discovered same-service FC01 children; it returns only after service+global native+captured auxiliaries exit. Every 10s emits wait progress. 120s failure raises StopIncomplete and leaves original unit unchanged, without restarting old over lingering auxiliaries. No broad kill, no new signals, no native/Agent termination.
- `Backend.start` uses systemctl start --no-block; installer software loop owns the bounded wait/progress.
- `verify_active` checks actual unit/process/path/SHA/config/DDS and actual track auxiliaries, then validates the existing supervisor-owned IPC bridge. Native socket is single-client, so the installer must NOT try to steal its connection. IPC requires same boot/MainPID/controller PID, no errors, original native diagnostic monotonic stamp after this acceptance began and <=10s old. Stability1s requires the original native stamp to advance; no aircraft/source TTL/READY/250ms requirement.
- Software progress every10s, immediate systemd failed Result/ExecMainStatus detection. Rollback verify_original also requires current process-bound IPC, not airplane fields.
- First unit and managed drop-in set finite TimeoutStartSec=600s, linked to same software acceptance constant. Existing TimeoutStopSec=infinity and SendSIGKILL=no safety remain unchanged. Since Type=simple is used, systemd start acceptance and software readiness are still separate.

## Actual tests

- Existing installer60 PASS, log `/tmp/fc01-step6-lifecycle-installer-tests.log`.
- New independent lifecycle8 PASS, log `/tmp/fc01-step6-lifecycle-new-tests.log`.
- Expanded105 methods PASS2.411s, log `/tmp/fc01-step6-lifecycle-expanded.log`.
- Command `PYTHONPATH=.:tests python3 -m unittest tests.test_navigation_prebuilt_install tests.test_prebuilt_maintenance_lifecycle tests.test_navigation_prebuilt_service tests.test_navigation_vision_mode tests.test_prebuilt_vision_status -q`.
- `git diff --check` clean.

New tests prove auxiliary exit ordering/residual refusal, no-feedback IPC acceptance, old/incorrect IPC refusal, original stamp must progress, immediate failed-unit recognition, bounded nonblocking systemd start, cgroup/ancestor ownership and PID reuse handling, Agent/external node exclusion.

Shared fake fixture got only owned_processes/same_process_alive/software_snapshot methods. Software-wait owner edits dependencies and timeouts in same file separately; keep those changes. My separate new test file imports Fixture using tests PYTHONPATH.

## Root followup

Run actual final ARM VM install/repeat/rollback. Capture output showing old auxiliary clean exit and current WAITING IPC accepted. No physical ARM/PX4/camera performance claim. Candidate native/target source not modified by this lifecycle work.

Do not interpret residual-auxiliary wait timeout as success: original unit remains, no new candidate starts. Current service supervision normally cleans all auxiliaries, so expected VM path is clean exit. No automatic SIGKILL or wide process-name kill was introduced.

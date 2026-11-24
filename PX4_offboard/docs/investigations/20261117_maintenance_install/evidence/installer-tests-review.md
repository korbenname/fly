# Maintenance installer: independent regression work

Current test result: PASS, 48 installer tests in 1.797 seconds. File edited only: `tests/test_navigation_prebuilt_install.py`. No production/service/native edits, host service operation or hardware operation. Root owns the installer implementation and final integration.

## Before/after

- Before migration, 37 previous-contract tests ran with 13 failures and six errors (`/tmp/fc01-maintenance-tests-before.log`). Failures were obsolete aircraft/Agent-session acceptance expectations and calls to intentionally removed helpers; package-integrity and software transaction tests were retained.
- The replacement fixture provides active/stopped/absent units, persisted runtime and DDS settings, local executable Agent file and explicit controller/service identities. `live_state()` and `retained_agent()` now raise AssertionError if any installer lifecycle calls them. No simulated aircraft readiness can accidentally make a test pass.
- The fixture records stop/controller-exit/start/write events. Every start asserts that no prior controller or Supervisor MainPID remains. Delayed shutdown advances mocked time without sleeping.
- New regressions found actual issues: malformed optional readiness JSON could throw after successful switching; runtime edits during dependency checks/sudo could be overwritten with previously parsed settings; a stopped-candidate restart needed original-backup recovery. Root fixed those implementation paths and the corresponding regressions pass.
- Latest module run: 48 PASS, `/tmp/fc01-maintenance-tests-final.log`. Test-source compile and `git diff --check` pass. Before the last two local-software rejection additions, the combined installer/service/builder/feedback suite passed 62 tests in 11.968 seconds (`/tmp/fc01-maintenance-tests-combined.log`).

## Covered final behavior

- Active old service, stopped old service, and active Supervisor with no native controller can install without PX4/RC/DDS messages or a running Agent/session. Missing, stale or malformed cached aircraft state is not consulted for installation/repeat/stop/rollback/recovery.
- First unit installation requires existing explicit local runtime, preserves its entire configuration and can roll back to unit absence. Missing runtime fails without creating settings, writing services or starting a controller.
- Static Environment/EnvironmentFiles and saved deployment DDS/Agent settings work without `/proc` environment from PID 0. Local Agent executable, native configuration, unit parser, dependencies and non-domain-0 policy errors remain pre-stop software failures.
- Repeat while active does not restart. Repeat while stopped preserves the original backup and starts only after zero old processes. Repeat restart failure restores that same original active/inactive policy instead of creating a candidate-as-original backup.
- Automatic failure restoration preserves original runtime/unit and active/inactive/absent state. Failed candidate Supervisor without native can be stopped and recovered.
- A 61-second native graceful exit succeeds within the installer budget and is observed before candidate start. A controller still present at 120 seconds refuses without force termination, unit replacement or candidate start. Rollback similarly observes candidate exit before old start.
- Duplicate/unmanaged/wrong-user controllers, service/controller changes during sudo, external unit edits, configuration races, path symlinks, corrupted backup/payload, wrong SHA/ELF, tar traversal/link, unlisted bytecode and systemd scalar-path ambiguity remain explicit refusals.
- Candidate acceptance checks actual exe/hash/cwd/unit/configuration/DDS rather than aircraft status. Separate status/install CLI tests prove absent/malformed optional aircraft snapshots still report software success and false flight readiness.
- Direct navigation help/status uses an actual fresh Python process with bytecode variables removed; no unlisted caches appear, strict tree verification still passes and repeat remains idempotent.

## Separate display finding

The current optional `flight_readiness()` uses a three-second controller timestamp window plus saved boolean fresh flags. It does not yet accumulate the original individual source ages, so e.g. an originally valid local sample can expire at 0.5 seconds while the saved three-second controller status still looks current. It can also label READY without checking the action-policy gates. This does not change deployment acceptance or native flight controls, but the displayed `ready_for_manual_rc` should remain conservative. Root was notified to reuse the existing pure `navigation_status.aircraft_feedback_current()` / guidance rules only for display, or report unknown until the ordinary status view confirms readiness. Do not reintroduce aircraft TTL checks into installation success.

Real ARM64 no-feedback install/repeat/rollback/failure integration is owned by the VM agent and is separate from these injected-backend regressions.

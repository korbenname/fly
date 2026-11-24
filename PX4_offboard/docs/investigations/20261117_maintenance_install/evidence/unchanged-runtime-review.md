# Maintenance installation: existing prebuilt service review

Result: existing `dist/arm64_fix1` runtime supports software standby with no PX4/RC/DDS feedback and no already-running Agent. No service, standby or native change is necessary for that requirement. No production edits were made. Installation acceptance belongs to the separate installer change; software presence/running identity must not be reported as flight readiness.

## Actual startup path

- `navigation_prebuilt_service.check()` verifies immutable payload/hash, ARM64 ELF, dynamic libraries, bundled message type support and the parameter-only native configuration checker. It neither starts an Agent/controller nor waits for aircraft messages, RC or an XRCE session.
- `run_service()` preserves the deployment runtime keys and gates through `snapshot_profile()` and invokes original `standby.supervise()` with the packaged controller and exact deployment snapshot.
- `standby.supervise()` calls original `launcher.preflight()`. That checks local ARM64/environment/binary/configuration/type support, readable/writable character-device `/dev/ttyS2`, unique controller and UART ownership, and a reusable same-user 115200 Agent OR an executable offline Agent binary. It does not require an existing Agent, PX4 connection, RC sample or DDS peer.
- With no existing Agent, the Supervisor starts the original Agent command `serial -D /dev/ttyS2 -b 115200 -v 0` and starts the native controller immediately. There is no handshake/session-success prerequisite between them. Agent recovery retries transport without replacing or commanding the native controller. A running Agent process labeled `CONNECTED` is process/transport evidence, not proof of an XRCE session or current aircraft state.
- `standby.monitor()` sends only `STATUS`. Absent status/local samples produce `WAITING` with `feedback_current=false`; stale telemetry alone does not stop the native process. `describe_row()` cannot upgrade that state into READY.

Missing/incompatible local libraries, configuration, Agent executable, UART permissions/device, unknown UART owner or duplicate native process remain concrete local software/resource failures. Absence of an Agent session, RC transmitter or aircraft DDS peer is distinct and does not block this service startup. A missing Agent executable is not equivalent to a stopped but installed Agent.

## Flight behavior remains unchanged

- The prebuilt service preserves the original runtime policy, including `arm_enabled=false`, `flight_enabled=false`, `visual_navigation_enabled=false`, manual RC mode and every source timeout. It must not manufacture disarmed/landed/RC-ready facts for maintenance.
- Native `RcNavigationStart::step()` in `rc_navigation.hpp` rejects missing source-clock/selected RC/axes/battery/preflight/ground evidence before entering prestream. No feedback therefore cannot produce a successful preparation result.
- `rc_preparation_output_policy()` in `publication_guard.hpp` still requires session/RC/battery/preflight/ground evidence even for zero prestream output. Programmatic Arm explicitly rejects the RC-navigation route. Starting the software and polling STATUS confers no flight authority.
- This review is source/unit evidence. The dedicated isolated ARM64 VM task is responsible for final no-feedback native state, output counters and service identity; no actual hardware result is claimed here.

## Shutdown boundary the installer must respect

`fc01_system_node.cpp` lines 738-765 retains its existing shutdown behavior: exit promptly with current unarmed/landed evidence, otherwise wait until 60 seconds have elapsed unless healthy armed Offboard recovery still requires controlled output. With PX4 off/no samples, the original 30-second installer stop budget is too short. The maintenance installer should allow more than 60 seconds plus scheduling margin and still prove every old native process has exited before starting the candidate. Do not change native flight timeouts or use forced process termination to manufacture successful switching.

The original Supervisor may retain an owned Agent after native exit when no current ground evidence exists. It logs that explicitly; a shared Agent is always retained. The maintenance installer must tolerate this legitimate transport process and preserve executable/arguments/ownership checks, rather than require an established session or kill the Agent. On the next start, original `choose_agent()` can reuse the valid retained process; unknown/duplicate serial owners continue to refuse.

## Verification performed

Read-only source inspection of `navigation_prebuilt_service.py`, `standby.py`, `launcher.py`, `navigation_service.py`, native RC/preparation publication guards and native shutdown.

13 focused existing tests PASS in 1.685 seconds: all six prebuilt-service contracts, native path selection without old SDK binary, unknown-feedback display refusal, Agent loss/recovery preserving native, unknown UART ownership refusal, valid Agent reuse, orphan-controller exclusion and missing-UART standby retry. Temporary test peers are isolated subprocess fixtures; no host service/hardware was operated. No new tests or code changes are needed for the unchanged service path.

Immediate finding was also handed to root in `/tmp/fc01-maintenance-service-findings.txt`. Installer lifecycle and final ARM64 VM evidence remain separate workstreams.

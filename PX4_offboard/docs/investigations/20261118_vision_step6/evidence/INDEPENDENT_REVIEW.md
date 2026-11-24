# Step6 independent review (read-only, 2026-11-18)

Reviewed actual shared working tree at HEAD 19a5aa8c191feccc6cab7a7d274b8d1900a5443f. Other agents were still editing; final freeze/review remains necessary. No service/hardware operations, edits or commits made by this reviewer.

## Confirmed blocking defect (P2 release tooling)

`build_arm64_runtime.py` scan, lines 230-238 at review time: a dependency record is compared before adding its `package` member. First external-lib occurrence is stored with package; second occurrence of same external lib starts with only path/sha, so dictionary comparison always says changed. Almost every real multi-ELF package hits this. Reproduced by mocking two install ELF candidates whose ldd outputs reference the same external shared library; actual scanner raises `ValueError dependency changed during scan: /tmp/.../shared.so` despite unchanged bytes. Collector agent informed; fix by adding provenance before comparison and add scanner-level shared-dependency test. Existing 8 runtime tests test collect, not this actual scan condition.

## Acceptance gap requiring follow-up (P2)

`navigation_prebuilt_install.py:643-676 verify_active` checks unique native/controller + unit + runtime + DDS environment only. In track mode a native may live while `VisionAuxiliary` ROS launch exits or an indispensable bridge/mission/camera process crashes. `navigation_prebuilt_service.py:354-361` records observer/leader errors but does not affect installer acceptance; no child-health check currently exists. Therefore software-only INSTALL_OK can describe a broken enabled visual software chain. Avoid adding physical-camera/image or PX4/RC/DDS feedback as installation gates; instead verify required local auxiliary process identities/aliveness (including no-device camera and bridge/mission standby). A child may die while ros2 launch leader stays alive. Root informed to test actual track launch and injected software failure in VM, and decide correction before delivery. Runtime native TTL/fault behavior remains separate and should not be changed by deployment code.

## Reviewed behavior that is sound

- install/config_check/verify_active no longer require ground snapshots or six aircraft feedback streams. Migration occurs before stopping old service. stop_and_wait confirms actual native gone before start; restore preserves original unit/environment/runtime snapshot.
- visual flags only inherit as explicit paired booleans; track requires1.5m/manualRCslot1. Default false remains altitude. Mode configuration changes only visual fields and retains thrust/RC/DDS keys, with old deployment backup and latest INSTALL_STATE runtime hash update now present.
- no fabricated confirmed calibration in delivered site template. track requires explicit device, intrinsic/extrinsic provenance, dimensions, capture latency, target physical model and standoff. Missing hardware path does not block software config.
- only one native owner (standby); ROS launch contains camera/bridge/mission/read-only interface. Shared deployment mode lock prevents service startup while same-version mode config writes. auxiliary exits are explicit errors; native runtime safety gate is not replaced by status file.
- visual status binds boot/MainPID/schema/monotonic receipt and independently expires child observations. Older status cannot grant current native READY/navigation authorization.
- build candidate/source/install hashes and target Python/config/launch byte binding are explicit. Cache removal, path traversal, ARM64 ELF checks and original archive size bounds remain.
- dependency collector does not vendor glibc/loader/C++ baseABI; dependency licenses/provenance included. dlopen/plugin closure still needs final relocated runtime test.

## Actual tests executed here

`PYTHONPATH=.:tests python3 -m unittest tests.test_arm64_bundle tests.test_arm64_runtime tests.test_navigation_vision_mode tests.test_prebuilt_vision_status -v`

39 tests PASS, ~0.94s, `/tmp/fc01-step6-independent-tests.log`. Tests use fixtures; they do not prove actual ARM runtime install. ARM64 actual builder/VM verification belongs root, not claimed by this review.

## Re-review after corrective edits (2026-11-18 06:08 local)

Both findings above are addressed in current actual files:

1. `build_arm64_runtime.scan` now enriches package provenance before comparing repeated records. Added `test_scan_repeated_external_dependency_keeps_provenance_consistent` runs actual scan with two ELF fixtures sharing one unchanged external lib; passes.
2. `verify_active` now calls `verify_vision_software` for enabledtrack/currenttarget bundles. It requires fresh same-service/boot observer envelope, actual bundled ros2 launch process and unique camera/mission/bridge/read-only-interface descendants with pinned executable/script files. It accepts empty observations and does not read image/RC/DDS/flight values. Missing bridge, exitedleader,observererror,staleidentity tests reject. Altitude skips auxiliary entirely. Runtime safety behavior is unchanged: acceptance is installer-only and auxiliary failure does not force-kill native.

Re-executed actual newest tests: `PYTHONPATH=.:tests python3 -m unittest tests.test_arm64_runtime tests.test_prebuilt_vision_status tests.test_navigation_vision_mode -v`:35PASS,0.55s, log `/tmp/fc01-step6-independent-recheck.log`. New helper for truly dependency-free ET_DYN extensions reviews ELF bounded dynamic programheaders and rejects PT_INTERP/DT_NEEDED/staticexecutables/truncated objects; no generic ldd error bypass added.

No additional blocking issue identified by this re-review. Final actual ARMVM install/track-no-camera/relocation/failure/rollback tests are still necessary and remain root-owned. Softwareprocess matching tests are fixtures, not actual VM results.

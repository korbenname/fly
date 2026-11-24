# Narrow offline node stub compatibility and exact control-sink comparison

Production source unchanged. Edited only:
- native/tests/adapter_compile_check.py: NativeFlightState narrow message struct; KeepLast QoS, RCLError/logger macro, stamp assignment from synthetic Time to support the new readonly producer without disabling it.
- native/tests/test_rc_recovery_adapter.py: its Stamp fixture uses same explicit synthetic Time conversion.
- tests/test_vision_step3_node_sinks.py new: actual SystemNode nativeRC full lifecycle built separately from827f1f4 and candidate. Reuses existing powered driver. Preserves ALL TARGET/HEARTBEAT/COMMAND field bytes and order; explicit control-state fields exact; full JSON logs preserved rather than claiming additive diagnostics are byteidentical.

PASS recovery fixture 1test: /tmp/fc01-step3-stub-recovery.log.
PASS new fullnode sinkcomparison 1test: /tmp/fc01-step3-node-sinks.log. 1,178 sink lines, state samples707. Exact counts/digest in /tmp/fc01-vision-step3-node-sinks/RESULT.json (please use actualJSON ratherthan thishandtyped count ifdiffers).
Compilation flags -std=c++17 -Wall -Wextra -Werror -pedantic -pthread. Synthetic deterministicclock/ROS only, nohardware.

Original tests.test_stage3_control_equivalence versus f067 currently rerunning /tmp/fc01-step3-old-comparison.log. Expected intentional failure fullJSON additive fields; do not claimold7/7 pass unlessactuallyconfirmed. Keep original strictcomparison, no production changes to satisfy historical report.

Firstnew test command had FileExistsError due build-baseline directorycolliding witharchive; fixedtestpath tobuild-baseline/build-candidate, rawfailure preserved /tmp/fc01-step3-node-sinks-first-directory.log.

Do not stage original unrelateduntrackedmicovio/tools. No commitperformed.

## Final old compatibility result
Original historical 7 tests now compile:5pass/2FAIL as expected on intentionally additive JSON bytes (not missingstub anymore). Exactfailure log /tmp/fc01-step3-old-comparison.log. Explicit independent projection confirms BOTH original traces' target/heartbeat/command/all-sink lines remainexact vsf067: ground/faultnode558lines and powerednode1178lines; /tmp/fc01-step3-old-comparison/sink-only-historical-summary.json. Do not reportold7pass or hidefullJSONfailure. Existing tests unchanged; narrow dedicated newstep3test verifies exactcompatible control-state projection separately vs827baseline.

Syntax -fsyntax-only probe PASS /tmp/fc01-step3-stub-syntax.log; recoveryfixture1/1PASS; newstep3fullnodesinks1/1PASS. LimiteddiffcheckPASS. No productionedit, no Gitcommit.

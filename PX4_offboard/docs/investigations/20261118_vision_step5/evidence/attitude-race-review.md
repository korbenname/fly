# Step5 attitude sample causal race review

Read only; no repository files changed. Current HEAD747b987. Real failure /tmp/fc01-step5-final/normal/result.json vs earlier PASS /tmp/fc01-step5-pipeline-final-normal.

## Confirmed software race

Production mission `_on_platform_state` stores bridge latest attitude sample stamp and rotation; `_publish_command` encodes FRD with that rotation and sends that exact stamp. Bridge and native subscribe independently to BEST_EFFORT PX4 DDS. Normal ROS does not guarantee the native received or processed the same latest sample before mission envelope. Native `horizontal_snapshot` requires exact stamp in own64-entry history. Controller `velocity_valid` requires matched sample. After tracking_seen becomes true, any no-match immediately requests protected descent. This is a legitimate fail-closed guard but a producer/consumer coordination bug: mission selected a sample never proven available at consumer. Passing before tracking has waiting_for_attitude doesn't solve later race.

Failure original chronological evidence:
- tracking envelope690/691/692 valid at monotonic46021.3207/.3502/.3702, attitude stamps1794948279509932000/9531256000/9551361000.
- envelope693 valid at46021.40046 with attitude1794948279593941000.
- real native phase_event at46021.4027 immediately DESCENT `VISUAL_RECOVERY_HORIZONTAL_TRACK_LOST_OR_WITHDRAWN`, latest native accepted local sample1794948279573805us; mission693 sample is20.136ms newer than native local watermark (attitude same synchronized producer).
- mission remained valid until46021.43055 (694), first invalid695 at46021.46003, after native descended; thus mission invalidation was consequence, not initial cause. Later0.5s diagnostics show match true after delayed sample arrives, hiding the initial missing match. No revocation before first descent in the previous state; revocation1 appears after mission subsequent validfalse.

## Deterministic actual controller reproduction

Compile/run `/tmp/fc01-step5-stamp-race.cpp`, includes real production horizontal controller via existing test helpers. Logs `/tmp/fc01-step5-stamp-race.log`:
- active track then new valid envelope associated to fresh but not-yet-available sample -> protected descent.
- encode using exact fresh native-received prior sample -> active.
- same selected prior sample after original stateTTL -> protected descent. No freshness exemption.

## Recommended fix (mission side only)

Use current authenticated NativeFlightState.attitude_sample_stamp_us (published after native actually consumed it), find EXACT stamp in current bridge PlatformHistory attitude samples, obtain its ENU/FLU quaternion/rotation, encode FRD using that rotation and carry the selected exact sample stamp. NativeFlightState already has field; no wire change/native safety weakening. Do not interpolate acknowledged stamp from surrounding bridge history because exact native q comparison requires same raw sample. If exact same-source/same-origin/same-reset stamp absent or originally stale, explicitly withdraw valid=false with reason; never pick nearest or newest fallback. Current NativeFlightGate identity/provenance/time/reset/navigation gate remains prerequisite. Original native matched history TTL still revalidates sample/receive. Fresh source age check on mission is additional, not replacement.

`PlatformHistory.fields['attitude']` currently holds maxlen160 raw `(ns,xyzw)` entries; exact lookup available. Native state10Hz + native20Hz means sample may be~100-150ms old; original state_timeout~0.3s should accommodate without increasing. At50Hz raw64 history holds1.28s; sourceTTL tighter. At actual firmware attitude200Hz raw64 history~.32s, still assess state10Hz scheduling and target ARM conditions.

Risk: same raw stamp/bridge source is not message authentication; existing unique publisher/source/reset/origin gates preserved. Need match current platform/native counters before choosing. Source restart/state epoch already invalidates mission. With independent DDS packet loss, native sample might genuinely be absent in bridge history; proper withdrawal remains, safe. Frame/sensor loss and source reset must not keep old selected stamp forever. Test known sample freshness, NaN, missing exact, wrong origin/reset, new native state watermark, tilted 3D recovery, and final native safety guard unchanged.

This is candidate control behavior repair authorized Step5, should remain isolated candidate and report native no-vision unchanged. Physical flight/ARM64 untested.

## Review of root implemented shared-sample patch

Current mission diff now chooses `native_gate.latest.attitude_sample_stamp_us*1000` and exact `PlatformHistory.fields['attitude']` q, encodes with chosen rotation. This is safety-preserving and resolves the reproduced ahead-of-consumer race in equal-rate50Hz fixture. Missing/stale sample validfalse does not bypass any native checks.

**Availability limit to test before claiming hardware readiness:** real PX4 attitude DDS may run at100/200Hz while bridge publishes PlatformState50Hz. PlatformHistory only sees sampled snapshots (latest q each20ms), whereas native10Hz state can acknowledge a raw sample that bridge snapshot skipped. Even though bridge received that sample internally, mission has no q for its exact stamp. New code then correctly withdraws and can initiate descent. Equal-rate synthetic50Hz test does not cover this independent decimation race. Need unequal-rate/jitter callback test or realDDS simulation with separate attitude200Hz/local50Hz and bridge50Hz. Do not widen TTL or select nearest sample without proving same q.

Possible bounded designs if reproduced: expose validated raw attitude history from bridge in an independent read-only endpoint (mission exactlookup can see every raw attitude); or extend/version NativeFlightState to carry exact attitude quaternion with source/reset provenance so mission encodes acknowledged raw q directly. Prefer avoid new wire changes this stage unless measured issue; at minimum report missing exact sample as explicit `NATIVE_ACK_ATTITUDE_NOT_IN_PLATFORM_HISTORY` to distinguish from target loss.

Executed current actual `_publish_command` with native ack99995000us, bridge history stamps99990000000ns and100000000000ns (nativeack lies5ms between50Hz snapshots) and otherwise all gates current. Result validfalse; confirms availability gap can occur without stale feedback. This cannot honestly be called stable actual200Hz integration yet. Equal q values in example do not permit substitute because raw sample identities remain strict. Exact q in native state is robust causal solution; merely raising bridge publishHz improves odds but cannot prove no loss/jitter mismatch.

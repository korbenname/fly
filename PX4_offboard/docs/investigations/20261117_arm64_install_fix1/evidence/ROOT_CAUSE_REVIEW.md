# FC01 direct status entry bytecode / installer rejection review

Date: 2026-11-17. Read-only source review and isolated archive reproduction. No service, UART, PX4 or hardware operation, and no production file changes.

## Confirmed finding — P2 release installation blocker

The original bundle is byte-exact complete. Its strict tree verifier correctly rejects an additional ordinary file created after extraction:

`__pycache__/navigation_status.cpython-310.pyc`

`navigation.py` imports `navigation_status` at module top level. The documented invocation is `/usr/bin/python3 VERSION/navigation.py status`; this bypasses `prebuilt_boot.sh`, which is where `PYTHONDONTWRITEBYTECODE=1` is exported. Default CPython writes imported module bytecode beside the module. Even `navigation.py --help` does that import before argparse handles the help request.

Actual isolated reproduction with the final delivered original archive (SHA256 `3c876f6f98d45df994658ad3f68691c12dee0cf14befa18f2824a52f58d2a123`):

1. `navigation_prebuilt_install.extract_checked` accepts the archive into a fresh private temporary directory.
2. Remove `PYTHONDONTWRITEBYTECODE` only from a subprocess environment.
3. Execute `/usr/bin/python3 <entry>/navigation.py --help`.
4. Exit 0; the only added file is exactly `__pycache__/navigation_status.cpython-310.pyc`.
5. `verify_tree(entry, manifest)` now raises `发行文件集合与清单不一致`.

This proves the reported filename is reproducible from the published status entry and does not require a damaged transfer, mismatched ROS ABI, archive missing files or changes to flight control.

## User evidence and failure order

`install()` extracts/checks the new archive, then revalidates an already existing version directory before reading service state or calling `require_ground`, and far before `stop_and_wait` / privileged drop-in writes. A previous attempt may publish the independent version directory before failing a later preflight; running its status script then creates the cache. The next install fails at `verify_tree`.

The supplied output independently shows `fc01-navigation.service` still active with no drop-ins, WorkingDirectory and native executable under `FC01_OrangePi_Navigation_20261112_release`. Thus it does not show a successful switch to Stage 3. The new directory's `navigation.py status` reads the common `$HOME/FC01_navigation/status.json`, so running that script is not evidence its matching controller is installed or active.

The separate STALE display refuses to assert current aircraft state because topic freshness including retained snapshot age is insufficient. The quoted 0.9 seconds is display evidence, not proof the aircraft is current. Do not weaken timing checks to make installation pass; inspect current native timing / fresh feedback if this blocks the later ground preflight.

## Minimal durable fix recommendation

1. Before importing `navigation_status`, set `sys.dont_write_bytecode = True` in `navigation.py`. This prevents imported helper bytecode even for the user-friendly direct Python invocation. Keep existing boot and installer environment settings.
2. Do not ignore `__pycache__`, `.pyc`, all extra files or checksums in the manifest verifier. Existing rejection is serving its intended exact-payload purpose. Merely setting the environment when rerunning the installer does not remove a cache already present; merely setting `dont_write_bytecode` also does not prevent reading existing cache.
3. Reuse the actual verified ARM64 install tree to publish a distinct version/package (e.g. `20261117_stage3_installfix1`). Keep the failed old version directory intact. This avoids broad cleanup of an operator directory, touching a potentially running version, or silently accepting extra executable bytecode. Native/control bytes and compilation evidence can remain unchanged and bound to the same Stage 3 candidate.
4. Provide matched new installer SHA pin and package; user should use the official installer, not the diagnostic text-modified variant. Preserve all ground/RC/Agent gates and original service transaction order.

## Meaningful regression checks

- Fresh real archive extract → direct `navigation.py --help` and `status` with bytecode env absent → no unlisted cache → strict `verify_tree` still passes.
- Reinstall same version after direct status query → idempotent `already_active`, no restart or backup overwrite.
- Existing old failed version with this cache remains unchanged; independent new version installs.
- Arbitrary extra `.pyc`, source mutation, symlink or missing file must continue to fail original verification before stopping old service.
- Matching standalone installer syntax/source pin checks and strict actual archive readback.
- Control/native/config diff unchanged; full installer unit regressions; actual ARM library/service integration reuse where available.

No independent ARM or real-systemd install is claimed in this review; only original final-package isolated extraction and direct entry reproduction were executed here.

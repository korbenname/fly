# FC01 20261117 install fix1 — independent code/regression review

Result: PASS for the actual source changes and targeted regression checks reviewed on 2026-11-17. No production files changed by reviewer; no service, hardware, UART or flight operation.

Original root cause and precise archive reproduction: `/tmp/fc01-pycache-review.md`.

## Scope and safety

- `navigation.py` adds exactly one executable AST statement: `sys.dont_write_bytecode = True` before importing `navigation_status`. Independent AST comparison after removing that statement is identical to the previous committed source; comments aside, no status interpretation, systemd request, argument dispatch or control policy changed.
- All earlier project imports occur after this setting. Standard-library modules imported before it reside outside the immutable project directory. Python does not write the `__main__` script itself as a cached import.
- This setting prevents creation but does not erase/read-disable preexisting bytecode. The planned distinct new version directory is therefore the correct way to avoid the already contaminated old directory without deleting evidence or accepting unchecked caches.
- `verify_tree` still rejects the same exact file-set inequality. Only bounded (first 30 paths) missing/extra counts/list text is added. It does not ignore caches, tolerate arbitrary unlisted files, weaken path/link/hash/ELF checks, or touch version contents.
- `git diff adc55d0 -- native/src native/config native/tools launcher.py standby.py navigation_service.py` is empty. Installer transaction, ground/RC/Agent ownership and timing gates are unchanged by this patch.

## Independently executed checks

1. `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=tests:. python3 -m unittest test_navigation_prebuilt_install -v`: all **37** installer tests passed (0.925s). These are unit/backend-injection checks, not physical device validation.
2. The new direct-entry test launches a fresh real Python subprocess without PYTHONDONTWRITEBYTECODE/PYTHONPYCACHEPREFIX/PYTHONPATH. It runs actual `navigation.py --help` and actual imported entry's `main(['status'])`, replacing only read-only status sources. It verifies no .pyc/cache directory, unchanged strict tree, idempotent repeat installation, same backup and one initial stop.
3. The new rejection test explicitly creates unlisted `__pycache__/navigation_status.cpython-310.pyc`. Reinstallation still rejects before any new stop and leaves those bytes intact. Existing integrity/dependency/authority/rollback regressions also pass.
4. Additional independent isolated check copied actual new navigation.py/navigation_status.py bytes, executed `/usr/bin/python3 .../navigation.py --help` with all three bytecode/module environment variables removed, and compared complete ordinary-file names and SHA256 before/after. Exit0; exact files/bytes unchanged; no pyc files.
5. AST comparison confirmed only the above one statement addition in navigation.py. `git diff --check` passed.

## Artifact/VM boundary

At this code-review point, the new package was not yet generated. Matched final SHA pin, independent strict extraction and final actual ARM64 systemd install/directstatus/repeat/rollback are separate tasks. This report does not claim those results or Orange Pi/UART/XRCE/PX4 hardware validation.

Retain the old failed version/cache unchanged and clearly identify the new standalone version in deployment commands. Do not use new-directory status output alone as installation proof: it reads the shared runtime snapshot; real service ExecStart/WorkingDirectory and native /proc executable identity must match the new version.

## Final actual fix1 artifact independent review — PASS

Reviewed after source commit `4c85a9e7e0c66193104164f03f4bfe28de709b9d` and actual generation:

- `dist/arm64_fix1/FC01_OrangePi_20261117_stage3_fix1_arm64.tar.gz`: 6,498,104 bytes; SHA256 `5d0f2f150ca9ce70cb86138e512c0af2c768a3267a9ad48820bce60fdbfb1373`.
- `dist/arm64_fix1/install_fc01.sh`: SHA256 `c52f285e32241cf3a12a8daf67e60cc4ae8ff61a2afe91eb94648dd2626eecff`.
- Both actual hashes and archive length match DELIVERY.json. Manifest version is `20261117_stage3_fix1`, full source/control IDs match the pinned summary.
- Actual final archive was independently strictly extracted using the production `extract_checked`: 4,974 files, exact set/hash/path checks passed. Exactly 26 ELF files, all valid AArch64.
- Actual unpacked `navigation.py --help` executed with `/usr/bin/python3`, PYTHONDONTWRITEBYTECODE/PYTHONPYCACHEPREFIX/PYTHONPATH removed; exit0. Complete file-name and SHA256 maps before/after were identical. Subsequent production `verify_tree` passed. No implicit exemption or cleanup was used.
- Independent comparison with the original final archive shows only TWO payload byte changes: `navigation.py`, `navigation_prebuilt_install.py`. All installed files, all native source/config files, all message typesupport and all 26 ARM ELF files are byte-identical. Manifest identity changes separately as expected.
- `bash -n install_fc01.sh` passed. Embedded Python AST parses. Embedded installer exactly matches committed `4c85a9e:navigation_prebuilt_install.py` after the single intended replacement that adds this archive SHA as both default and sole allowed --expected-sha256 value. No other embedded change.

The artifact checks do not run a controller or service. ARM64 VM install/directstatus/repeat/rollback are the separate VM task; Orange Pi/UART/XRCE/PX4 hardware remains untested here.

# Maintenance installer actual diff: independent review

Reviewed current working-tree `navigation_prebuilt_install.py` and `build_arm64_bundle.py` on 2026-11-17. Read-only review; only `/tmp` reports/reproductions created, with no host service, hardware or repository edits during this review.

Final disposition: PASS. The first-install ancestor check is corrected, the readiness fixture now supplies source timing/action-policy facts, and all 71 focused tests pass. Core deployment lifecycle meets the requested separation from aircraft readiness.

## Resolved correction

`local_service_files()` now rejects every dedicated drop-in ancestor symlink in its first-unit branch using `(own, *own.parents)`. The new absent-unit plus symlinked `.d` regression passes and verifies no outside write/stop/start. This closes the previously reproduced privileged destination gap.

## Confirmed implementation behavior

- Transitive installer success paths (`install`, active/stopped repeat, `stop_and_wait`, `verify_active`, `restore`, `verify_original`, `rollback`) have no live IPC/aircraft freshness/ground/RC/Agent-session query. Removed helper names do not appear anywhere in the installer. Local Agent executable verification is retained.
- Existing stopped runtime is explicit and required; first unit creation does not generate default flight parameters. Static Environment/EnvironmentFiles and saved deployment settings provide DDS/Agent path with no old processes. Only the established domain-0 contract fills absent defaults.
- Full runtime keys survive except the existing three release identity fields. Source/runtime hashes are now bound across slow dependency and sudo waits, so operator edits are preserved and refused before stop.
- Foreign/duplicate/wrong-user controllers refuse. Old service is requested to stop gracefully; 120-second budget accommodates unchanged native 60-second no-feedback shutdown. Candidate start occurs only after inactive/failed plus MainPID 0 and no controller. No forced termination was added.
- Candidate software acceptance verifies real active unit, unique descended native process, exact candidate executable/hash/cwd, intended unit paths, complete runtime and inherited DDS environment, then checks process identity again. It does not require messages or flight readiness.
- Active repeat preserves PID and rollback point. Stopped repeat preserves the original backup, validates it, starts only after proof of no old process, and uses that original backup for failure recovery. Existing candidate with mismatched transaction refuses rather than becoming its own rollback baseline.
- Restore preserves original active/inactive state, restores complete runtime, retains unchanged main unit/drop-ins/environment files, and removes only newly created main unit/dedicated drop-in when the original unit was absent. Current backup validation explicitly rejects arbitrary absent-file deletion records. Independent temporary injected extra absent-file record was correctly refused before restore.
- Optional readiness JSON structural errors are now false/unknown display. Newly added `source_freshness()` accumulates actual native per-source ages, and action-policy gates are required only for display. These helpers are called only by `flight_readiness`, never deployment acceptance. This resolves the earlier display concern without reinstating install gates.

## Packaging path

`--reuse-archive` strictly extracts and verifies the immutable existing tar, reads installer source from committed Git HEAD, creates a separately pinned standalone script and records distinct archive-source and installer-source identities. It does not rewrite/recompile the tar or native control source. Its CLI rejects mixing reuse mode with build inputs. `write_installer` asserts exactly one replacement target, applies fixed default and sole allowed SHA, writes executable shell plus hash sidecar.

Independent temporary generation using the current source passed shell syntax, embedded Python compile, exact one-location fixed SHA replacement and sidecar digest. Final committed-source generation and final DELIVERY hashes must still be reviewed after root commits; the current VM script may intentionally identify an earlier worktree revision and should be reported by its actual SHA.

## Tests / scope

- Final maintenance installer suite: 54 PASS in 1.937 seconds, `/tmp/fc01-maintenance-display-tests-final.log`.
- Final combined installer/service/builder/feedback suite: 71 PASS in 12.114 seconds, `/tmp/fc01-maintenance-additional-combined.log`.
- Readiness true fixture now includes `source_timing`, `attitude_sample_fresh`, `land_fresh` and true action gates inside the original topic TTL. Added expiry, each closed gate and optional subprocess failure regressions all preserve software install success.
- Test source syntax and `git diff --check` passed. No production files were edited by this review agent.

Isolated real ARM64 VM no-feedback lifecycle results belong to the VM agent. No physical Orange Pi/PX4/flight qualification is inferred.

## Follow-up verification

The current source resolves the review findings: optional subprocess failure stays display-only; cached source TTLs age correctly; wrapper passes actual script name to rollback command; production software acceptance requires a one-second stable identity; stop timeout is recorded as `stop_incomplete`; rollback rechecks its drop-in after authentication and after stop; first-install drop-in ancestors reject symlinks.

The final test suite covers the stability reset, both rollback edit races, truthful incomplete-stop record/message, optional readiness TTL/gates/subprocess failures, and the first-unit symlink destination refusal. No outstanding correction remains from this independent review. Final committed-source generation, archive hash immutability and real ARM64 VM outcomes remain root/VM-agent delivery checks.

# Additional maintenance acceptance/display tests

Final disposition: PASS. Implemented requested follow-up in `tests/test_navigation_prebuilt_install.py` only. No production edits, commits, host-service operations or hardware operations by this agent.

Validation on 2026-11-17:

- Installer module: 54 tests PASS in 1.937 seconds, `/tmp/fc01-maintenance-display-tests-final.log`.
- Combined installer, prebuilt service, ARM64 bundle and feedback freshness suite: 71 tests PASS in 12.114 seconds, `/tmp/fc01-maintenance-additional-combined.log`.
- In-memory compilation of the test source and `git diff --check`: PASS.

Added/updated checks:

- Fixture explicitly sets `acceptance_stability_seconds = 0.0` for ordinary transaction tests. The real Backend declares 1.0 seconds.
- A deterministic clock test enables the 1.0-second window, removes the controller midway, and confirms the stability timer resets. It cannot accept until 1.8 simulated seconds, a complete second after the controller reappears, without wall-clock waiting.
- Rollback authentication that appends to the dedicated drop-in must refuse before another stop and preserve the edit. An edit occurring during stop must remain intact and must not start the old service. Both pass against production pre/post-stop pin checks.
- A 121-second delayed old exit leaves the old process and original unit intact, records `phase=stop_incomplete`, reports that the old service configuration was not modified, and does not falsely claim restoration. The earlier busy-controller regression now expects the same truthful outcome.
- Readiness positive fixture supplies all six actual source timings, sample/land booleans and action gates. It is accepted only inside the original per-source TTL. Source expiry, each closed action gate, identity mismatch and future/stale timestamps show not ready; software repeat still succeeds.
- Optional `systemctl` subprocess failure during readiness reporting yields false readiness without disturbing the already accepted software. Malformed JSON/absence CLI tests remain passing.
- First-install symlinked drop-in parent refuses before stop/write/start and preserves the outside directory. The regression originally exposed the missing ancestor check; production now checks `(own, *own.parents)` and all 54 installer tests pass.

No outstanding test or implementation finding remains from this review. Tests are ready for root to commit. Real ARM64 VM lifecycle evidence and final delivery generation belong to their respective owning agents/root; these unit results do not imply physical flight qualification.

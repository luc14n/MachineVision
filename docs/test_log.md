# Test Log

This document records test outcomes for both automated PC-side tests and
manual firmware validation.

---

## Format

### <YYYY-MM-DD> — <what was tested>
- Result: PASS / FAIL
- Notes: <relevant observations, environment details, and follow-ups>

---

### 2026-04-02 — Phase 0 repository bring-up baseline
- Result: PASS
- Notes: Initialized project structure for upcoming phases and prepared
  baseline documentation for tracking test outcomes. Added baseline firmware
  and PC-side config scaffolding, then executed automated tests.

### 2026-04-02 — Automated baseline config tests (`python -m pytest tests/ -v`)
- Result: PASS
- Notes: 9 tests collected, 9 passed in 0.08s on macOS (Python 3.13.5,
  pytest 8.3.4). Validated `pc_app/config.py` import stability and core
  defaults including `SERIAL_BAUD` and `POSITION_SENTINEL == (-1, -1)`.
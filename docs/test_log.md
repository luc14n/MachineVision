# Test Log

This document records test outcomes for both automated PC-side tests and
manual firmware validation.

---

## Format

### <YYYY-MM-DD> — <what was tested>
- Result: PASS / FAIL
- Notes: <relevant observations, environment details, and follow-ups>

---

## Phase 0 Hardware Bring-Up Verification Checklist

Use this checklist during OpenMV/Portenta bring-up to confirm all Phase 0
acceptance items are complete.

### Preconditions
- OpenMV IDE is installed and launches correctly.
- Portenta H7 + Vision Shield is connected via USB data cable.
- Board has OpenMV runtime flashed (or can be updated from IDE).

### Checklist
- [X] Board connects in OpenMV IDE without persistent errors.
- [X] Live camera framebuffer appears in OpenMV IDE.
- [ ] `firmware/main.py` opens and runs from the repository path.
- [ ] Serial terminal shows continuous coordinate output lines.
- [ ] No-detection case emits sentinel `-1,-1`.
- [ ] Detection caose emits normalized coordinate pairs (`nx,ny` in [0,1]).
- [ ] `firmware/calibrate.py` runs and prints LAB min/max updates.
- [ ] `firmware/color_config.py` can be updated from calibration values.
- [ ] Power-cycle behavior is understood (Run vs Deploy distinction verified).

### Result Template (copy for each hardware verification run)

### <YYYY-MM-DD> — Phase 0 hardware bring-up verification
- Result: PASS / FAIL
- Environment:
  - Board: Portenta H7 + Vision Shield
  - Host OS: <macOS version>
  - OpenMV IDE: <version>
  - Firmware runtime: <version/notes>
- Checklist Results:
  - Board connects in IDE: PASS / FAIL
  - Live framebuffer visible: PASS / FAIL
  - `main.py` runs from repo: PASS / FAIL
  - Serial streaming observed: PASS / FAIL
  - Sentinel `-1,-1` observed: PASS / FAIL
  - Normalized `nx,ny` observed: PASS / FAIL
  - `calibrate.py` updates LAB ranges: PASS / FAIL
  - `color_config.py` updated/tested: PASS / FAIL
  - Run vs Deploy behavior verified: PASS / FAIL
- Notes:
  - <serial port used>
  - <lighting conditions>
  - <any dropped frames / disconnect behavior>
  - <follow-up actions if FAIL>

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
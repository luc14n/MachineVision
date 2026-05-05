# Real-Time Predictive Ball Tracking System

This repository contains the full implementation plan and codebase for a real-time ball tracking pipeline using an **OpenMV-powered Portenta H7** camera and a **Python desktop application** for prediction and visualization.

The project is organized to support a phase-by-phase build from hardware bring-up to a demo-ready system and final MATH 470 paper deliverable.

---

## Project Goals

- Detect a green ball in camera frames on embedded hardware.
- Stream normalized 2D position data over serial in real time.
- Fit motion trajectories using SVD-based least squares.
- Predict near-future motion using a Kalman filter.
- Visualize measurements, fit, prediction, and confidence on a PC.
- Produce reproducible test logs and a documented engineering workflow.

---

## Repository Structure

```text
MachineVision/
├── firmware/              # OpenMV MicroPython scripts (board-side)
├── app/                   # Desktop Python application (SVD, Kalman, Visualization)
├── tests/                 # Unit/integration tests for app
├── docs/                  # Test logs, notes, supporting docs
├── tools/                 # Calibration/session helper scripts
├── DEVELOPMENT_PLAN.md    # Phase plan and technical specification
├── GIT_CONVENTIONS.md     # Required branching/commit/test workflow
├── MATH-470_Project.tex   # Course paper source
└── README.md              # This file
```

---

## Key Documents

- `DEVELOPMENT_PLAN.md`  
  Authoritative project plan, architecture, and acceptance criteria by phase.

- `GIT_CONVENTIONS.md`  
  Required git workflow, commit format, testing cycle, and merge policy.

Please treat those two files as governing documents for implementation work.

---

## Development Workflow Summary

1. Create a feature branch from `main` for each major change.
2. Implement one minor change at a time.
3. Commit implementation.
4. Run relevant tests.
5. Document test results in `docs/test_log.md`.
6. Commit test documentation.
7. Merge back to `main` with `--no-ff` only after full test pass.

See `GIT_CONVENTIONS.md` for exact commands and rules.

---

## Firmware (OpenMV / Portenta H7)

Firmware lives under `firmware/` and is intended to run on the board via OpenMV runtime.

Typical board-side responsibilities:
- camera initialization,
- green ball blob detection,
- normalized coordinate emission as serial lines (`nx,ny`),
- sentinel output when target is lost (e.g., `-1,-1`).

Use `firmware/README.md` for detailed OpenMV workflow and deployment steps.

---

## PC Application

The desktop app in `app/` is responsible for:
- serial ingestion and buffering,
- session record/replay tooling,
- SVD trajectory fitting,
- Kalman-based prediction,
- visualization and operator controls.

Testing code is under `tests/`.

---

## Phase Status

Current implementation should proceed according to `DEVELOPMENT_PLAN.md` phase ordering:

- Phase 0: Hardware bring-up & environment setup
- Phase 1: Firmware serial streaming
- Phase 2: PC serial reader and buffer
- Phase 3: SVD fitter
- Phase 4: Kalman predictor orchestration
- Phase 5: Visualization
- Phase 6: Integration and tuning
- Phase 7: Paper finalization

Refer to each phase’s acceptance criteria before advancing.

---

## Local Setup (PC Side)

Recommended baseline:

1. Install Python 3.10+.
2. Create a virtual environment.
3. Install dependencies from `requirements.txt`:
```bash
pip install -r requirements.txt
```
4. Run tests with:
```bash
python -m pytest tests/ -v
```

If tests are not yet present for a new change, add them before completing the change cycle.

---

## Notes on Data and Generated Files

- Raw session recordings and local machine artifacts should not be committed by default.
- Respect `.gitignore` entries for caches, logs, virtual environments, and generated outputs.
- Commit only intentional demo assets when explicitly needed.

---

## License

See `LICENSE`.

---

## Maintainer Notes

If you are contributing as an engineer/agent:

- Follow `GIT_CONVENTIONS.md` strictly.
- Keep `main` stable and demo-ready.
- Prefer small, testable increments with documented evidence in `docs/test_log.md`.
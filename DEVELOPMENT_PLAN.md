# Development Plan: Real-Time Predictive Ball Tracking System
**Project:** MATH 470 / CS Capstone — Predictive Object Tracking
**Author:** Lucian Rectanus — Frostburg State University
**Hardware:** Arduino Portenta H7 + Arduino Portenta Vision Shield
**Last Updated:** See Git history

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Repository Structure](#3-repository-structure)
4. [Coordinate & Naming Conventions](#4-coordinate--naming-conventions)
5. [Phase Breakdown](#5-phase-breakdown)
6. [Technical Specifications](#6-technical-specifications)
7. [Mathematical Framework](#7-mathematical-framework)
8. [Risk Register](#8-risk-register)
9. [Timeline](#9-timeline)
10. [Milestones & Deliverables](#10-milestones--deliverables)

---

## 1. Project Overview

A real-time predictive tracking system for a colored ball on a handheld stick. The Portenta Vision Shield camera detects the ball each frame and streams its position over USB serial to a host PC. The PC maintains a rolling position history, runs an SVD-initialized Kalman filter, and renders a live visualization — either on a plain background or overlaid on the camera feed — including a widening confidence ellipse cone over predicted future positions.

### Goals
| Goal | Target |
|------|--------|
| Detection frame rate | 30 FPS target, 15 FPS minimum |
| Prediction horizon | 5 steps ahead (configurable via `PREDICTION_HORIZON`) |
| Tracking loss behavior | Kalman predict-only up to `TRACKING_LOSS_MAX` steps, then reset to frame center |
| Visualization | Python/OpenCV app — plain background or live overlay |
| Confidence display | Widening 2D ellipse cone over predicted path |
| Motion handling | Smooth motion primary; sudden changes accepted with degraded accuracy |

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     PORTENTA H7 (OpenMV)                        │
│                                                                 │
│  Camera Capture (QVGA 320×240)                                  │
│       │                                                         │
│       ▼                                                         │
│  Color Isolation (LAB colorspace, green blob threshold)         │
│       │                                                         │
│       ▼                                                         │
│  Blob Detection → Centroid pixel (cx, cy)                       │
│       │                                                         │
│       ▼                                                         │
│  Normalize to [0.0, 1.0] → Serial TX: "cx,cy\n"                │
│  No detection             → Serial TX: "-1,-1\n"                │
└─────────────────────────────────────────────────────────────────┘
                                    │ USB (tethered, 115200 baud)
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                     HOST PC (Python)                            │
│                                                                 │
│  Serial Reader (background thread)                              │
│       │  Receives normalized (nx, ny) ∈ [0,1] or sentinel      │
│       │  Scales to pixel space: px = nx * DISPLAY_WIDTH,        │
│       │                         py = ny * DISPLAY_HEIGHT        │
│       ▼                                                         │
│  Position Buffer  (rolling N×2 matrix, pixel coordinates)       │
│       │                                                         │
│       ▼                                                         │
│  SVD Fitter  (batch, runs every M frames)                       │
│       │  → polynomial coefficients cx, cy                       │
│       │  → residual variance σ²x, σ²y  (feeds R matrix)        │
│       │  → velocity/accel estimates    (feeds Kalman init)      │
│       ▼                                                         │
│  Kalman Filter  (runs every frame)                              │
│       │  state: [px, py, vx, vy, ax, ay]ᵀ  (pixel space)       │
│       │  → predict() each frame                                 │
│       │  → update(measurement) when tracking                    │
│       │  → predict_ahead(k) for confidence cone                 │
│       ▼                                                         │
│  Visualizer  (OpenCV window)                                    │
│       │  → detected position, trail, predicted path, ellipses   │
│       │  → HUD: FPS, tracking status, velocity estimate         │
│       │  → toggle: plain background / live frame overlay        │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow Per Frame
1. **Portenta** detects blob → normalizes centroid to `[0,1]` → sends `"nx,ny\n"` or `"-1,-1\n"`.
2. **Serial reader** receives value → scales to pixel space → pushes to position buffer.
3. **SVD fitter** re-fits every `SVD_REFIT_INTERVAL` frames; updates `R` and Kalman init state.
4. **Kalman filter** runs `predict()` every frame; runs `update()` only when tracking is active.
5. **Visualizer** composites all overlays and displays the result.

---

## 3. Repository Structure

```
MachineVision/
│
├── firmware/                        # Source files for the Portenta H7 — Git is the source of truth
│   ├── main.py                      # Camera init, blob detect, normalize, serial TX
│   ├── color_config.py              # LAB color thresholds for green ball (edit here to retune)
│   ├── calibrate.py                 # Runs ON the board via OpenMV IDE to find LAB thresholds interactively
│   └── README.md                    # OpenMV IDE workflow, flashing steps, and file sync instructions
│
├── pc_app/                          # Runs on host PC
│   ├── main.py                      # Entry point: wires serial reader + predictor + visualizer
│   ├── serial_reader.py             # Background thread: reads serial, scales to pixel space, queues
│   ├── position_buffer.py           # Rolling N×2 pixel-coordinate matrix
│   ├── svd_fitter.py                # SVD least squares: fit, predict, residuals
│   ├── kalman_filter.py             # Kalman filter: predict, update, predict_ahead, ellipse
│   ├── predictor.py                 # Orchestrator: SVD init + Kalman execution + loss handling
│   ├── visualizer.py                # All OpenCV rendering: trail, cone, HUD, mode toggle
│   └── config.py                    # Single source of truth for all tunable parameters
│
├── tests/                           # Offline unit tests — no hardware required
│   ├── mock_serial.py               # Simulates serial stream from a .csv or a generator function
│   ├── test_position_buffer.py
│   ├── test_svd_fitter.py
│   └── test_kalman_filter.py
│
├── tools/
│   ├── record_session.py            # Saves a live serial session to .csv (runs on PC)
│   └── replay_session.py            # Feeds a .csv into pc_app in place of live serial (runs on PC)
│
├── docs/
│   ├── math_derivations.md          # Full SVD + Kalman derivations (source for MATH 470 paper)
│   ├── calibration_guide.md         # Lighting setup and LAB threshold tuning walkthrough
│   └── system_diagram.png
│
├── MATH-470_Project.tex             # Original proposal (do not modify)
├── DEVELOPMENT_PLAN.md              # This file
├── requirements.txt                 # Python dependencies (keep in sync with Section 6)
└── README.md                        # Setup, flash, and run instructions
```

---

## 4. OpenMV IDE Workflow

This section defines exactly how OpenMV IDE interacts with the project directory. Read it before Phase 0.

### 4.1 The File Sync Model

The board has its own internal flash filesystem, completely separate from your PC's filesystem. OpenMV IDE bridges the two. The rule for this project is:

**`firmware/` in the Git repo is the single source of truth. Never edit files directly on the board.**

| Location | Purpose |
|----------|---------|
| `firmware/main.py` (repo) | Edited in your normal code editor; version-controlled |
| `firmware/color_config.py` (repo) | Edited after calibration; version-controlled |
| `firmware/calibrate.py` (repo) | Calibration helper; opened and run in OpenMV IDE |
| `main.py` (board root) | Deployed copy — updated from repo, not edited directly |
| `color_config.py` (board root) | Deployed copy — updated from repo, not edited directly |

### 4.2 The Development Loop (Run Without Deploying)

During active development, you do not need to write files to the board's flash. OpenMV IDE's **Run** button sends the currently open file to the board's RAM and executes it immediately. Nothing is written to flash, and the board reverts to its previous deployed state on power cycle.

**Workflow for iterating on `firmware/main.py`:**
1. Open `firmware/main.py` from the project directory in OpenMV IDE (`File → Open`)
2. Edit the file in your normal code editor (VS Code, etc.) and save it
3. Switch to OpenMV IDE and click **Run** (the file on disk is re-read each time)
4. Observe output in the OpenMV serial console
5. Repeat from step 2

OpenMV IDE does not need to be your primary editor — it only needs to be open with the file loaded so you can click Run.

### 4.3 Deploying to the Board (Run on Boot Without IDE)

When you want the board to run `main.py` automatically on power-up (e.g., for the live demo), deploy the files to the board's flash:

1. In OpenMV IDE, open `firmware/main.py` from the repo
2. Go to **Tools → Save open script to OpenMV Cam** — this writes `main.py` to the board root
3. Repeat for `firmware/color_config.py`: open it, then **Tools → Save open script to OpenMV Cam**
4. Power-cycle the board — it will now run `main.py` automatically without the IDE

Alternatively, the board appears as a USB drive. You can copy `main.py` and `color_config.py` directly from `firmware/` to the board's root in Finder/Explorer.

### 4.4 Color Calibration (`firmware/calibrate.py`)

`firmware/calibrate.py` is a separate MicroPython script that runs **on the board** to find LAB threshold values interactively. It is not part of the main firmware loop.

**Workflow:**
1. Open `firmware/calibrate.py` in OpenMV IDE
2. Click **Run** — it streams LAB values for the current camera view to the serial console
3. Point the camera at the green ball; note the L, A, B min/max values
4. Update the threshold constants in `firmware/color_config.py` in the repo
5. Commit `color_config.py`

This file lives in `firmware/` (not `tools/`) because it runs on the board as MicroPython, not on the PC as Python.

---

## 5. Coordinate & Naming Conventions

This section exists to prevent implementation bugs caused by coordinate or naming ambiguity. Read it before writing any code.

### 5.1 Coordinate Pipeline

| Stage | Space | Range | Notes |
|-------|-------|-------|-------|
| Firmware blob centroid | Pixel | `(0..319, 0..239)` | Raw from `find_blobs()` |
| Serial transmission | Normalized float | `[0.0, 1.0]` | `nx = cx/320`, `ny = cy/240` |
| PC after serial read | Pixel (display) | `(0..DISPLAY_WIDTH-1, 0..DISPLAY_HEIGHT-1)` | `px = nx * DISPLAY_WIDTH` |
| Position buffer | Pixel (display) | Same as above | All math runs in display pixel space |
| Kalman state | Pixel (display) | Same as above | Units: pixels and pixels/frame |

**Rule:** All computation on the PC side (SVD, Kalman, visualizer) uses display pixel coordinates. Normalization only exists to decouple the firmware resolution from the display resolution. Never store normalized coordinates in the position buffer.

### 5.2 Sentinel Value

`-1,-1\n` is the tracking loss signal. The PC serial reader checks for this before scaling. `-1` must never be scaled to pixel space; it must set `is_tracking = False` in the buffer.

### 5.3 Matrix Naming — SVD

The SVD decomposition is written as:

$$\mathbf{A} = \mathbf{U} \mathbf{\Sigma} \mathbf{W}^T$$

where **$\mathbf{A}$** is the Vandermonde matrix (replaces the ambiguous $\mathbf{V}$ used in some textbooks). This avoids the collision between "Vandermonde" and the right singular vectors. The pseudoinverse is:

$$\mathbf{A}^+ = \mathbf{W} \mathbf{\Sigma}^{-1} \mathbf{U}^T$$

Use $\mathbf{A}$, $\mathbf{U}$, $\mathbf{\Sigma}$, $\mathbf{W}$ consistently in all code, comments, and the paper.

### 5.4 `k` vs `TRACKING_LOSS_MAX`

These are two distinct parameters that happen to share the same default value (5). They must not be conflated:

| Parameter | Config name | Meaning |
|-----------|-------------|---------|
| Prediction horizon | `PREDICTION_HORIZON` | Number of future steps predicted and visualized every frame, always |
| Tracking loss limit | `TRACKING_LOSS_MAX` | Number of consecutive frames without detection before the Kalman state is reset to frame center |

### 5.5 "Frame Center" Reset

When tracking loss exceeds `TRACKING_LOSS_MAX`, the Kalman state resets to:
- Position: `(DISPLAY_WIDTH / 2, DISPLAY_HEIGHT / 2)`
- Velocity: `(0, 0)`
- Acceleration: `(0, 0)`
- Covariance `P`: reset to initial high-uncertainty diagonal (defined in `config.py` as `KALMAN_INIT_P`)

The filter will then wait for the first valid detection to run a proper `update()` and converge.

---

## 6. Phase Breakdown

Phases must be completed in order. Each phase has explicit acceptance criteria — do not move to the next phase until all criteria are met.

---

### Phase 0 — Hardware Bring-Up & Environment Setup
**Duration:** 2–3 days
**Goal:** Confirm the hardware pipeline works end-to-end and all tools are in place before any algorithm work begins.

#### Tasks
- [x] Install OpenMV IDE; flash OpenMV firmware onto the Portenta H7 + Vision Shield using the OpenMV IDE firmware updater
- [x] In OpenMV IDE, open `File → New File`, run a snapshot hello-world: confirm live camera feed is visible in the frame buffer panel
- [ ] Create the full folder structure from Section 3 in the GitHub repo; initial commit (can be empty placeholder files)
- [ ] Write `firmware/calibrate.py`: streams LAB values for the center region of each frame to the serial console
- [ ] Open `firmware/calibrate.py` in OpenMV IDE (`File → Open`, navigate to the project directory); click **Run**; point camera at the green ball; record L, A, B min/max values from the serial console output
- [ ] Write the recorded threshold values into `firmware/color_config.py` in the repo; commit
- [ ] Write `firmware/main.py` stub: imports `color_config.py`, captures a frame, finds any blob meeting the thresholds, prints the centroid to the serial console
- [ ] Open `firmware/main.py` in OpenMV IDE (`File → Open`); click **Run**; confirm centroid values print in the serial console while the green ball is in frame
- [ ] Set up Python virtual environment on PC; install `pyserial`, `numpy`, `opencv-python`, `scipy`
- [ ] Freeze dependencies into `requirements.txt`

#### Acceptance Criteria
- OpenMV IDE shows a live camera feed in the frame buffer panel
- `firmware/calibrate.py` streams LAB values when run via OpenMV IDE Run button
- `firmware/color_config.py` contains threshold values derived from the actual ball under expected lighting
- OpenMV serial console prints blob centroid values when `firmware/main.py` is run via the Run button
- Python environment installs cleanly from `requirements.txt`
- GitHub repo has the full folder structure committed, including `firmware/calibrate.py` and `firmware/color_config.py`

---

### Phase 1 — Firmware: Serial Streaming
**Duration:** 2–3 days
**Goal:** Stream normalized centroid coordinates to the PC over USB serial at ≥30 FPS.

#### Tasks
- [ ] Extend `firmware/main.py` (edit in your code editor, run via OpenMV IDE Run button):
  - Select the largest qualifying blob (area above `MIN_BLOB_AREA`, defined in `color_config.py`) to reject noise
  - Normalize centroid: `nx = cx / img.width()`, `ny = cy / img.height()`
  - On valid detection: send `"nx,ny\n"` (two floats, 4 decimal places)
  - On no detection: send `"-1,-1\n"`
  - Target loop rate: 30 FPS — profile with OpenMV's `clock` object and print FPS to serial console during testing
- [ ] Set UART/serial baud rate to 115200 in `firmware/main.py` and confirm it matches `SERIAL_BAUD` in `pc_app/config.py`
- [ ] On the PC side, open a terminal (`python -m serial.tools.miniterm /dev/tty.usbmodemXXXX 115200`) and confirm the expected output format at ≥30 FPS while `firmware/main.py` is running via the OpenMV IDE Run button
- [ ] Write `firmware/README.md` covering: (1) flashing the OpenMV runtime, (2) the Run-button development loop, (3) deploying to board flash for standalone use, (4) how to re-run calibration

#### Resolution Strategy
Start at **QVGA (320×240)**. OpenMV's `find_blobs()` is hardware-assisted and fast at this resolution. Only increase resolution if the ball is physically too small to reliably detect (e.g., tracking from a large distance). Do not increase resolution to "improve quality" — it will reduce FPS.

#### Acceptance Criteria
- PC terminal displays `nx,ny\n` values in `[0,1]` range at ≥30 FPS while ball is in frame
- PC terminal displays `-1,-1\n` when ball is removed from frame
- No other output formats appear on the serial line

---

### Phase 2 — PC: Serial Reader, Position Buffer & Session Tools
**Duration:** 3 days
**Goal:** Reliable serial ingestion, correct pixel-space conversion, rolling buffer, and the record/replay tooling that all subsequent phases depend on for offline testing.

#### Tasks
- [ ] Implement `pc_app/serial_reader.py`:
  - Background thread reads serial line-by-line (non-blocking to main loop)
  - Parses `"nx,ny\n"`: checks for sentinel first; if sentinel, pushes `None`; otherwise scales to pixel space (`px = nx * DISPLAY_WIDTH`, `py = ny * DISPLAY_HEIGHT`) and pushes `(px, py)` to a `queue.Queue`
  - Timestamps each entry with `time.monotonic()`
  - Handles serial disconnect gracefully (logs warning, does not crash)
- [ ] Implement `pc_app/position_buffer.py`:
  - Maintains a rolling `(N, 2)` numpy array in **pixel coordinates**
  - `push(px, py)`: right-shifts all columns, inserts new position at index 0 (most recent)
  - `push(None)`: right-shifts, inserts `np.nan` at index 0; sets internal `_loss_count += 1`
  - `get_matrix()`: returns the full `(N, 2)` array (may contain `NaN` entries)
  - `get_valid_matrix()`: returns only rows without `NaN`
  - `is_tracking()`: returns `True` if the most recent entry is not `NaN`
  - `loss_count()`: returns consecutive frames since last valid detection
- [ ] Implement `tests/mock_serial.py`: a class that mimics `serial_reader.py`'s queue interface by feeding from a list, generator, or `.csv` file
- [ ] Implement `tests/test_position_buffer.py`: test push, right-shift, NaN handling, and `is_tracking()` transitions
- [ ] Implement `tools/record_session.py`: connects to serial, records `timestamp,nx,ny\n` rows to a `.csv` file in `tools/sessions/`; stops on `Ctrl+C`; requires `firmware/main.py` to be running on the board (via Run button or deployed to flash)
- [ ] Implement `tools/replay_session.py`: reads a recorded `.csv` and feeds it through `mock_serial.py` into `pc_app/main.py` at the original timestamps (or at a configurable speed multiplier)

#### Why record/replay is built here
`record_session.py` and `replay_session.py` depend only on the serial format defined in Phase 1. All subsequent phases (3, 4, 5) should be developed and tested entirely using replay mode — no hardware required after this point until Phase 6.

#### Acceptance Criteria
- Buffer correctly right-shifts on every push; oldest entry is dropped
- `NaN` entries are inserted on sentinel; `is_tracking()` returns `False` immediately after
- `loss_count()` increments correctly on consecutive sentinels, resets on valid detection
- Unit tests pass without hardware
- `record_session.py` produces a valid `.csv`; `replay_session.py` feeds it back without errors

---

### Phase 3 — SVD Least Squares Fitter
**Duration:** 3–4 days
**Goal:** Fit parametric polynomial models $\hat{x}(t)$ and $\hat{y}(t)$ to the position history using SVD-based least squares. This is the primary mathematical content for MATH 470.

#### Mathematical Approach

Time is represented as the column index in the position buffer: index 0 = most recent, index $N-1$ = oldest. To build a polynomial fit, use the reversed time vector $\mathbf{t} = [N-1, N-2, \dots, 1, 0]^T$ so that $t=0$ maps to the most recent observation. This matches the proposal's convention and keeps prediction at negative $t$ values (future) consistent.

Build the Vandermonde matrix $\mathbf{A}$ (degree $d$, $N$ rows):

$$\mathbf{A} = \begin{bmatrix} 1 & t_0 & t_0^2 & \cdots & t_0^d \\ 1 & t_1 & t_1^2 & \cdots & t_1^d \\ \vdots & & & & \vdots \\ 1 & t_{N-1} & t_{N-1}^2 & \cdots & t_{N-1}^d \end{bmatrix}, \quad \mathbf{A} \in \mathbb{R}^{N \times (d+1)}$$

Compute the economy SVD:

$$\mathbf{A} = \mathbf{U} \mathbf{\Sigma} \mathbf{W}^T$$

Compute the pseudoinverse (zero singular values below `SVD_SINGULAR_THRESHOLD`):

$$\mathbf{A}^+ = \mathbf{W} \mathbf{\Sigma}^{-1} \mathbf{U}^T$$

Solve for polynomial coefficients in $x$ and $y$ independently:

$$\mathbf{c}_x = \mathbf{A}^+ \mathbf{p}_x, \qquad \mathbf{c}_y = \mathbf{A}^+ \mathbf{p}_y$$

where $\mathbf{p}_x$ and $\mathbf{p}_y$ are the pixel-coordinate columns of the valid position matrix.

Predict at time $\tau$ (use $\tau < 0$ for future steps relative to now):

$$\hat{x}(\tau) = \sum_{j=0}^{d} c_{x,j} \cdot \tau^j, \qquad \hat{y}(\tau) = \sum_{j=0}^{d} c_{y,j} \cdot \tau^j$$

Residual variance (used to set the Kalman measurement noise matrix $\mathbf{R}$):

$$\sigma^2_x = \frac{1}{N} \|\mathbf{A}\mathbf{c}_x - \mathbf{p}_x\|^2, \qquad \sigma^2_y = \frac{1}{N} \|\mathbf{A}\mathbf{c}_y - \mathbf{p}_y\|^2$$

#### Extracting Velocity and Acceleration for Kalman Initialization

At $t=0$ (most recent frame), the polynomial derivatives give:

$$\hat{v}_x = c_{x,1}, \quad \hat{a}_x = 2 c_{x,2}, \qquad \hat{v}_y = c_{y,1}, \quad \hat{a}_y = 2 c_{y,2}$$

These six values $[\hat{x}(0),\, \hat{y}(0),\, \hat{v}_x,\, \hat{v}_y,\, \hat{a}_x,\, \hat{a}_y]^T$ initialize the Kalman state vector (see Phase 4).

#### Tasks
- [ ] Implement `pc_app/svd_fitter.py`:
  - `fit(valid_positions)`: builds $\mathbf{A}$, computes SVD via `numpy.linalg.svd(full_matrices=False)`, applies threshold to $\mathbf{\Sigma}$, returns `(cx, cy, sigma2_x, sigma2_y)`
  - `predict(cx, cy, tau)`: evaluates polynomials at scalar or array `tau`
  - `get_init_state(cx, cy)`: returns the 6-element Kalman init vector using the derivative formulas above
  - Guard: if `valid_positions` has fewer than `d+2` rows, return `None` (not enough data to fit)
- [ ] Default polynomial degree: `SVD_POLY_DEGREE = 2`. Configurable in `config.py`.
- [ ] Write `tests/test_svd_fitter.py`:
  - Known parabolic trajectory: verify coefficients recover exactly
  - Noisy linear trajectory: verify residuals are non-zero and reasonable
  - Edge case: fewer points than `d+2` returns `None` without error
- [ ] Document full derivation in `docs/math_derivations.md`

#### Acceptance Criteria
- Fitter recovers known polynomial trajectories within floating-point tolerance in unit tests
- `None` is returned cleanly when there is insufficient data
- Residual variances are positive and correctly scale with injected noise in tests
- `get_init_state()` returns derivatives consistent with the fitted coefficients

---

### Phase 4 — Kalman Filter & Predictor Orchestrator
**Duration:** 4–5 days
**Goal:** Implement the Kalman filter and the `predictor.py` orchestrator that ties SVD initialization to Kalman execution.

#### State Vector (pixel space)

$$\mathbf{x}_k = [p_x,\; p_y,\; v_x,\; v_y,\; a_x,\; a_y]^T$$

All units are pixels and pixels/frame. $\Delta t = 1$ frame throughout (time is measured in frames, consistent with the position buffer's column indexing).

#### Kalman Matrices

**State Transition** $\mathbf{F}$ (constant acceleration model):

$$\mathbf{F} = \begin{bmatrix} 1 & 0 & 1 & 0 & \tfrac{1}{2} & 0 \\ 0 & 1 & 0 & 1 & 0 & \tfrac{1}{2} \\ 0 & 0 & 1 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 & 0 & 1 \\ 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 & 0 & 1 \end{bmatrix}$$

**Observation Matrix** $\mathbf{H}$ (position only):

$$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \end{bmatrix}$$

**Measurement Noise** $\mathbf{R}$ (set from SVD residuals):

$$\mathbf{R} = \begin{bmatrix} \sigma^2_x & 0 \\ 0 & \sigma^2_y \end{bmatrix}$$

If SVD returns `None` (insufficient data), use the fallback `KALMAN_MEAS_NOISE_DEFAULT` scalar times $\mathbf{I}_2$.

**Process Noise** $\mathbf{Q}$: diagonal matrix, scale factor `KALMAN_PROCESS_NOISE` applied to each diagonal element. This is the primary tuning parameter for responsiveness vs. smoothness.

#### Adaptive Process Noise Rule

Every time SVD re-fits (every `SVD_REFIT_INTERVAL` frames), compute the mean residual variance:

$$\bar{\sigma}^2 = \frac{\sigma^2_x + \sigma^2_y}{2}$$

Scale `Q` as:

$$\mathbf{Q}_{\text{scaled}} = \text{clip}\!\left(\frac{\bar{\sigma}^2}{\sigma^2_{\text{ref}}},\; Q_{\text{min}},\; Q_{\text{max}}\right) \cdot \mathbf{Q}_{\text{base}}$$

where `KALMAN_Q_REF`, `KALMAN_Q_MIN`, `KALMAN_Q_MAX` are defined in `config.py`. This is the complete rule — implement it exactly.

#### Confidence Ellipse Computation

After `predict_ahead(k)` returns a list of `(px, py, P_k)` tuples, extract the $2 \times 2$ positional submatrix from each `P_k`:

$$\mathbf{P}_{pos} = \mathbf{P}_k[0\!:\!2,\; 0\!:\!2]$$

Compute eigenvalues $\lambda_1 \geq \lambda_2$ and eigenvectors of $\mathbf{P}_{pos}$. Scale by the chi-squared critical value for 2 DOF at the configured confidence level:

$$s = \sqrt{\chi^2_{2,\, \alpha}}, \quad \text{e.g.,} \quad s = \sqrt{5.991} \approx 2.448 \text{ for } 95\%$$

Ellipse semi-axes: $a = s\sqrt{\lambda_1}$, $b = s\sqrt{\lambda_2}$. Rotation angle: angle of the first eigenvector. Use `scipy.stats.chi2.ppf(CONFIDENCE_LEVEL, df=2)` for the critical value. This is why `scipy` is a dependency.

#### SVD–Kalman Initialization and Re-fit Logic (lives in `predictor.py`)

| Condition | Action |
|-----------|--------|
| Startup or post-reset | Run SVD on all valid buffer entries. If SVD returns `None` (< `d+2` points), initialize Kalman at frame center with zero velocity, high-uncertainty `P`. |
| Every `SVD_REFIT_INTERVAL` frames | Re-run SVD; update `R` with new residuals; update adaptive `Q`; do NOT reset the Kalman state. |
| Tracking active | Call `kalman.predict()` then `kalman.update(measurement)` every frame. |
| Tracking lost (sentinel received) | Call `kalman.predict()` only (no update); increment `loss_count`. |
| `loss_count > TRACKING_LOSS_MAX` | Reset Kalman state to frame center (see Section 4.5); clear buffer; wait for new detections. |

#### Tasks
- [ ] Implement `pc_app/kalman_filter.py`:
  - `__init__(state, P, F, H, Q, R)`: stores all matrices as numpy arrays
  - `predict()`: $\mathbf{x} = \mathbf{F}\mathbf{x}$, $\mathbf{P} = \mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$; returns `(px, py)`
  - `update(meas)`: standard Kalman gain + state + covariance update; returns corrected `(px, py)`
  - `predict_ahead(k)`: runs `k` sequential `predict()` steps on a copy of the current state; returns list of `(px, py, P_copy)` without modifying the live filter state
  - `get_ellipse(P_pos, confidence)`: computes and returns `(cx, cy, a, b, angle_deg)` using `scipy.stats.chi2.ppf`
  - `reset(center_px, center_py, init_P)`: resets state to frame center with zero velocity/acceleration
- [ ] Implement `pc_app/predictor.py`:
  - Holds one `SVDFitter`, one `KalmanFilter`, and the `PositionBuffer`
  - `step(new_position_or_None)`: pushes to buffer, runs SVD if due, runs Kalman predict+update or predict-only, handles reset logic
  - Returns a single dict: `{"current": (px,py), "predicted": [(px,py), ...], "ellipses": [(cx,cy,a,b,angle), ...], "tracking": bool, "loss_count": int}`
- [ ] Write `tests/test_kalman_filter.py`:
  - Constant velocity trajectory: verify state converges within 10 frames
  - Tracking loss: verify `predict_ahead` does not modify live state
  - Reset: verify state is frame center after reset
- [ ] Document all Kalman equations in `docs/math_derivations.md`

#### Acceptance Criteria
- Filter converges on a constant-velocity test trajectory within 10 frames in unit tests
- `predict_ahead(k)` does not mutate the live filter state (test explicitly)
- Confidence ellipses are valid (positive semi-axes, angle in `[-90, 90]`) for all `k` steps
- Adaptive `Q` scaling produces a larger `Q` for high-residual inputs and a smaller `Q` for low-residual inputs (verified in unit test)
- `predictor.step()` returns the correct dict structure for both tracking and non-tracking inputs

---

### Phase 5 — Visualization Application
**Duration:** 3–4 days
**Goal:** OpenCV desktop application that renders the tracking and prediction overlays. Develop entirely in replay mode using a recorded session from Phase 2.

#### Rendering Layers (drawn in this order, bottom to top)
1. **Background**: live camera frame (overlay mode) or black canvas (plain mode)
2. **Historical trail**: last `TRAIL_LENGTH` valid positions as fading dots (white → gray)
3. **Current position**: filled green circle at most recent detected position
4. **Predicted path**: line connecting the `PREDICTION_HORIZON` predicted positions (blue)
5. **Predicted position dots**: small dots at each predicted step (blue)
6. **Confidence ellipses**: drawn at each predicted step, color-graded green → yellow → red as step index increases
7. **HUD**: top-left text block — FPS, tracking status (`TRACKING` / `LOST: N frames`), velocity magnitude in px/frame

#### Technology
Use **OpenCV** (`cv2.imshow`) as the sole rendering backend. The window is managed in `main.py`'s event loop. Do not add PyQt6 unless rendering performance is a measured problem (it won't be for this resolution).

#### Keyboard Controls
| Key | Action |
|-----|--------|
| `o` | Switch to live overlay mode |
| `b` | Switch to plain background mode |
| `+` / `-` | Increase / decrease `PREDICTION_HORIZON` by 1 (min 1, max 20) |
| `q` | Quit |

#### Tasks
- [ ] Implement `pc_app/visualizer.py`:
  - `draw_frame(bg_frame_or_None, predictor_output, config)`: takes the full predictor output dict and returns a composited numpy frame ready for `cv2.imshow`; pass `None` for background to use plain mode
  - `_draw_trail(frame, buffer)`: draws `TRAIL_LENGTH` most recent valid positions with linear alpha fade
  - `_draw_cone(frame, predicted, ellipses)`: draws path line, dots, and color-graded ellipses
  - `_draw_hud(frame, stats)`: draws FPS and tracking status text
- [ ] Implement `pc_app/main.py`: main event loop that reads from serial (or mock), calls `predictor.step()`, calls `visualizer.draw_frame()`, calls `cv2.imshow()`, handles keyboard input
- [ ] Test the full PC pipeline in replay mode using a session recorded in Phase 2 before any hardware is connected
- [ ] Run `replay_session.py` at 2× speed to verify the system does not drop frames

#### Acceptance Criteria
- Application runs at ≥30 FPS in replay mode on the host PC (measure with OpenCV's `getTickCount`)
- Confidence cone visibly widens with each predicted step
- Both display modes render correctly with the `o`/`b` toggle
- `PREDICTION_HORIZON` adjusts in real time with `+`/`-`
- Application exits cleanly on `q` and on serial disconnect

---

### Phase 6 — Hardware Integration, Tuning & Demo Prep
**Duration:** 3–4 days
**Goal:** Connect the Portenta and run the full system live. Tune parameters. Prepare the demo.

#### Tasks
- [ ] Connect Portenta H7 via USB; update `SERIAL_PORT` in `config.py` to match the system port
- [ ] Run `tools/calibrate_color.py` again in the actual demo location/lighting; update `firmware/color_config.py` if thresholds have drifted
- [ ] Run full system; confirm end-to-end FPS meets ≥30 FPS target in the demo environment
- [ ] Tune `KALMAN_PROCESS_NOISE` (base `Q`): start at `1e-2`, increase if prediction is too sluggish during fast motion, decrease if prediction is too noisy during slow motion
- [ ] Tune `SVD_REFIT_INTERVAL`: start at 10 frames; reduce if tracking feels slow to adapt, increase if compute budget is tight
- [ ] Test tracking loss: remove ball from frame, verify `-1,-1\n` sentinels flow correctly, verify Kalman predicts and eventually resets
- [ ] Test sudden movement: rapid direction reversal; confirm prediction is wrong but recovers quickly (expected behavior, not a bug)
- [ ] Record a clean demo session with `record_session.py`: ≥2 minutes of varied movement including tracking loss events
- [ ] Test that the recorded session replays cleanly as a backup demo
- [ ] Write `README.md`: hardware setup, `config.py` port setting, flash instructions link, run command
- [ ] Write `docs/calibration_guide.md`: lighting setup, `calibrate_color.py` walkthrough, where to store thresholds
- [ ] Final GitHub commit: tag `v1.0-demo`

#### Acceptance Criteria
- Full system runs live at ≥30 FPS (minimum 15 FPS acceptable)
- Demo runs stably for ≥5 minutes without crash or serial error
- Prediction visibly anticipates the ball's trajectory during smooth motion
- Tracking loss and recovery work correctly within `TRACKING_LOSS_MAX` frames
- Backup replay session confirmed functional on a fresh terminal

---

### Phase 7 — MATH 470 Paper
**Duration:** Parallel with Phases 5–6; finalized after demo
**Due:** Mid-May

#### Key Requirement
`docs/math_derivations.md` must be kept up to date throughout all phases. Writing the paper is then an editorial task, not a research task. Do not leave derivations for the last week.

#### Paper Outline
1. **Introduction** — motivation, system overview, contribution
2. **Image Processing as Linear Algebra** — frames as matrices, colorspace as linear transformation
3. **SVD-Based Least Squares** — Vandermonde construction, SVD derivation (using $\mathbf{A}, \mathbf{U}, \mathbf{\Sigma}, \mathbf{W}$ naming), pseudoinverse, polynomial fit, residual variance
4. **Kalman Filter Theory** — state space model, predict and update equations, covariance propagation
5. **SVD–Kalman Integration** — initialization from polynomial derivatives, adaptive process noise rule
6. **Confidence Intervals** — covariance ellipse derivation, chi-squared scaling, cone construction
7. **Results** — FPS measured, prediction error vs. horizon (from replay sessions), residual variance under smooth vs. sudden motion
8. **Discussion** — limitations, sudden-motion degradation, future work (e.g., extended Kalman filter for nonlinear motion)

#### Tasks
- [ ] After each phase, add the corresponding derivation to `docs/math_derivations.md`
- [ ] After Phase 6, run `replay_session.py` and collect quantitative metrics: mean prediction error per horizon step, mean FPS, residual variance samples
- [ ] Draft `MATH-470_Paper.tex` using `docs/math_derivations.md` as source
- [ ] Submit by mid-May deadline

---

## 7. Technical Specifications

### Firmware (`firmware/`)
| Parameter | Value |
|-----------|-------|
| Runtime | OpenMV (MicroPython) on Portenta H7 |
| Camera resolution | QVGA 320×240 |
| Colorspace | LAB |
| Detection method | `find_blobs()` with LAB threshold from `color_config.py` |
| Blob selection | Largest blob above `MIN_BLOB_AREA` |
| Serial baud | 115200 |
| Serial output (tracking) | `"nx,ny\n"` — two floats in `[0,1]`, 4 decimal places |
| Serial output (no detection) | `"-1,-1\n"` |
| Target frame rate | 30 FPS |
| Development tool | OpenMV IDE — open files from `firmware/` via `File → Open`; use Run button to execute without flashing |
| Deploy to flash | OpenMV IDE `Tools → Save open script to OpenMV Cam`, or copy files to board USB drive root |

### PC Application (`pc_app/`)
| Parameter | Value |
|-----------|-------|
| Language | Python 3.10+ |
| Dependencies | `numpy`, `pyserial`, `opencv-python`, `scipy` |
| Serial baud | 115200 |
| Coordinate space | Display pixels throughout |
| Position buffer size `N` | `BUFFER_SIZE = 20` |
| Prediction horizon `k` | `PREDICTION_HORIZON = 5` |
| SVD polynomial degree `d` | `SVD_POLY_DEGREE = 2` |
| Kalman state dimension | 6 |
| Kalman observation dimension | 2 |
| Confidence level | `CONFIDENCE_LEVEL = 0.95` |
| `scipy` usage | `scipy.stats.chi2.ppf` for ellipse scaling |

### `config.py` — Complete Parameter Reference
```python
# ── Serial ────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/tty.usbmodemXXXX"  # macOS; Windows: "COMX"
SERIAL_BAUD = 115200                    # Must match firmware

# ── Display ───────────────────────────────────────────────────────
DISPLAY_WIDTH  = 960
DISPLAY_HEIGHT = 720

# ── Position Buffer ───────────────────────────────────────────────
BUFFER_SIZE = 20          # N: rolling history length (frames)

# ── SVD Fitter ────────────────────────────────────────────────────
SVD_POLY_DEGREE       = 2      # d: polynomial degree for x(t), y(t)
SVD_REFIT_INTERVAL    = 10     # M: refit every M frames
SVD_SINGULAR_THRESHOLD = 1e-6  # zero singular values below this

# ── Kalman Filter ─────────────────────────────────────────────────
KALMAN_PROCESS_NOISE     = 1e-2   # base Q diagonal scale factor
KALMAN_MEAS_NOISE_DEFAULT = 1e1  # R scale when SVD unavailable (pixels²)
KALMAN_INIT_P            = 1e4   # initial P diagonal (high uncertainty)
KALMAN_Q_REF             = 1.0   # σ²_ref for adaptive Q scaling
KALMAN_Q_MIN             = 0.1   # minimum Q scale multiplier
KALMAN_Q_MAX             = 10.0  # maximum Q scale multiplier

# ── Prediction ────────────────────────────────────────────────────
PREDICTION_HORIZON  = 5   # k: future steps predicted every frame
TRACKING_LOSS_MAX   = 5   # max consecutive lost frames before reset
                          # NOTE: these two parameters are independent;
                          # their default values happen to be equal.
CONFIDENCE_LEVEL    = 0.95

# ── Visualization ─────────────────────────────────────────────────
TRAIL_LENGTH = 15         # rendered trail length (≤ BUFFER_SIZE)
SHOW_TRAIL   = True
```

---

## 8. Mathematical Framework

### 8.1 Why SVD for Least Squares?

The Vandermonde matrix $\mathbf{A}$ is often ill-conditioned, particularly with small $N$ or high polynomial degree. Solving via normal equations $(\mathbf{A}^T\mathbf{A})\mathbf{c} = \mathbf{A}^T\mathbf{p}$ squares the condition number, amplifying numerical errors in camera-noisy data. SVD computes the pseudoinverse directly and allows near-zero singular values to be zeroed (truncated SVD), providing implicit regularization.

### 8.2 Why Kalman for Real-Time Tracking?

SVD is a batch method: it re-fits all $N$ past observations equally every time it runs. It cannot incorporate a new measurement in $O(1)$ time. The Kalman filter is a recursive Bayesian estimator that processes one observation per frame, weights recent observations appropriately through its gain mechanism, and propagates uncertainty explicitly via the covariance matrix $\mathbf{P}$ — which also yields the confidence ellipses at no extra cost.

### 8.3 Why Both?

| Capability | SVD Alone | Kalman Alone | Hybrid |
|---|---|---|---|
| Informed initialization | ✓ | ✗ (cold start) | ✓ |
| O(1) per-frame update | ✗ | ✓ | ✓ |
| Principled noise model | Residuals only | ✓ (covariance) | ✓ |
| Adaptive to motion changes | ✗ | ✓ (with Q tuning) | ✓ |
| Confidence intervals | Approximate | ✓ | ✓ |
| Mathematical depth for paper | ✓✓ | ✓✓ | ✓✓✓ |

SVD initializes the Kalman state from a polynomial fit rather than a cold guess, cuts convergence time from ~20 frames to ~5, and provides the measurement noise matrix $\mathbf{R}$ from residuals. The Kalman filter handles all real-time estimation and uncertainty propagation.

---

## 9. Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Vision Shield driver issues in OpenMV | Medium | High | Phase 0 tests this on day 1; OpenMV has official H7 support |
| 30 FPS not achievable at QVGA | Low | Medium | Profile in Phase 1; LAB blob detection is fast; 15 FPS is accepted minimum |
| Color detection unreliable under demo lighting | Medium | High | `firmware/calibrate.py` run in Phase 0 AND again in Phase 6 at the demo location |
| Kalman diverges on sudden movements | Low | Low | Accepted limitation; adaptive Q provides partial recovery; document in paper |
| Serial latency causes frame jitter on PC | Low | Medium | Threaded reader with monotonic timestamps decouples serial rate from render rate |
| Hardware failure during demo | Low | High | Recorded replay session from Phase 6 is the backup — test it before demo day |
| `NaN` in buffer causes SVD/Kalman crash | Medium | High | `get_valid_matrix()` filters NaN before SVD; guard in `svd_fitter.fit()` |
| Paper not finished due to time pressure | Medium | High | `docs/math_derivations.md` updated after every phase; paper is editorial work only at the end |

---

## 10. Timeline

| Week | Phases | Goal |
|------|--------|------|
| **Week 1** | 0, 1 | Hardware confirmed; green blob streaming over serial at ≥30 FPS |
| **Week 2** | 2, 3 | Serial reader, buffer, record/replay tools, SVD fitter — all unit tested |
| **Week 3** | 4 | Kalman filter and predictor orchestrator complete; tested in replay mode |
| **Week 4** | 5, 6 | Visualizer complete; full system live with hardware; `v1.0-demo` tagged |
| **Week 5–6** | 7 | Paper drafted and submitted |

> **Schedule buffer rule:** If Phase 0 hardware bring-up extends past day 3 of Week 1, cut Phase 5 scope to plain background mode only (drop live overlay). The overlay is a visualization enhancement; the prediction system is the deliverable.

> **Demo fallback rule:** If hardware is unavailable on demo day, `replay_session.py` with a pre-recorded session is a complete and acceptable demo. Record this session at the end of Phase 6.

---

## 11. Milestones & Deliverables

| # | Milestone | Deliverable | Target |
|---|-----------|-------------|--------|
| M0 | Hardware verified | OpenMV shows camera feed; `firmware/calibrate.py` streams LAB values; `firmware/main.py` prints centroids to serial console via Run button | Day 2–3 of Week 1 |
| M1 | Serial stream confirmed | PC terminal receives `nx,ny\n` at ≥30 FPS; sentinel works; `firmware/README.md` documents the full workflow | End of Week 1 |
| M2 | Offline infrastructure complete | Buffer, SVD fitter, record/replay tools — all unit tests pass | End of Week 2 |
| M3 | Prediction core complete | Kalman + predictor pass unit tests; full pipeline verified in replay mode | End of Week 3 |
| M4 | **Demo ready** | Live demo stable ≥5 min; backup replay recorded and tested; `v1.0-demo` tagged | **End of Week 4** |
| M5 | Paper submitted | `MATH-470_Paper.tex` finalized and submitted | Mid-May |

---

*This is a living document. Update when decisions change, but the coordinate conventions in Section 5, the OpenMV workflow in Section 4, and the parameter names in Section 7 are fixed — do not rename them mid-project.*

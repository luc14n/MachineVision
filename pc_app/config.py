"""
PC application configuration defaults.

This module centralizes tunable parameters used by the serial reader,
trajectory fitting, Kalman prediction, and visualization layers.

Do not hardcode machine-specific values in this file.
Use `pc_app/config_local.py` for local overrides when needed.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Serial / firmware interface
# ---------------------------------------------------------------------------

# Port can be overridden at runtime (CLI arg or env) or in config_local.py.
SERIAL_PORT: str | None = None

# Must match firmware UART/USB serial baud rate.
SERIAL_BAUD: int = 115200

# Seconds to wait for serial reads before returning control to app loop.
SERIAL_TIMEOUT_S: float = 0.02

# Expected line format from firmware: "nx,ny\n"
SERIAL_FIELD_DELIMITER: str = ","

# Sentinel emitted by firmware when no blob is detected.
NO_DETECTION_SENTINEL: tuple[int, int] = (-1, -1)

# Compatibility alias used across plan/tests/docs.
POSITION_SENTINEL: tuple[int, int] = NO_DETECTION_SENTINEL

# ---------------------------------------------------------------------------
# Frame / coordinate defaults
# ---------------------------------------------------------------------------

# OpenMV QVGA default used by this project.
FRAME_WIDTH: int = 320
FRAME_HEIGHT: int = 240

# Safety clamp for normalized coordinates if conversion noise occurs.
NORM_MIN: float = 0.0
NORM_MAX: float = 1.0

# ---------------------------------------------------------------------------
# Position buffer / session recording
# ---------------------------------------------------------------------------

# Rolling history retained in memory for fit + render.
POSITION_BUFFER_SIZE: int = 180  # ~6 seconds at 30 FPS

# Frames with no detection retained as sentinel points in raw stream.
KEEP_SENTINEL_IN_RAW_STREAM: bool = True

# Optional path where session CSV is saved when recording is enabled.
# None means "disabled unless provided by CLI/UI action".
SESSION_OUTPUT_PATH: str | None = None

# ---------------------------------------------------------------------------
# SVD polynomial least-squares fitter
# ---------------------------------------------------------------------------

# Polynomial degree for x(t), y(t). Parabolic model => degree 2.
SVD_POLY_DEGREE: int = 2

# Minimum valid points required to fit reliably.
# For degree d, need at least d + 1 mathematically; use d + 2 for robustness.
SVD_MIN_POINTS: int = SVD_POLY_DEGREE + 2

# Time horizon (seconds) for fit window considered in rolling solve.
SVD_WINDOW_S: float = 0.8

# Regularization epsilon used in pseudo-inverse stability checks.
SVD_EPSILON: float = 1e-9

# Reject fit if condition number exceeds this threshold.
SVD_MAX_CONDITION_NUMBER: float = 1e8

# ---------------------------------------------------------------------------
# Kalman filter / predictor orchestration
# State: [x, y, vx, vy, ax, ay]^T  (pixel space)
# ---------------------------------------------------------------------------

# Nominal update interval (seconds). Use measured dt if available.
DT_DEFAULT_S: float = 1.0 / 30.0

# Measurement noise standard deviation in pixels.
MEASUREMENT_STD_PX: float = 3.0

# Base process noise scalar (adapted at runtime by predictor logic).
PROCESS_NOISE_BASE: float = 1e-2

# Adaptive process-noise multiplier bounds.
Q_SCALE_MIN: float = 0.5
Q_SCALE_MAX: float = 8.0

# How long we tolerate missing measurements before reset/re-init behavior.
TRACKING_LOSS_MAX: int = 8

# Require this many consecutive detections to initialize from SVD.
INIT_DETECTION_MIN: int = 6

# Max future horizon for lookahead prediction.
PREDICT_AHEAD_MAX_S: float = 1.0

# ---------------------------------------------------------------------------
# Confidence ellipse rendering / export
# ---------------------------------------------------------------------------

# Quantile scale for 2D covariance ellipse (approx. 95% ~= 2.4477).
ELLIPSE_SIGMA_SCALE: float = 2.4477

# Guard against near-singular covariance for rendering.
ELLIPSE_EIGEN_FLOOR: float = 1e-9

# ---------------------------------------------------------------------------
# Visualization defaults (for upcoming phase integration)
# ---------------------------------------------------------------------------

WINDOW_TITLE: str = "Real-Time Predictive Ball Tracking"
DISPLAY_FPS_TARGET: int = 60
TRAIL_LENGTH: int = 80
SHOW_CONFIDENCE_ELLIPSE: bool = True
SHOW_SVD_CURVE: bool = True
SHOW_KALMAN_PREDICTION: bool = True

# ---------------------------------------------------------------------------
# Debug / logging
# ---------------------------------------------------------------------------

LOG_LEVEL: str = "INFO"  # DEBUG, INFO, WARNING, ERROR
PRINT_SERIAL_PARSE_ERRORS: bool = False
PRINT_TIMING_WARNINGS: bool = False

# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------


def validate_config() -> None:
    """Raise ValueError if required parameters are invalid."""
    if SERIAL_BAUD <= 0:
        raise ValueError("SERIAL_BAUD must be > 0")
    if FRAME_WIDTH <= 0 or FRAME_HEIGHT <= 0:
        raise ValueError("FRAME_WIDTH and FRAME_HEIGHT must be > 0")
    if POSITION_BUFFER_SIZE < 8:
        raise ValueError("POSITION_BUFFER_SIZE must be >= 8")
    if SVD_POLY_DEGREE < 1:
        raise ValueError("SVD_POLY_DEGREE must be >= 1")
    if SVD_MIN_POINTS < SVD_POLY_DEGREE + 1:
        raise ValueError("SVD_MIN_POINTS must be >= SVD_POLY_DEGREE + 1")
    if DT_DEFAULT_S <= 0:
        raise ValueError("DT_DEFAULT_S must be > 0")
    if TRACKING_LOSS_MAX < 1:
        raise ValueError("TRACKING_LOSS_MAX must be >= 1")
    if PREDICT_AHEAD_MAX_S <= 0:
        raise ValueError("PREDICT_AHEAD_MAX_S must be > 0")
    if not (0 < Q_SCALE_MIN <= Q_SCALE_MAX):
        raise ValueError("Q scale bounds invalid")
    if ELLIPSE_SIGMA_SCALE <= 0:
        raise ValueError("ELLIPSE_SIGMA_SCALE must be > 0")


# Validate on import so config errors fail fast in development.
validate_config()

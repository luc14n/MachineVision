# firmware/main.py
# OpenMV Portenta H7 ball tracker -> serial normalized coordinates stream
#
# Output format (one line per frame):
#   "<nx>,<ny>\n"
# where nx, ny are normalized pixel coordinates in [0, 1], rounded to 4 dp.
#
# Sentinel when no ball is detected:
#   "-1,-1\n"

import time

import sensor
from pyb import USB_VCP

try:
    import color_config
except ImportError:
    # Fallback defaults if color_config.py is missing during quick bring-up.
    # Recommended: define these in color_config.py and keep this as backup.
    class color_config:
        # LAB threshold tuple format:
        # (L_min, L_max, A_min, A_max, B_min, B_max)
        LAB_THRESHOLD = (25, 90, -80, -10, 0, 80)

        # Minimum blob area/pixels to reject tiny noise blobs.
        PIXELS_THRESHOLD = 120
        AREA_THRESHOLD = 120

        # Optional ROI (x, y, w, h); set to None for full frame.
        ROI = None

        # OpenMV color tracking merge margin.
        MERGE = True
        MARGIN = 10

        # Optional image settings
        AUTO_GAIN = False
        AUTO_WHITEBAL = False
        AUTO_EXPOSURE = True


# ----------------------------
# Camera / sensor setup
# ----------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=1500)

# Keep color metrics stable for blob tracking.
if hasattr(sensor, "set_auto_gain"):
    sensor.set_auto_gain(getattr(color_config, "AUTO_GAIN", False))
if hasattr(sensor, "set_auto_whitebal"):
    sensor.set_auto_whitebal(getattr(color_config, "AUTO_WHITEBAL", False))
if hasattr(sensor, "set_auto_exposure"):
    sensor.set_auto_exposure(getattr(color_config, "AUTO_EXPOSURE", True))

clock = time.clock()
vcp = USB_VCP()

# Frame geometry for normalization
frame_w = sensor.width()
frame_h = sensor.height()
inv_w = 1.0 / float(frame_w - 1 if frame_w > 1 else 1)
inv_h = 1.0 / float(frame_h - 1 if frame_h > 1 else 1)

THRESHOLDS = [getattr(color_config, "LAB_THRESHOLD", (25, 90, -80, -10, 0, 80))]
PIXELS_THRESHOLD = int(getattr(color_config, "PIXELS_THRESHOLD", 120))
AREA_THRESHOLD = int(getattr(color_config, "AREA_THRESHOLD", 120))
ROI = getattr(color_config, "ROI", None)
MERGE = bool(getattr(color_config, "MERGE", True))
MARGIN = int(getattr(color_config, "MARGIN", 10))

# Debug cadence (prints FPS periodically to serial, comment out if undesired)
DEBUG_EVERY_N_FRAMES = 0  # set e.g. 30 to emit "# fps=..."
frame_count = 0


def _best_blob(blobs):
    # Pick the blob with largest pixel count for robust center estimate.
    if not blobs:
        return None
    best = blobs[0]
    best_pixels = best.pixels()
    for b in blobs[1:]:
        p = b.pixels()
        if p > best_pixels:
            best = b
            best_pixels = p
    return best


def _write_line(text):
    # USB_VCP.write accepts bytes
    try:
        vcp.write(text.encode("utf-8"))
    except Exception:
        # Ignore transient USB errors and continue real-time loop.
        pass


while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        THRESHOLDS,
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        roi=ROI,
        merge=MERGE,
        margin=MARGIN,
    )

    blob = _best_blob(blobs)

    if blob is None:
        _write_line("-1,-1\n")
    else:
        cx = blob.cx()
        cy = blob.cy()

        # Normalize to [0, 1] in image coordinates:
        # x: left->right, y: top->bottom
        nx = cx * inv_w
        ny = cy * inv_h

        # Clamp against any edge-case rounding drift
        if nx < 0.0:
            nx = 0.0
        elif nx > 1.0:
            nx = 1.0

        if ny < 0.0:
            ny = 0.0
        elif ny > 1.0:
            ny = 1.0

        _write_line("{:.4f},{:.4f}\n".format(nx, ny))

    frame_count += 1
    if DEBUG_EVERY_N_FRAMES and (frame_count % DEBUG_EVERY_N_FRAMES == 0):
        _write_line("# fps={:.2f}\n".format(clock.fps()))

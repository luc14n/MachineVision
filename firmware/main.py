import struct
import time

import sensor
from pyb import USB_VCP

# Prefer auto-generated config (written by calibrate.py) when present.
# Falls back to the repo-tracked defaults in color_config.py.
try:
    import color_config_auto as color_config
except ImportError:
    try:
        import color_config
    except ImportError:

        class color_config:
            # Grayscale threshold: (lo, hi)
            GRAY_THRESHOLD = (0, 70)
            PIXELS_THRESHOLD = 120
            AREA_THRESHOLD = 120
            ROI = None
            MERGE = True
            MARGIN = 10
            AUTO_GAIN = True
            AUTO_EXPOSURE = True


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

if hasattr(sensor, "set_auto_gain"):
    sensor.set_auto_gain(getattr(color_config, "AUTO_GAIN", True))
if hasattr(sensor, "set_auto_exposure"):
    sensor.set_auto_exposure(getattr(color_config, "AUTO_EXPOSURE", True))

clock = time.clock()
vcp = USB_VCP()

frame_w = sensor.width()
frame_h = sensor.height()
inv_w = 1.0 / float(frame_w - 1 if frame_w > 1 else 1)
inv_h = 1.0 / float(frame_h - 1 if frame_h > 1 else 1)

threshold = getattr(color_config, "GRAY_THRESHOLD", (0, 70))
THRESHOLDS = [threshold]
PIXELS_THRESHOLD = int(getattr(color_config, "PIXELS_THRESHOLD", 120))
AREA_THRESHOLD = int(getattr(color_config, "AREA_THRESHOLD", 120))
ROI = getattr(color_config, "ROI", None)
MERGE = bool(getattr(color_config, "MERGE", True))
MARGIN = int(getattr(color_config, "MARGIN", 10))


def _best_blob(blobs):
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


while True:
    clock.tick()
    img = sensor.snapshot()

    # IMPORTANT: only pass roi if it's a tuple/list.
    if ROI is None:
        blobs = img.find_blobs(
            THRESHOLDS,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=MERGE,
            margin=MARGIN,
        )
    else:
        blobs = img.find_blobs(
            THRESHOLDS,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            roi=ROI,
            merge=MERGE,
            margin=MARGIN,
        )

    blob = _best_blob(blobs)

    if blob is not None:
        cx = blob.cx()
        cy = blob.cy()
        # Radius from blob box size; keep visible but bounded.
        r = (blob.w() + blob.h()) // 4
        if r < 4:
            r = 4
        img.draw_circle(cx, cy, r, color=255, thickness=2)
        img.draw_cross(cx, cy, color=255, size=8, thickness=1)

        nx = cx * inv_w
        ny = cy * inv_h

        if nx < 0.0:
            nx = 0.0
        elif nx > 1.0:
            nx = 1.0

        if ny < 0.0:
            ny = 0.0
        elif ny > 1.0:
            ny = 1.0
    else:
        nx, ny = -1.0, -1.0

    img.compress(quality=35)

    # Pack floats and integers into a fast binary struct instead of a string
    # 'f' is float, 'I' is unsigned integer (4 bytes)
    # < indicates little-endian
    header = struct.pack("<ffI", nx, ny, img.size())

    try:
        # Send a magic start sequence so the PC knows a frame is coming
        vcp.write(b"SNAP")
        vcp.write(header)
        vcp.write(img)
    except Exception:
        pass

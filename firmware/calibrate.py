"""
calibrate.py — OpenMV LAB threshold calibration helper.

Purpose:
- Stream observed LAB min/max values for a center ROI in real time.
- Help you derive a robust LAB threshold tuple for `color_config.py`.

How to use:
1) Open this file in OpenMV IDE and click Run.
2) Hold/move the target ball through the expected tracking area.
3) Watch the serial output for expanding LAB ranges.
4) Stop after ~10–20 seconds and copy values into `color_config.py`,
   adding a small safety margin (commonly 3–8 units per bound).

Output format:
LAB_MINMAX:l_min,l_max,a_min,a_max,b_min,b_max
"""

import time

import sensor
from pyb import LED, UART

# -----------------------------
# Tunables
# -----------------------------
UART_PORT = 3
UART_BAUD = 115200

# Center ROI as fraction of frame size (w_frac, h_frac).
# Keep modest so samples are mostly target pixels while being stable.
ROI_W_FRAC = 0.30
ROI_H_FRAC = 0.30

# Print interval to avoid flooding terminal.
PRINT_EVERY_N_FRAMES = 3

# Optional quality gates
MIN_PIXELS_FOR_UPDATE = 40  # ignore tiny/noisy detections
ENABLE_COLOR_HINT = False  # set True if you want to restrict by rough hint

# Very loose green-ish LAB hint (only used if ENABLE_COLOR_HINT=True)
# OpenMV threshold format: (l_min, l_max, a_min, a_max, b_min, b_max)
GREEN_HINT = (0, 100, -64, 10, -10, 64)


# -----------------------------
# Helpers
# -----------------------------
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def roi_from_sensor():
    w = sensor.width()
    h = sensor.height()
    rw = int(w * ROI_W_FRAC)
    rh = int(h * ROI_H_FRAC)
    rw = clamp(rw, 8, w)
    rh = clamp(rh, 8, h)
    rx = (w - rw) // 2
    ry = (h - rh) // 2
    return (rx, ry, rw, rh)


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)  # Required for LAB operations
    sensor.set_framesize(sensor.QVGA)  # 320x240, good balance for calibration
    sensor.skip_frames(time=1200)
    # Disable auto adjustments for stable color readings
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
    return time.clock()


def init_uart():
    return UART(UART_PORT, UART_BAUD, timeout_char=1000)


def emit(uart, text):
    # OpenMV UART expects bytes
    uart.write((text + "\n").encode())


# -----------------------------
# Main calibration routine
# -----------------------------
def run():
    led_g = LED(2)  # green LED
    led_r = LED(1)  # red LED

    clock = init_sensor()
    uart = init_uart()
    roi = roi_from_sensor()

    # Running extrema (initialize to opposite bounds)
    l_min, l_max = 255, 0
    a_min, a_max = 255, 0
    b_min, b_max = 255, 0

    frame_idx = 0
    updated_once = False

    emit(uart, "CALIBRATE_START")
    emit(uart, "ROI:{},{},{},{}".format(roi[0], roi[1], roi[2], roi[3]))

    while True:
        clock.tick()
        img = sensor.snapshot()

        # Draw ROI for visual guidance in OpenMV IDE framebuffer.
        img.draw_rectangle(roi, color=(255, 255, 255), thickness=1)

        # Option A: direct LAB stats on ROI
        if not ENABLE_COLOR_HINT:
            stats = img.get_statistics(roi=roi)
            # stats object fields are available as methods on OpenMV
            l_lo, l_hi = stats.l_min(), stats.l_max()
            a_lo, a_hi = stats.a_min(), stats.a_max()
            b_lo, b_hi = stats.b_min(), stats.b_max()

            # crude gate: skip pathological empty/noisy updates
            # (for direct stats we approximate confidence by spread sanity)
            if (l_hi - l_lo) >= 0:
                l_min = min(l_min, l_lo)
                l_max = max(l_max, l_hi)
                a_min = min(a_min, a_lo)
                a_max = max(a_max, a_hi)
                b_min = min(b_min, b_lo)
                b_max = max(b_max, b_hi)
                updated_once = True
                led_g.on()
                led_r.off()
            else:
                led_g.off()
                led_r.on()

        # Option B: constrained by rough color hint blobs
        else:
            blobs = img.find_blobs(
                [GREEN_HINT],
                roi=roi,
                merge=True,
                pixels_threshold=MIN_PIXELS_FOR_UPDATE,
            )
            if blobs:
                # Use the largest blob as calibration sample source
                largest = blobs[0]
                for b in blobs:
                    if b.pixels() > largest.pixels():
                        largest = b

                bx, by, bw, bh = largest.rect()
                stats = img.get_statistics(roi=(bx, by, bw, bh))
                l_lo, l_hi = stats.l_min(), stats.l_max()
                a_lo, a_hi = stats.a_min(), stats.a_max()
                b_lo, b_hi = stats.b_min(), stats.b_max()

                l_min = min(l_min, l_lo)
                l_max = max(l_max, l_hi)
                a_min = min(a_min, a_lo)
                a_max = max(a_max, a_hi)
                b_min = min(b_min, b_lo)
                b_max = max(b_max, b_hi)
                updated_once = True

                img.draw_rectangle(largest.rect(), color=(0, 255, 0), thickness=2)
                led_g.on()
                led_r.off()
            else:
                led_g.off()
                led_r.on()

        frame_idx += 1
        if frame_idx % PRINT_EVERY_N_FRAMES == 0:
            fps = clock.fps()
            if updated_once:
                line = "LAB_MINMAX:{},{},{},{},{},{} FPS:{:.1f}".format(
                    l_min, l_max, a_min, a_max, b_min, b_max, fps
                )
                emit(uart, line)
                print(line)
            else:
                line = "LAB_MINMAX:NA FPS:{:.1f}".format(fps)
                emit(uart, line)
                print(line)


# Entry point
run()

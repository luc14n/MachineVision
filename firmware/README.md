# Firmware — Portenta H7 + Vision Shield

This directory is the **OpenMV IDE project root** for the Portenta H7.
All files here are MicroPython source. Git is the source of truth — never
edit files directly on the board.

---

## Directory Contents

| File | Purpose | Runs on |
|------|---------|---------|
| `main.py` | Camera init, blob detection, serial TX | Board (production) |
| `color_config.py` | LAB color thresholds for the green ball | Board (imported by main.py) |
| `calibrate.py` | Interactive LAB threshold finder | Board (development only) |
| `README.md` | This file | PC |

Only `main.py` and `color_config.py` need to be deployed to the board.
`calibrate.py` and `README.md` stay on the PC.

---

## One-Time Setup — Flash the OpenMV Runtime

The Portenta H7 must have the OpenMV firmware flashed before any of this
works. This only needs to be done once (or after a factory reset).

1. Download and install [OpenMV IDE](https://openmv.io/pages/download)
2. Connect the Portenta H7 via USB
3. In OpenMV IDE, click the **Connect** button (bottom-left)
4. If prompted that the firmware is outdated or missing, click
   **Update Firmware** and follow the on-screen steps
5. Once connected, the frame buffer panel should show a live camera image

---

## Development Loop — Run Without Deploying

During development, use the OpenMV IDE **Run button** to execute scripts
directly from this directory without writing anything to the board's flash.
The board reverts to its deployed state on power cycle.

**Setup (do once per session):**
1. Open OpenMV IDE
2. `File → Open` → navigate to this `firmware/` directory → open `main.py`
3. OpenMV IDE now treats `firmware/` as its working directory — imports
   like `import color_config` will resolve to `firmware/color_config.py`

**Iteration loop:**
1. Edit `main.py` or `color_config.py` in your normal code editor and save
2. Switch to OpenMV IDE — the file on disk is re-read on each Run
3. Click **Run** (green play button, bottom-left)
4. Observe output in the **Serial Terminal** panel (bottom of OpenMV IDE)
5. Repeat from step 1

You do not need to use OpenMV IDE as your editor. Any editor that saves
to disk works. OpenMV IDE is only needed to click Run and view serial output.

---

## Color Calibration — Find LAB Thresholds

Run this whenever lighting conditions change or when setting up for the
first time.

1. In OpenMV IDE, `File → Open` → open `calibrate.py`
2. Click **Run**
3. Point the camera at the green ball — the serial terminal will stream
   the L, A, B min/max values observed in the center region of the frame
4. Move the ball around the expected operating area for ~10 seconds
5. Note the full range of L, A, B values seen
6. Stop the script (click **Stop**)
7. Open `color_config.py` in your code editor
8. Update `LAB_THRESHOLD` with the observed min/max values, adding a small
   margin (~5 units) on each bound for robustness
9. Save `color_config.py` and commit it to Git

Re-run calibration at the demo location under demo lighting before Phase 6.

---

## Deploy to Board — Standalone Run on Power-Up

When you want the board to run `main.py` automatically without the IDE
(e.g., for the live demo), deploy `main.py` and `color_config.py` to the
board's flash filesystem.

### Method A — USB Drive (recommended)

The board mounts as a USB drive when connected.

1. Open the board's drive in Finder (macOS) or Explorer (Windows)
2. Copy `main.py` from this directory to the board drive root
3. Copy `color_config.py` from this directory to the board drive root
4. Eject the drive safely
5. Power-cycle the board — it will now run `main.py` automatically

Do **not** copy `calibrate.py` or `README.md` to the board.

### Method B — OpenMV IDE Save

1. In OpenMV IDE, open `main.py` (`File → Open`)
2. `Tools → Save open script to OpenMV Cam` — saves `main.py` to board root
3. Repeat: open `color_config.py`, then `Tools → Save open script to OpenMV Cam`
4. Power-cycle the board

This is a one-file-at-a-time operation, which is why Method A is faster
when deploying both files.

---

## Verify Standalone Deployment

After deploying and power-cycling:

1. Disconnect and reconnect USB (or use a USB power bank instead of the PC)
2. Open a serial terminal on the PC:
   ```
   python -m serial.tools.miniterm /dev/tty.usbmodemXXXX 115200
   ```
   Replace the port with the actual port shown for your board.
3. Confirm `nx,ny\n` values stream at ~30 FPS when the ball is in frame
4. Confirm `-1,-1\n` streams when the ball is removed

If nothing appears, reconnect to OpenMV IDE and re-check that both files
were saved to the board root (visible in the board's USB drive).

---

## Baud Rate

The serial baud rate is set to **115200** in `main.py` and must match
`SERIAL_BAUD` in `pc_app/config.py`. Do not change one without changing
the other.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| OpenMV IDE can't connect | Board not in OpenMV mode | Double-tap reset button; reflash firmware |
| `ImportError: color_config` | Wrong working directory in IDE | `File → Open` the file from `firmware/` — do not use Recent Files if you moved the repo |
| FPS below 15 | Resolution too high or blob search area too large | Confirm `QVGA` (320×240) is set; reduce `roi` in `find_blobs()` if used |
| Ball not detected | LAB thresholds don't match current lighting | Re-run `calibrate.py` and update `color_config.py` |
| Serial output garbled on PC | Baud rate mismatch | Confirm both `main.py` and `config.py` use 115200 |
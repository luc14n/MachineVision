"""
Color threshold configuration for OpenMV blob detection.

This module stores LAB threshold values used by `main.py` when detecting
the target ball color. Re-run calibration under the current lighting
conditions and update `LAB_THRESHOLD` as needed.
"""

# LAB threshold tuple format for OpenMV:
# (L Min, L Max, A Min, A Max, B Min, B Max)
LAB_THRESHOLD = (30, 85, -70, -10, 5, 55)

# Optional area filter defaults for blob detection in `main.py`.
# These are conservative starting points and can be tuned later.
PIXELS_THRESHOLD = 120
AREA_THRESHOLD = 120
MERGE_BLOBS = True
MARGIN = 10

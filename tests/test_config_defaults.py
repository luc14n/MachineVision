"""
Initial configuration default validation tests.

These tests are intentionally lightweight and defensive so they can run
early in the project lifecycle while `pc_app/config.py` evolves.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path

import pytest


def _project_root() -> Path:
    # tests/ -> project root
    return Path(__file__).resolve().parents[1]


def _load_config_module():
    """
    Load `pc_app.config` in a way that works when the repo root is not already
    on sys.path.
    """
    root = _project_root()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    return importlib.import_module("pc_app.config")


def test_config_module_exists():
    """
    Guardrail test: Phase 0/1 should provide pc_app/config.py.
    """
    config_path = _project_root() / "pc_app" / "config.py"
    assert config_path.exists(), "Expected pc_app/config.py to exist"


def test_expected_core_constants_are_present():
    """
    Ensure the baseline constants used by serial + visualization exist.
    """
    cfg = _load_config_module()

    required = [
        "SERIAL_BAUD",
        "POSITION_SENTINEL",
    ]

    missing = [name for name in required if not hasattr(cfg, name)]
    assert not missing, f"Missing expected config constant(s): {missing}"


def test_serial_baud_is_positive_int():
    cfg = _load_config_module()

    assert isinstance(cfg.SERIAL_BAUD, int), "SERIAL_BAUD must be an int"
    assert cfg.SERIAL_BAUD > 0, "SERIAL_BAUD must be > 0"


def test_serial_baud_is_standard_or_high_speed():
    """
    Not overly strict, but catches accidental invalid values (e.g., 0, 123).
    """
    cfg = _load_config_module()

    common_valid = {
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        460800,
        921600,
    }
    assert cfg.SERIAL_BAUD in common_valid, (
        f"Unexpected SERIAL_BAUD value: {cfg.SERIAL_BAUD}"
    )


def test_position_sentinel_has_two_values():
    cfg = _load_config_module()

    sentinel = cfg.POSITION_SENTINEL
    assert isinstance(sentinel, tuple), (
        "POSITION_SENTINEL should be a tuple for immutability"
    )
    assert len(sentinel) == 2, "POSITION_SENTINEL must contain exactly (x, y)"


def test_position_sentinel_uses_negative_one_pair():
    """
    Per plan convention, missing detections should be encoded as (-1, -1).
    """
    cfg = _load_config_module()

    assert cfg.POSITION_SENTINEL == (-1, -1), (
        "POSITION_SENTINEL should default to (-1, -1) to match "
        "firmware serial streaming convention"
    )


@pytest.mark.parametrize("env_name", ["SERIAL_PORT", "DISPLAY_WIDTH", "DISPLAY_HEIGHT"])
def test_no_required_runtime_env_vars_for_import(env_name):
    """
    Importing config should not require environment variables at import time.
    This keeps unit tests and tooling stable across machines.
    """
    original = os.environ.pop(env_name, None)
    try:
        # Force a fresh import attempt
        if "pc_app.config" in sys.modules:
            del sys.modules["pc_app.config"]
        _load_config_module()
    finally:
        if original is not None:
            os.environ[env_name] = original

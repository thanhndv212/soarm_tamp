"""TAMP must consume calibration checks without exposing calibration actions."""

from __future__ import annotations

import threading

from soarm_sdk.calibration.frame import RobotCalibration
from soarm_tamp.dashboard.panels.watchdog import watchdog_violations


class _Context:
    calibration = None
    lock = threading.Lock()


def test_watchdog_blocks_without_a_calibration():
    assert watchdog_violations(_Context()) == ["no calibration loaded"]


def test_watchdog_blocks_calibration_without_explicit_tolerances():
    ctx = _Context()
    ctx.calibration = RobotCalibration(joints=[])

    violations = watchdog_violations(ctx)

    assert violations == ["calibration has no explicit acceptance tolerances"]

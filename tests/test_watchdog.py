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


def test_play_drives_the_ghost_not_the_live_mirror():
    """Was: ManifestPlayer and the live-poll loop both drove fk_update, so
    a preview fought the real arm's position for the same mesh."""
    import inspect

    from soarm_tamp.dashboard.panels import _common

    src = inspect.getsource(_common.PlanControls.play)
    assert "self.fk_update_ghost" in src
    assert "ManifestPlayer(\n            self.run_dir,\n            self.fk_update," not in src


def test_a_successful_plan_draws_a_persistent_trail():
    import inspect

    from soarm_tamp.dashboard.panels import _common

    src = inspect.getsource(_common.PlanControls.plan)
    assert "_draw_trail" in src
    assert "rc == 0" in src


def test_the_trail_uses_the_gripper_frame_not_a_random_link():
    import inspect

    from soarm_tamp.dashboard.panels import _common

    src = inspect.getsource(_common.PlanControls._draw_trail)
    assert "gripper_frame_link" in src
    assert "/planned_path" in src


def test_the_panel_builders_accept_and_forward_fk_update_ghost():
    import inspect

    from soarm_tamp.dashboard.panels import pickplace, tcp

    for mod, build_fn, inner_fn in (
        (tcp, "build_tcp_panel", "_build_tcp"),
        (pickplace, "build_pickplace_panel", "_build_pickplace"),
    ):
        build_src = inspect.getsource(getattr(mod, build_fn))
        inner_src = inspect.getsource(getattr(mod, inner_fn))
        assert "fk_update_ghost" in build_src, build_fn
        assert "fk_update_ghost=fk_update_ghost" in inner_src, inner_fn

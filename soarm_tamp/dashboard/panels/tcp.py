"""TCP Plan tab: send the TCP somewhere and prove the pipeline end to end.

The smallest thing the stack can do — one Cartesian goal, solved to a
configuration and planned through the library's own graph — which makes it
the right thing to press first. If this tab works, the container, the
manifest handoff, the calibration and the servos are all wired correctly,
and anything the pick-and-place then fails at is the task, not the plumbing.
"""

from __future__ import annotations

import random
from pathlib import Path
from typing import Any

from soarm_sdk.dashboard.app import Panel

from ...geometry import TOPDOWN_RADIUS_MAX_M, TOPDOWN_RADIUS_MIN_M
from ._common import Console, PlanControls

__all__ = ["build_tcp_panel"]

RUN_DIR = "runs/dash_tcp"

# The verified top-down workspace, from the 600k-sample sweep in
# studies/reachability.py. Sampling outside it produces IK failures that
# say nothing about the pipeline, which is the opposite of this tab's job.
Z_MIN_M, Z_MAX_M = 0.015, 0.065
# Coverage inside the annulus is 17 of 20: the failures cluster at
# r ~ 0.11-0.12 m where the arm folds in tight. Keeping the sampler off
# that edge makes a red result mean something.
SAFE_RADIUS_MIN_M = 0.14


def build_tcp_panel(fk_update=None) -> Panel:
    def _build(server: Any, ctx: Any) -> None:
        _build_tcp(server, ctx, fk_update)

    return Panel("TCP Plan", _build)


def _build_tcp(server: Any, ctx: Any, fk_update) -> None:
    server.gui.add_markdown(
        "## TCP pose\n"
        "One Cartesian goal, planned in the container and streamed here. "
        "The cheapest end-to-end check of the whole pipeline."
    )

    with server.gui.add_folder("Goal"):
        x_h = server.gui.add_number("x (m)", initial_value=0.22, step=0.005)
        y_h = server.gui.add_number("y (m)", initial_value=0.0, step=0.005)
        z_h = server.gui.add_number("z (m)", initial_value=0.05, step=0.005)
        roll_h = server.gui.add_number("roll (deg)", initial_value=180.0, step=5.0)
        random_btn = server.gui.add_button("🎲 Random reachable pose")
        plan_btn = server.gui.add_button("Plan TCP pose", color="green")

    with server.gui.add_folder("Run"):
        view_btn = server.gui.add_button("Replay in viewer (:8000)", color="blue")
        stop_view_btn = server.gui.add_button("Stop viewer")
        dry_h = server.gui.add_checkbox("Dry run (no hardware)", initial_value=True)
        exec_btn = server.gui.add_button("Execute", color="orange")

    log_md = server.gui.add_markdown("")
    console = Console(log_md)
    console.clear()
    ctrl = PlanControls(
        server, ctx, console,
        run_dir=Path(__file__).resolve().parents[3] / RUN_DIR,
        fk_update=fk_update,
    )

    @random_btn.on_click
    def _random(_: Any) -> None:
        r = random.uniform(SAFE_RADIUS_MIN_M, TOPDOWN_RADIUS_MAX_M)
        a = random.uniform(-0.9, 0.9)  # radians of yaw, kept off the extremes
        x_h.value = round(r * (1 - a * a / 2), 4)
        y_h.value = round(r * a, 4)
        z_h.value = round(random.uniform(Z_MIN_M, Z_MAX_M), 4)
        console.say(
            f"sampled x={x_h.value} y={y_h.value} z={z_h.value} "
            f"(r={r:.3f} m, annulus {TOPDOWN_RADIUS_MIN_M}-{TOPDOWN_RADIUS_MAX_M})"
        )

    @plan_btn.on_click
    def _plan(_: Any) -> None:
        ctrl.plan(
            [
                "tcp",
                "--out", RUN_DIR,
                "--xyz", str(x_h.value), str(y_h.value), str(z_h.value),
                "--rpy", str(roll_h.value), "0", "0",
                "--viewer", "none",
            ],
            label=f"planning TCP ({x_h.value}, {y_h.value}, {z_h.value})",
        )

    @view_btn.on_click
    def _view(_: Any) -> None:
        ctrl.replay()

    @stop_view_btn.on_click
    def _stop_view(_: Any) -> None:
        ctrl.stop_replay()

    @exec_btn.on_click
    def _exec(_: Any) -> None:
        ctrl.execute(
            dry_run=bool(dry_h.value),
            port=ctx.device_h.value or None,
            rate_hz=30.0,
            max_step=0.02,
            force=False,
        )

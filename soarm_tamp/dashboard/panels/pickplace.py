"""Pick & Place tab: plan the cube task, watch it, then run it.

Everything the TCP tab does, plus the parts that only a grasp has: a jaw
that opens and closes, an object that has to still be there, and the option
to plan from where the arm actually is rather than from the model's zero.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from soarm_sdk.dashboard.app import Panel

from ...geometry import CUBE_SIZE_M, PICK_XY, PLACE_XY
from ._common import Console, PlanControls

__all__ = ["build_pickplace_panel"]

RUN_DIR = "runs/dash_cube"
START_FILE = "runs/dash_start.json"


def build_pickplace_panel(fk_update=None, fk_update_ghost=None) -> Panel:
    def _build(server: Any, ctx: Any) -> None:
        _build_pickplace(server, ctx, fk_update, fk_update_ghost)

    return Panel("Pick & Place", _build)


def _build_pickplace(server: Any, ctx: Any, fk_update, fk_update_ghost=None) -> None:
    root = Path(__file__).resolve().parents[3]

    server.gui.add_markdown(
        f"## Cube pick and place\n"
        f"A **{CUBE_SIZE_M * 1000:.0f} mm** cube from "
        f"A = {tuple(PICK_XY)} to B = {tuple(PLACE_XY)} m.\n\n"
        f"Put the cube at A before running on hardware — the plan assumes "
        f"it is there and nothing checks."
    )

    with server.gui.add_folder("Plan"):
        from_here_h = server.gui.add_checkbox(
            "Start from the arm's current pose", initial_value=True
        )
        capture_btn = server.gui.add_button("Capture pose now")
        plan_btn = server.gui.add_button("Plan pick & place", color="green")

    with server.gui.add_folder("Play here"):
        speed_h = server.gui.add_slider(
            "Speed", min=0.25, max=4.0, step=0.25, initial_value=1.0
        )
        loop_h = server.gui.add_checkbox("Loop", initial_value=False)
        play_btn = server.gui.add_button("▶ Play in this view", color="blue")
        stop_play_btn = server.gui.add_button("■ Stop")

    with server.gui.add_folder("Run"):
        dry_h = server.gui.add_checkbox("Dry run (no hardware)", initial_value=True)
        force_h = server.gui.add_checkbox(
            "Force (skip calibration checks)", initial_value=False
        )
        server.gui.add_markdown(
            "⚠️ *Force* skips `check_ready()` — the same override "
            "`execute.run(..., force=True)` documents as \"at your own "
            "risk\". Use only for a specific, understood gap (e.g. one "
            "joint's zero not yet pose-verified); it does not fix the "
            "calibration, it just stops the refusal."
        )
        pace_h = server.gui.add_checkbox("Pace dry run at true speed", initial_value=False)
        speed_scale_h = server.gui.add_slider(
            "Speed scale (feedforward)", min=0.0, max=2.0, step=0.1,
            initial_value=1.5,
        )
        server.gui.add_markdown(
            "Multiplies the plan's own per-step velocity when commanding "
            "each servo. 1.5 (the `execute.py` default) asks every joint "
            "to run 50% ahead of the plan's own pace; on a fast segment "
            "that can ask more than a joint's real margin allows — seen on "
            "this arm as `wrist_flex` falling behind and the run aborting "
            "(`ABORTING: ... off the planned path`). Try 1.0 first (the "
            "plan's own pace, no overdrive) if that happens."
        )
        exec_btn = server.gui.add_button("Execute", color="orange")

    with server.gui.add_folder("Full scene viewer (separate port)"):
        server.gui.add_markdown(
            "The container-side viewer, for the planning scene — cube, table, "
            "grasp frames. Opens its own viser on **:8000**."
        )
        view_btn = server.gui.add_button("Open scene viewer")
        follow_btn = server.gui.add_button("Follow live there")
        stop_view_btn = server.gui.add_button("Stop scene viewer")

    log_md = server.gui.add_markdown("")
    console = Console(log_md)
    console.clear()
    ctrl = PlanControls(
        server, ctx, console, run_dir=root / RUN_DIR, fk_update=fk_update,
        fk_update_ghost=fk_update_ghost,
    )

    @capture_btn.on_click
    def _do_capture(_: Any) -> None:
        ctrl.capture_start(root / START_FILE)

    @plan_btn.on_click
    def _plan(_: Any) -> None:
        args = ["plan", "--out", RUN_DIR, "--viewer", "none"]
        label = "planning pick & place"
        if from_here_h.value:
            if not ctrl.capture_start(root / START_FILE):
                return
            args += ["--start", START_FILE]
            label += " from the measured pose"
        ctrl.plan(args, label=label)

    @play_btn.on_click
    def _play(_: Any) -> None:
        ctrl.play(speed=float(speed_h.value), loop=bool(loop_h.value))

    @stop_play_btn.on_click
    def _stop_play(_: Any) -> None:
        ctrl.stop_play()

    @view_btn.on_click
    def _view(_: Any) -> None:
        ctrl.replay()

    @follow_btn.on_click
    def _follow(_: Any) -> None:
        ctrl.replay(follow=True)

    @stop_view_btn.on_click
    def _stop_view(_: Any) -> None:
        ctrl.stop_replay()

    @exec_btn.on_click
    def _exec(_: Any) -> None:
        if force_h.value:
            console.say(
                "⚠️ FORCED — calibration acceptance checks are being skipped "
                "for this run"
            )
        ctrl.execute(
            dry_run=bool(dry_h.value),
            port=ctx.device_h.value or None,
            rate_hz=30.0,
            max_step=0.02,
            force=bool(force_h.value),
            pace=bool(pace_h.value),
            speed_scale=float(speed_scale_h.value),
        )

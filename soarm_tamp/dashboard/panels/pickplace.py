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


def build_pickplace_panel(fk_update=None) -> Panel:
    def _build(server: Any, ctx: Any) -> None:
        _build_pickplace(server, ctx, fk_update)

    return Panel("Pick & Place", _build)


def _build_pickplace(server: Any, ctx: Any, fk_update) -> None:
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

    with server.gui.add_folder("Run"):
        view_btn = server.gui.add_button("Replay in viewer (:8000)", color="blue")
        follow_btn = server.gui.add_button("Follow live in viewer")
        stop_view_btn = server.gui.add_button("Stop viewer")
        dry_h = server.gui.add_checkbox("Dry run (no hardware)", initial_value=True)
        pace_h = server.gui.add_checkbox("Pace dry run at true speed", initial_value=False)
        exec_btn = server.gui.add_button("Execute", color="orange")

    log_md = server.gui.add_markdown("")
    console = Console(log_md)
    console.clear()
    ctrl = PlanControls(
        server, ctx, console, run_dir=root / RUN_DIR, fk_update=fk_update
    )

    def _capture() -> bool:
        """Read the arm's pose to a file the container-side planner can use.

        Needs the serial port, which the live mirror is holding, so the
        mirror stands down for the read and comes straight back.
        """
        import json
        import math

        from ...conventions import URDF_LIMITS
        from ...read_pose import capture

        port = ctx.device_h.value
        if not port:
            console.say("no serial port set in the sidebar")
            return False
        was_polling = bool(getattr(ctx, "polling", False))
        try:
            if was_polling:
                ctx.stop_polling()
            pose = capture(port, 0.3)
            out = root / START_FILE
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(pose, indent=1) + "\n")

            # A limp arm rests where the URDF says it cannot be, and the
            # planner refuses such a pose rather than clamping it. Say so
            # here, where it is fixable, instead of in the planner's output.
            bad = [
                f"{n} {math.degrees(v):+.1f} deg"
                for n, v in zip(pose["joint_names"], pose["q"])
                if not URDF_LIMITS[n][0] <= v <= URDF_LIMITS[n][1]
            ]
            console.say(f"captured pose -> {out.name}")
            if bad:
                console.say("OUT OF BOUNDS: " + ", ".join(bad))
                console.say("lift the arm into range; planning will refuse this")
                return False
            return True
        except Exception as exc:
            console.say(f"capture failed: {exc}")
            return False
        finally:
            if was_polling:
                ctx.start_polling()

    @capture_btn.on_click
    def _do_capture(_: Any) -> None:
        _capture()

    @plan_btn.on_click
    def _plan(_: Any) -> None:
        args = ["plan", "--out", RUN_DIR, "--viewer", "none"]
        label = "planning pick & place"
        if from_here_h.value:
            if not _capture():
                return
            args += ["--start", START_FILE]
            label += " from the measured pose"
        ctrl.plan(args, label=label)

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
        ctrl.execute(
            dry_run=bool(dry_h.value),
            port=ctx.device_h.value or None,
            rate_hz=30.0,
            max_step=0.02,
            force=False,
            pace=bool(pace_h.value),
        )

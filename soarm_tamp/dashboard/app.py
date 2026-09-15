"""Assemble the soarm_tamp dashboard.

Tab order is the order you should press them in: bring the arm up and check
the mirror tracks, prove the pipeline with a single TCP goal, then run the
pick and place. Each step only makes sense once the one before it works.

Uses the same :class:`~soarm_sdk.dashboard.app.DashboardProfile` shape as
``soarm_sdk.cli.dashboard`` — one entry in :func:`profiles` per named panel
set — even though there is only the one profile today. A future stripped-down
or task-specific dashboard is a new entry there, not a new ``build_*`` module.
``profiles()`` builds the dict lazily rather than at import time, the same
reason :func:`build_app` imports ``soarm_sdk.dashboard`` inside itself: this
module has to stay importable (for ``DEFAULT_URDF`` alone) without viser.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional

if TYPE_CHECKING:
    from soarm_sdk.dashboard.app import DashboardProfile

__all__ = ["build_app", "profiles", "DEFAULT_URDF"]

#: The SO-101 — the revision this workspace actually has. The 3-D view maps
#: ticks through ~/.soarm_sdk/calibration.json, so it tracks the real arm
#: rather than a nominal zero.
DEFAULT_URDF = (
    Path(__file__).resolve().parents[3]
    / "SO-ARM100"
    / "Simulation"
    / "SO101"
    / "so101_new_calib.urdf"
)


def _register_plan_and_run(app) -> None:
    from soarm_sdk.dashboard.panels import setup

    from .panels.pickplace import build_pickplace_panel
    from .panels.tcp import build_tcp_panel
    from .panels.watchdog import build_watchdog_panel

    # TAMP needs a connection and live mirror, not hardware setup or
    # calibration controls. Those workflows live in soarm_sdk.
    app.register(setup.build_startup_panel())
    app.register(build_watchdog_panel())

    # The planning tabs drive the same 3-D view, so a replayed or executed
    # trajectory shows up in the same place the live arm does.
    app.register(
        build_tcp_panel(fk_update=app.fk_update, fk_update_ghost=app.fk_update_ghost)
    )
    app.register(
        build_pickplace_panel(
            fk_update=app.fk_update, fk_update_ghost=app.fk_update_ghost
        )
    )


def profiles() -> Dict[str, DashboardProfile]:
    # Deferred: importing soarm_sdk.dashboard.app requires viser, and this
    # module otherwise stays importable without it until build_app() runs.
    from soarm_sdk.dashboard.app import DashboardProfile

    return {
        "plan_and_run": DashboardProfile(
            name="plan_and_run",
            title="soarm_tamp — plan and run",
            register=_register_plan_and_run,
            description="Plan in the HPP container, run on the SO-101 host.",
        ),
    }


def build_app(
    *,
    device: str = "",
    baud: int = 1_000_000,
    port: int = 8080,
    urdf_path: Optional[Path] = None,
    joint_ids: Optional[List[int]] = None,
    use_stream: bool = True,
    profile: str = "plan_and_run",
):
    from soarm_sdk.dashboard import DashboardApp

    chosen = profiles()[profile]
    app = DashboardApp(
        title=chosen.title,
        port=port,
        device=device,
        baud=baud,
        urdf_path=urdf_path if urdf_path is not None else DEFAULT_URDF,
        joint_ids=joint_ids,
        use_stream=use_stream,
    )
    chosen.register(app)
    return app

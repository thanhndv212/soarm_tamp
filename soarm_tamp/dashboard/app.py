"""Assemble the soarm_tamp dashboard.

Tab order is the order you should press them in: bring the arm up and check
the mirror tracks, prove the pipeline with a single TCP goal, then run the
pick and place. Each step only makes sense once the one before it works.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

__all__ = ["build_app", "DEFAULT_URDF"]

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


def build_app(
    *,
    device: str = "",
    baud: int = 1_000_000,
    port: int = 8080,
    urdf_path: Optional[Path] = None,
    joint_ids: Optional[List[int]] = None,
    use_stream: bool = True,
):
    from soarm_sdk.dashboard import DashboardApp
    from soarm_sdk.dashboard.panels import setup

    from .panels.pickplace import build_pickplace_panel
    from .panels.tcp import build_tcp_panel
    from .panels.watchdog import build_watchdog_panel

    app = DashboardApp(
        title="soarm_tamp — plan and run",
        port=port,
        device=device,
        baud=baud,
        urdf_path=urdf_path if urdf_path is not None else DEFAULT_URDF,
        joint_ids=joint_ids,
        use_stream=use_stream,
    )

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
    return app

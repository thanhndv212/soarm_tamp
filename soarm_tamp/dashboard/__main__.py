"""``python -m soarm_tamp.dashboard`` — the plan-and-run dashboard.

The CLI layer (device/baud/port/urdf/interval-ms/calibration flags, the
streaming/rerun flags, auto-device-selection) lives in
``soarm_sdk.cli.dashboard.launch`` — shared with ``soarm-dashboard-setup``
and ``soarm-dashboard-calibration`` rather than a second, independently
drifting copy of it. This module only supplies what's specific to
soarm_tamp: which profile, which default URDF, and that streaming defaults
on (the plan-and-run panels need the persistent interface for TCP goals
and pick-and-place execution anyway, so there is no reason to make an
operator ask for it — ``--no-stream`` is there for the rare case that
matters).
"""

from __future__ import annotations

from typing import List, Optional

from .app import DEFAULT_URDF, profiles

__all__ = ["main"]


def main(argv: Optional[List[str]] = None) -> None:
    from soarm_sdk.cli.dashboard import launch

    launch(
        profiles()["plan_and_run"],
        argv,
        "soarm_tamp.dashboard",
        default_urdf=DEFAULT_URDF,
        default_use_stream=True,
    )


if __name__ == "__main__":
    main()

"""Read-only calibration and model-bound watchdog for TAMP operation."""

from __future__ import annotations

import math
from typing import Any, Dict

from soarm_sdk.calibration.pipeline import AcceptanceTolerances, CalibrationPipeline
from soarm_sdk.dashboard.app import Panel
from soarm_sdk.dashboard.fk import SOARM100_JOINT_NAMES

from ...conventions import safe_planning_bounds

__all__ = ["build_watchdog_panel", "watchdog_violations"]


def build_watchdog_panel() -> Panel:
    """Show planning-relevant divergence without exposing calibration actions."""
    handles: Dict[str, Any] = {}
    return Panel(
        "Execution Watchdog",
        lambda server, ctx: _build(server, handles),
        lambda ctx: _on_tick(ctx, handles),
    )


def _build(server: Any, handles: Dict[str, Any]) -> None:
    server.gui.add_markdown(
        "Read-only guard for planning. Calibrate with "
        "`soarm-dashboard-calibration`; this tab never changes offsets, signs, "
        "or servo limits."
    )
    handles["status"] = server.gui.add_markdown("*Waiting for a calibration and live arm state…*")


def _on_tick(ctx: Any, handles: Dict[str, Any]) -> None:
    warnings = watchdog_violations(ctx)
    handles["status"].content = (
        "✅ **No large calibration/model deviation detected.**"
        if not warnings
        else "⚠️ **Do not start a new plan or execution:**\n\n"
        + "\n".join(f"- {warning}" for warning in warnings)
    )


def watchdog_violations(ctx: Any) -> list[str]:
    """Return actionable TAMP preflight violations without changing the arm."""
    cal = getattr(ctx, "calibration", None)
    if cal is None:
        return ["no calibration loaded"]
    tolerance = AcceptanceTolerances.from_notes(cal.notes)
    if tolerance is None:
        return ["calibration has no explicit acceptance tolerances"]
    report = CalibrationPipeline(cal).report()
    incomplete = [stage.stage.value for stage in report.stages if not stage.passed]
    if incomplete:
        return ["calibration acceptance is incomplete: " + ", ".join(incomplete)]

    with ctx.lock:
        positions = dict(ctx.state.positions)
        connected = ctx.state.connected
    if not connected:
        return []

    warnings = []
    for name, deviation_deg in ctx.calibration_drift():
        if abs(math.radians(deviation_deg)) > tolerance.model_deviation_rad:
            warnings.append(
                f"mirror/file disagreement: {name} {deviation_deg:+.1f} deg"
            )

    bounds = dict(zip(cal.names, safe_planning_bounds(cal)))
    for sid, name in zip(ctx.joint_ids, SOARM100_JOINT_NAMES):
        ticks = positions.get(sid)
        joint = next((joint for joint in cal.joints if joint.name == name), None)
        if ticks is None or joint is None or name not in bounds:
            continue
        lo, hi = bounds[name]
        angle = joint.to_rad(ticks)
        over = max(lo - angle, angle - hi, 0.0)
        if over > tolerance.model_deviation_rad:
            warnings.append(
                f"arm/model planning-bound deviation: {name} "
                f"{math.degrees(over):.1f} deg"
            )

    return warnings

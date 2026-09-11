"""Locating and checking the arm's URDF-frame calibration.

This module used to carry the URDF-to-servo mapping itself, with offsets
derived from comparing two limit tables and signs left as an outright
guess. That belonged in the SDK, not here: ``soarm_sdk.calibration``
now owns the model, the seeding and the persistence, and
``ServoHardwareInterface`` applies it alongside joint-limit and step-size
clamping. What is left here is the part specific to planning:

* finding the calibration file,
* refusing to stream a trajectory against an unvalidated one,
* turning the calibration's measured travel into the joint bounds the
  planner is allowed to use.

The import of ``soarm_sdk`` is deliberately lazy. ``soarm_tamp.plan`` runs
inside the HPP container, which has pyhpp but no serial stack; only the
host-side executor needs the SDK, and a module-level import would break
planning.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:  # pragma: no cover
    from soarm_sdk.calibration.frame import RobotCalibration

# so101_new_calib.urdf joint limits, URDF joint order. Duplicated from
# soarm_sdk.calibration.seed on purpose: the container-side planner reads
# these without the SDK installed.
URDF_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

JOINT_ORDER: tuple[str, ...] = tuple(URDF_LIMITS)

#: Where the SDK's seeding CLI writes by default. Override with
#: ``SOARM_CALIBRATION`` for a second arm.
DEFAULT_CALIBRATION = Path.home() / ".soarm_sdk" / "calibration.json"


def calibration_path() -> Path:
    env = os.environ.get("SOARM_CALIBRATION")
    return Path(env) if env else DEFAULT_CALIBRATION


def load_calibration(path: str | Path | None = None) -> "RobotCalibration":
    """Load the arm's calibration, or explain how to make one."""
    from soarm_sdk.calibration.frame import RobotCalibration

    p = Path(path) if path else calibration_path()
    if not p.exists():
        raise FileNotFoundError(
            f"no calibration at {p}\n"
            "  Seed one (no hardware needed):\n"
            "    soarm-seed-calibration --lerobot <lerobot.json>"
        )
    return RobotCalibration.load(p)


def check_ready(cal: "RobotCalibration") -> list[str]:
    """Reasons this calibration must not drive a planned trajectory.

    Empty list means good to go. Returned rather than raised so a caller
    can show every problem at once instead of one per run.
    """
    problems: list[str] = []
    if not cal.validated:
        problems.append(
            "calibration is not validated — the direction signs are still "
            "assumed +1 and have never been checked on this arm"
        )
    if cal.names != list(JOINT_ORDER):
        problems.append(
            f"joint order is {cal.names}, expected {list(JOINT_ORDER)}"
        )
    if cal.suspect_joints:
        problems.append(
            "measured travel disagrees with the URDF on: "
            + ", ".join(cal.suspect_joints)
        )
    return problems


def safe_planning_bounds(cal: "RobotCalibration") -> list[tuple[float, float]]:
    """Joint bounds, in URDF radians, that this arm can actually reach.

    The intersection of the URDF's limits with the measured travel, mapped
    through the calibration. Planning against the URDF alone produces
    trajectories the arm cannot execute — the first working plan here drove
    wrist_roll to 2.841 rad, past the servo's stop, on 41 waypoints.
    Clamping at send time would deform the path instead of following it, so
    the restriction belongs in what the planner is allowed to use.

    Unlike the earlier hand-derived version, this needs no conservative
    both-signs guess: the calibration states the sign, so the reachable
    interval is known rather than bracketed.
    """
    out: list[tuple[float, float]] = []
    for j in cal.joints:
        a, b = j.to_rad(j.tick_min), j.to_rad(j.tick_max)
        reach_lo, reach_hi = min(a, b), max(a, b)
        u_lo, u_hi = URDF_LIMITS[j.name]
        out.append((max(u_lo, reach_lo), min(u_hi, reach_hi)))
    return out


def format_bounds_yaml(bounds: Sequence[tuple[float, float]]) -> str:
    """Render bounds as the joint_groups block of the task config."""
    return "\n".join(
        f"    - {{joint: so101/{name + ',':16s} initial: 0.0, "
        f"bounds: [{lo:.4f}, {hi:.4f}]}}"
        for name, (lo, hi) in zip(JOINT_ORDER, bounds)
    )

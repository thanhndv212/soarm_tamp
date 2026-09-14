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
            + " — and those joints' zeros were inferred from those same "
            "limits, so the mapping cannot be trusted. Re-zero them against "
            "a pose you can verify (soarm_sdk.calibration.rezero_from_pose)."
        )
    from soarm_sdk.calibration.pipeline import CalibrationPipeline

    report = CalibrationPipeline(cal).report()
    incomplete = [stage.stage.value.replace("_", " ") for stage in report.stages if not stage.passed]
    if incomplete:
        problems.append(
            "calibration acceptance is incomplete: "
            + ", ".join(incomplete)
            + " — complete it in soarm-dashboard-calibration"
        )
    return problems


def span_warnings(cal: "RobotCalibration") -> list[str]:
    """Span mismatches worth saying out loud that are not blocking.

    A joint whose zero came from a verified pose is unaffected by the URDF
    being wrong about how far it travels — that is why it is not in
    :func:`check_ready`. It still matters for *planning*: the URDF's limits
    are not this arm's real reach, so plan against ``safe_planning_bounds``
    rather than the model's own numbers.
    """
    informational = [
        j.name
        for j in cal.joints
        if j.span_mismatch and not j.suspect
    ]
    if not informational:
        return []
    return [
        "measured travel disagrees with the URDF on: "
        + ", ".join(informational)
        + " — zeros are pose-anchored so the mapping stands, but the URDF's "
        "limits understate this arm's reach; plan against the measured bounds."
    ]


SCENE_FILE = "scene.json"


def write_scene(run_dir: str | Path) -> Path:
    """Record the scene a plan was made in, next to its manifest.

    The manifest says where the arm should go, not what the world looked
    like when that was decided. Change the cube size or the grasp frame and
    every waypoint in an existing run still loads fine while referring to
    geometry that no longer exists — so it replays a collision-checked path
    around an object of the wrong size, or grips at an offset that misses.
    Cheap to record, and the only way execute.py can tell.
    """
    import json

    from .geometry import scene_fingerprint

    path = Path(run_dir) / SCENE_FILE
    path.write_text(json.dumps(scene_fingerprint(), indent=2) + "\n")
    return path


def check_scene(run_dir: str | Path) -> list[str]:
    """Reasons this run does not match the scene as it stands now.

    A run with no scene file predates the record and cannot be checked —
    reported as a problem rather than waved through, since the runs that
    predate it are exactly the stale ones.
    """
    import json

    from .geometry import compare_scene

    path = Path(run_dir) / SCENE_FILE
    if not path.exists():
        return [
            f"no {SCENE_FILE} in {run_dir} — this run predates scene "
            "recording, so it cannot be checked against the current "
            "geometry. Re-plan it."
        ]
    try:
        recorded = json.loads(path.read_text())
    except Exception as exc:
        return [f"could not read {path}: {exc}"]

    diffs = compare_scene(recorded)
    if not diffs:
        return []
    return [
        "this run was planned against a different scene: "
        + "; ".join(diffs)
        + ". Re-plan it."
    ]


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

    Once the travel has been *accepted* — repeated, non-simulated endpoint
    measurements agreeing within the arm's tolerance — it replaces the URDF's
    limits outright rather than being intersected with them. The URDF's
    numbers are a model's opinion and are conservative here; the mechanism
    stops where it stops, and planning against the narrower of the two throws
    away reach the arm actually has. Until the travel is accepted the
    intersection stands, because an unaccepted sweep can be the *encoder's*
    range rather than the joint's — a wrapped ``wrist_roll`` measures a full
    turn, and handing that to a planner as permission would be worse than
    being conservative.

    The policy itself lives in :mod:`soarm_sdk.calibration.limits` so this
    and ``ServoRobot`` cannot drift apart about what a bound means.
    """
    from soarm_sdk.calibration.limits import effective_limits

    declared = [URDF_LIMITS[j.name] for j in cal.joints]
    return effective_limits(cal, declared)


#: How far a YAML bound may sit from the computed one before it is stale.
#: The file carries 4 decimals, so anything past a rounding step is a real
#: disagreement rather than a formatting artefact.
BOUNDS_TOLERANCE_RAD = 2e-4


def stale_bounds(
    cal: "RobotCalibration", config_path: str | Path
) -> list[str]:
    """Joints where the planner's YAML bounds no longer match the arm.

    :func:`safe_planning_bounds` is computed from the live calibration, but
    the planner does not call it — it reads numbers baked into the task
    YAML, inside a container, with no way to know they were generated
    against a calibration that has since been re-zeroed four times. The
    bounds then drift in whichever direction the zeros moved, silently, and
    in the direction that matters: a bound that is now too *wide* lets the
    planner commit to a joint angle the arm cannot reach, and the deviation
    only appears at send time as clamping, which deforms the path rather
    than following it.

    So compare, and say which joints and which way. Returned as strings
    rather than raised, so every stale joint is reported at once.
    """
    import re

    text = Path(config_path).read_text()
    want = dict(zip(JOINT_ORDER, safe_planning_bounds(cal)))
    pat = re.compile(
        r"joint: so101/(?P<name>\w+),\s*initial:\s*[-\d.]+,\s*"
        r"bounds: \[\s*(?P<lo>[-\d.]+),\s*(?P<hi>[-\d.]+)\s*\]"
    )
    seen = set()
    out: list[str] = []
    for m in pat.finditer(text):
        name = m.group("name")
        seen.add(name)
        if name not in want:
            continue
        lo, hi = float(m.group("lo")), float(m.group("hi"))
        w_lo, w_hi = want[name]
        notes = []
        if abs(lo - w_lo) > BOUNDS_TOLERANCE_RAD:
            way = "past what the arm can reach" if lo < w_lo else "short of the arm's reach"
            notes.append(f"lower {lo:+.4f} vs {w_lo:+.4f} ({way})")
        if abs(hi - w_hi) > BOUNDS_TOLERANCE_RAD:
            way = "past what the arm can reach" if hi > w_hi else "short of the arm's reach"
            notes.append(f"upper {hi:+.4f} vs {w_hi:+.4f} ({way})")
        if notes:
            out.append(f"{name}: " + "; ".join(notes))
    missing = [n for n in JOINT_ORDER if n not in seen]
    if missing:
        out.append(f"not in the config at all: {', '.join(missing)}")
    return out


def format_bounds_yaml(bounds: Sequence[tuple[float, float]]) -> str:
    """Render bounds as the joint_groups block of the task config."""
    return "\n".join(
        f"    - {{joint: so101/{name + ',':16s} initial: 0.0, "
        f"bounds: [{lo:.4f}, {hi:.4f}]}}"
        for name, (lo, hi) in zip(JOINT_ORDER, bounds)
    )

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

# so101_new_calib.urdf joint limits, URDF joint order. Duplicated from the
# URDF itself on purpose: the container-side planner reads these without
# the SDK installed.
URDF_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

JOINT_ORDER: tuple[str, ...] = tuple(URDF_LIMITS)

#: Where soarm-calibrate-rom / soarm-dashboard-calibration write by
#: default. Override with ``SOARM_CALIBRATION`` for a second arm.
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
            "  Measure one on the arm:\n"
            "    soarm-calibrate-rom --arm-id <name>\n"
            "  then confirm it (or work through the guided dashboard):\n"
            "    soarm-dashboard-calibration"
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


def _yaml_bounds_by_name(config_path: str | Path) -> dict[str, tuple[float, float]]:
    """Parse the ``joint_groups`` bounds out of a task YAML, by joint name.

    Shared by :func:`stale_bounds` and :func:`unreachable_bounds` so there is
    exactly one regex that knows this file's shape, not two that could drift
    apart from each other.
    """
    import re

    text = Path(config_path).read_text()
    pat = re.compile(
        r"joint: so101/(?P<name>\w+),\s*initial:\s*[-\d.]+,\s*"
        r"bounds: \[\s*(?P<lo>[-\d.]+),\s*(?P<hi>[-\d.]+)\s*\]"
    )
    return {
        m.group("name"): (float(m.group("lo")), float(m.group("hi")))
        for m in pat.finditer(text)
    }


def assert_narrows(
    source: Sequence[tuple[float, float]],
    derived: Sequence[tuple[float, float]],
    names: Sequence[str],
    *,
    tolerance_rad: float = BOUNDS_TOLERANCE_RAD,
) -> list[str]:
    """Joints where *derived* claims range that *source* does not actually have.

    The rule this stack broke and every mature motion framework enforces
    explicitly (ros2_control's hard/soft split, MoveIt generating
    ``joint_limits.yaml`` from a URDF and only allowing edits to narrow it):
    a config derived from another may shrink it, never grow it. *source* is
    the ground truth for one comparison — a servo's EEPROM, a calibration's
    measured travel, a URDF — and *derived* is whatever downstream config
    claims to operate within it.

    Both :func:`phantom_range` (servo vs. the stack's belief) and
    :func:`unreachable_bounds` (the arm vs. the planner YAML) are this
    function with different *source*/*derived* pairs; new boundaries in the
    stack should be a new call to this, not a new bespoke comparison.

    Returns one message per joint where *derived* reaches outside *source*
    in either direction, beyond *tolerance_rad*. Order-independent per pair;
    *names* just labels each ``(source, derived)`` pair for the message.
    """
    out: list[str] = []
    for name, (s_lo, s_hi), (d_lo, d_hi) in zip(names, source, derived):
        notes = []
        if s_lo - d_lo > tolerance_rad:
            notes.append(f"lower {d_lo:+.4f} claimed vs {s_lo:+.4f} actual")
        if d_hi - s_hi > tolerance_rad:
            notes.append(f"upper {d_hi:+.4f} claimed vs {s_hi:+.4f} actual")
        if notes:
            out.append(f"{name}: " + "; ".join(notes) + " (past what the source allows)")
    return out


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
    rather than raised, so every stale joint is reported at once. Unlike
    :func:`unreachable_bounds`, reports *both* directions — a narrow bound
    is only a cost in reach, not a hazard, but it is still worth a person's
    attention.
    """
    want = dict(zip(JOINT_ORDER, safe_planning_bounds(cal)))
    have = _yaml_bounds_by_name(config_path)
    out: list[str] = []
    for name in JOINT_ORDER:
        if name not in have:
            continue
        lo, hi = have[name]
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
    missing = [n for n in JOINT_ORDER if n not in have]
    if missing:
        out.append(f"not in the config at all: {', '.join(missing)}")
    return out


def unreachable_bounds(
    cal: "RobotCalibration", config_path: str | Path
) -> list[str]:
    """Joints the planner is allowed to use past where this arm can go.

    The dangerous half of :func:`stale_bounds`. A bound that is too *narrow*
    costs reach the arm has; a bound that is too *wide* produces a plan the
    arm cannot execute — and it does not fail loudly. Every command is
    clamped at the real limit, so the joint parks against its hard stop
    while the trajectory carries on asking for more, and the only symptom is
    the arm quietly falling behind a path it can never reach.

    Measured here: ``wrist_flex`` bounded at +1.6581 in the YAML against a
    mechanism that stops at +1.2686. A plan through +1.5136 left the joint
    0.245 rad behind — precisely the overshoot — for the rest of the run.
    """
    have = _yaml_bounds_by_name(config_path)
    names = [n for n in JOINT_ORDER if n in have]
    source = dict(zip(JOINT_ORDER, safe_planning_bounds(cal)))
    return assert_narrows(
        source=[source[n] for n in names],
        derived=[have[n] for n in names],
        names=names,
    )


#: Range smaller than this is not worth reporting: a tick either way is
#: rounding, not a joint the arm cannot use.
PHANTOM_TOLERANCE_RAD = 0.02


def phantom_range(
    cal: "RobotCalibration", servo_limits_ticks: dict[int, tuple[int, int]]
) -> list[str]:
    """Range the stack believes in that the servos' own firmware forbids.

    Every layer here — :func:`safe_planning_bounds`, the planner's YAML,
    ``ServoRobot``'s enforced limits — derives what the arm can reach from
    the *calibration*. None of them ask the servos, which carry their own
    MIN/MAX_ANGLE_LIMIT in EEPROM and enforce it below everything software
    can see.

    When those disagree the failure is silent and very hard to read. A goal
    past a servo's cap is accepted into GOAL_POSITION, reports no error and
    draws no current, and the joint simply stops contributing to the
    trajectory. Measured on thanh_arm: servo 4 (``wrist_flex``) caps at 3046
    ticks against a calibration that recorded 3314, so a plan through
    +1.2390 rad left the joint parked at +0.8544 for an entire run — every
    remaining waypoint timing out against an arrival that could not happen,
    which is what turned the arm's motion into stop-start.

    *servo_limits_ticks* maps servo id -> (min_ticks, max_ticks), as
    :meth:`ServoHardwareInterface.read_angle_limits` returns it.
    """
    believed = safe_planning_bounds(cal)
    names, source, derived = [], [], []
    for i, (sid, j) in enumerate(zip(sorted(servo_limits_ticks), cal.joints)):
        min_t, max_t = servo_limits_ticks[sid]
        if min_t < 0 or max_t < 0:
            continue
        a, b = j.to_rad(min_t), j.to_rad(max_t)
        names.append(f"{j.name} (servo {sid})")
        source.append((min(a, b), max(a, b)))
        derived.append(believed[i])
    return assert_narrows(
        source=source, derived=derived, names=names,
        tolerance_rad=PHANTOM_TOLERANCE_RAD,
    )


def waypoints_beyond_servo_limits(
    waypoint_rows: Sequence[Sequence[float]],
    servo_limits_ticks: dict[int, tuple[int, int]],
    cal: "RobotCalibration",
    joint_ids: Sequence[int],
) -> dict[str, tuple[float, float, float, float]]:
    """The worst violation per joint, for a manifest against the live servos.

    Shared by the dashboard's pre-execute check and ``execute.py``'s own —
    before this, only the dashboard path checked, so the bare
    ``python -m soarm_tamp.execute`` CLI used to verify every other fix in
    this document had no servo-limit preflight at all.

    *waypoint_rows* are JOINT_ORDER-ordered radian rows (a manifest's raw or
    resampled waypoints). *servo_limits_ticks* maps servo id -> raw
    ``(min, max)`` ticks, as :meth:`ServoHardwareInterface.read_angle_limits`
    returns it. *joint_ids* is the servo id at each JOINT_ORDER position —
    positional, the same convention used everywhere else in this stack.

    Returns ``{name: (over, value, lo, hi)}`` for joints the plan asks past
    the live servo range; empty when the whole manifest is within it. A goal
    past a servo's cap is accepted and silently never acted on — no error,
    no current draw — so this exists to say so *before* streaming it, not
    after the joint has quietly stopped moving.
    """
    allowed: dict[str, tuple[float, float]] = {}
    for sid, j in zip(joint_ids, cal.joints):
        lims = servo_limits_ticks.get(sid)
        if lims is None or lims[0] < 0 or lims[1] < 0:
            continue
        a, b = j.to_rad(lims[0]), j.to_rad(lims[1])
        allowed[j.name] = (min(a, b), max(a, b))

    worst: dict[str, tuple[float, float, float, float]] = {}
    for row in waypoint_rows:
        for name, value in zip(JOINT_ORDER, row):
            if name not in allowed:
                continue
            lo, hi = allowed[name]
            over = max(lo - value, value - hi, 0.0)
            if over > worst.get(name, (0.0, 0.0, 0.0, 0.0))[0]:
                worst[name] = (over, value, lo, hi)
    return worst


def format_bounds_yaml(bounds: Sequence[tuple[float, float]]) -> str:
    """Render bounds as the joint_groups block of the task config."""
    return "\n".join(
        f"    - {{joint: so101/{name + ',':16s} initial: 0.0, "
        f"bounds: [{lo:.4f}, {hi:.4f}]}}"
        for name, (lo, hi) in zip(JOINT_ORDER, bounds)
    )

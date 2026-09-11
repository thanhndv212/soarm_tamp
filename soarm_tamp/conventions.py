"""Joint convention mapping between the planning URDF and the real servos.

This is the most dangerous module in the package. Everything upstream is
geometry that either solves or does not; a mistake here produces a plan
that looks perfect and drives the arm into the table.

The problem
-----------
Planning happens against ``SO-ARM100/Simulation/SO101/so101_new_calib.urdf``.
Commands go to ``soarm_sdk``, whose canonical joint config is
``soarm_sdk/configs/soarm100.yaml``. The two do NOT share a convention:

    URDF joint      SDK joint     URDF mid   SDK mid   offset
    shoulder_pan    Rotation         0.000     0.000    0.000
    shoulder_lift   Pitch            0.000    -1.573   -1.573   ~ -pi/2
    elbow_flex      Elbow            0.000     1.483   +1.483   ~ +pi/2
    wrist_flex      Wrist_Pitch      0.000     0.000    0.000
    wrist_roll      Wrist_Roll       0.049     0.000   -0.049
    gripper         Jaw              0.785     0.788   +0.003

Offsets can be recovered by comparing the midpoints of the two limit
tables, as above. **Signs cannot.** Five of the six URDF ranges are
symmetric about zero, and a symmetric range maps onto its SDK counterpart
equally well with sign +1 or -1 — the limits simply carry no information
to distinguish them. No amount of reading either file settles it.

So the signs below are assumptions, not measurements, and the only thing
that can settle them is the physical arm. ``calibrate_conventions.py``
does that with the servos' torque disabled, so nothing can move under
power while the question is still open. Until it has been run,
``execute.py`` refuses to stream a trajectory.

Ordering
--------
Both sides happen to enumerate the same six joints base-to-tip, so the
mapping is positional. That is asserted in ``URDF_JOINT_ORDER`` /
``SDK_JOINT_ORDER`` rather than assumed silently.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

# Order of the first six entries of a planned configuration vector — the
# joints of the `so101` model, as pinocchio builds them from the URDF.
URDF_JOINT_ORDER: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# soarm_sdk/configs/soarm100.yaml `joint_names`, servo IDs 1-6.
SDK_JOINT_ORDER: tuple[str, ...] = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
)

# Recovered from the limit-table midpoints shown in the module docstring.
# servo_angle = sign * urdf_angle + offset
DEFAULT_OFFSETS_RAD: tuple[float, ...] = (
    0.0,
    -math.pi / 2,
    +math.pi / 2,
    0.0,
    0.0,
    0.0,
)

# ASSUMED. See the module docstring: not derivable from either model.
ASSUMED_SIGNS: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# Servo-side limits, from soarm_sdk/configs/soarm100.yaml. Commands are
# clamped to these as a last line of defence regardless of what the plan
# asked for.
SDK_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-1.92, 1.92),
    (-3.32, 0.174),
    (-0.174, 3.14),
    (-1.66, 1.66),
    (-2.79, 2.79),
    (-0.174, 1.75),
)

VERIFIED_FILE = Path(__file__).parent / "conventions_verified.json"


@dataclass(frozen=True)
class JointConvention:
    """``servo = sign * urdf + offset``, per joint, plus its provenance."""

    signs: tuple[float, ...] = ASSUMED_SIGNS
    offsets: tuple[float, ...] = DEFAULT_OFFSETS_RAD
    verified: bool = False
    source: str = "assumed (limit-table midpoints; signs unverified)"
    notes: dict = field(default_factory=dict)

    def urdf_to_servo(self, q_urdf: list[float]) -> list[float]:
        if len(q_urdf) != 6:
            raise ValueError(f"expected 6 joint values, got {len(q_urdf)}")
        return [s * q + o for s, q, o in zip(self.signs, q_urdf, self.offsets)]

    def servo_to_urdf(self, q_servo: list[float]) -> list[float]:
        if len(q_servo) != 6:
            raise ValueError(f"expected 6 joint values, got {len(q_servo)}")
        return [(q - o) / s for s, q, o in zip(self.signs, q_servo, self.offsets)]

    def clamp_servo(self, q_servo: list[float]) -> tuple[list[float], int]:
        """Clamp to the SDK limits. Returns (clamped, n_clamped)."""
        out, n = [], 0
        for q, (lo, hi) in zip(q_servo, SDK_LIMITS_RAD):
            c = min(max(q, lo), hi)
            if abs(c - q) > 1e-9:
                n += 1
            out.append(c)
        return out, n


def load() -> JointConvention:
    """Return the verified convention if one exists, else the assumed one."""
    if not VERIFIED_FILE.exists():
        return JointConvention()
    data = json.loads(VERIFIED_FILE.read_text())
    return JointConvention(
        signs=tuple(float(v) for v in data["signs"]),
        offsets=tuple(float(v) for v in data["offsets"]),
        verified=True,
        source=data.get("source", str(VERIFIED_FILE)),
        notes=data.get("notes", {}),
    )


def save_verified(signs, offsets, notes: dict | None = None) -> Path:
    VERIFIED_FILE.write_text(
        json.dumps(
            {
                "signs": list(signs),
                "offsets": list(offsets),
                "source": "calibrate_conventions.py against the physical arm",
                "notes": notes or {},
            },
            indent=2,
        )
        + "\n"
    )
    return VERIFIED_FILE


# URDF-native joint limits, from so101_new_calib.urdf.
URDF_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-1.91986, 1.91986),
    (-1.74533, 1.74533),
    (-1.69, 1.69),
    (-1.65806, 1.65806),
    (-2.74385, 2.84121),
    (-0.174533, 1.74533),
)


def safe_planning_bounds() -> list[tuple[float, float]]:
    """Joint bounds, in URDF coordinates, that the servos can actually reach.

    The two models disagree about range, not just about zero. Planning
    against the URDF's limits alone produces trajectories the arm cannot
    execute: the first successful plan here drove wrist_roll to 2.841 rad,
    0.051 rad past the servo's 2.79 stop, on 41 waypoints. Clamping at
    send time would silently deform the path instead of following it, so
    the fix belongs upstream, in what the planner is allowed to use.

    Because the signs are unverified (see the module docstring), each
    joint is intersected against the servo range under BOTH sign
    hypotheses. That is deliberately conservative: it costs a little reach
    and it means these bounds stay valid whichever way calibration lands.
    The jaw (index 5) is exempt and keeps its URDF range. Its limits are
    asymmetric, so the both-signs intersection would collapse it to
    +/-0.174 rad — a 10 degree jaw that cannot open around anything. It
    does not need the protection either: it is frozen throughout planning,
    and at execution it is driven straight from geometry.py's measured
    JAW_TABLE rather than from any planned trajectory.
    """
    out = []
    for i, ((ul, uh), (sl, sh), o) in enumerate(
        zip(URDF_LIMITS_RAD, SDK_LIMITS_RAD, DEFAULT_OFFSETS_RAD)
    ):
        if i == 5:
            out.append((ul, uh))
            continue
        lo = max(ul, sl - o, o - sh)
        hi = min(uh, sh - o, o - sl)
        out.append((lo, hi))
    return out

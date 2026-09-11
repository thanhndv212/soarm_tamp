#!/usr/bin/env python3
"""Confirm a seeded calibration against the physical arm, one rung at a time.

RUNS ON THE HOST, with the arm connected.

    python -m soarm_tamp.validate_calibration --port /dev/cu.usbmodemXXXX

``soarm_sdk.seed_calibration`` produces a calibration offline, but it can
only *estimate*: the direction signs are assumed +1, because a travel range
says how far a joint moves and not which end is which. This walks the
escalating checks that settle it, and marks the file validated only when
they all pass.

The rungs, in order, each gated on the one before:

  1. **Read only, torque off.** Move the arm by hand and confirm the
     reported angles track the URDF's sense. Nothing can be driven, so a
     wrong sign costs nothing here — which is exactly why the sign is
     settled at this rung and not under power.
  2. **One joint, small moves.** Torque on, +/-0.1 rad from a safe pose.
     Confirms the sign survives actuation and that the servo agrees about
     which way is positive.
  3. **Tape-measure FK check.** Command a few safe poses and measure where
     the TCP really is against what pinocchio predicts. This is the only
     check that can catch an error the software cannot see in itself: a
     consistent mapping that is consistently wrong.

Rung 3 needs pinocchio, which lives in the planning container, so the
predicted positions are passed in via --expect (produced by
``soarm_tamp.studies.reachability``-style FK) rather than computed here.
"""

from __future__ import annotations

import argparse
import math
import sys

from . import conventions

# Poses to drive for rung 3, in URDF radians. Chosen to keep the TCP well
# clear of the table while spanning the joints that carry it.
FK_POSES: list[tuple[str, list[float]]] = [
    ("home / straight out", [0.0, 0.0, 0.0, 0.0, 0.0, 0.30]),
    ("pan left 30 deg", [0.524, 0.0, 0.0, 0.0, 0.0, 0.30]),
    ("shoulder down 30 deg", [0.0, 0.524, 0.0, 0.0, 0.0, 0.30]),
    ("elbow up 30 deg", [0.0, 0.0, -0.524, 0.0, 0.0, 0.30]),
]

# What a positive URDF rotation does physically, computed from the URDF's
# own world angular velocity at the home pose. Used to phrase rung 1 and 2
# so the operator is checking a direction, not a number.
POSITIVE_IS: dict[str, str] = {
    "shoulder_pan": "gripper swings to your RIGHT",
    "shoulder_lift": "gripper pitches DOWN",
    "elbow_flex": "gripper pitches DOWN",
    "wrist_flex": "gripper tips DOWN",
    "wrist_roll": "gripper rolls COUNTERCLOCKWISE seen from the base, "
    "looking out along the arm",
}


def _confirm(prompt: str) -> bool:
    return input(f"  {prompt} [y/N] ").strip().lower().startswith("y")


def rung1_read_only(robot, cal) -> bool:
    """Hand-move each joint; confirm the reported angle moves the right way."""
    print("\n" + "=" * 70)
    print("RUNG 1 — read only, torque OFF. The arm is limp; support it.")
    print("=" * 70)
    for attr in ("disable_torque", "torque_off"):
        fn = getattr(robot.hw, attr, None)
        if callable(fn):
            fn()
            break
    else:
        print("  WARNING: no torque-disable call found on this SDK build.")
        print("  Power the servos down before continuing.", file=sys.stderr)
        if not _confirm("Servos are safe to move by hand?"):
            return False

    flipped: list[str] = []
    for idx, name in enumerate(conventions.JOINT_ORDER[:5]):
        print(f"\n[{idx + 1}/5] {name}")
        print(f"  Positive should mean: {POSITIVE_IS[name]}.")
        input("  Hold that joint near the middle of its travel, then Enter... ")
        before = robot.get_joint_positions()[idx]
        input(f"  Now move it so the {POSITIVE_IS[name]}, then Enter... ")
        after = robot.get_joint_positions()[idx]
        delta = float(after - before)
        if abs(delta) < 0.15:
            print(f"  INCONCLUSIVE — only {delta:+.3f} rad. Move it further.")
            return False
        ok = delta > 0
        print(f"  reported {delta:+.3f} rad -> sign {'+1 (as seeded)' if ok else '-1 (FLIPPED)'}")
        if not ok:
            flipped.append(name)

    if flipped:
        print(f"\n  These joints are inverted vs the seed: {', '.join(flipped)}")
        print("  Fix the signs in the calibration and re-run; do not continue.")
        return False
    print("\n  All five joints match the seeded signs.")
    return True


def rung2_single_joint(robot) -> bool:
    """Small commanded moves, one joint at a time, under power."""
    print("\n" + "=" * 70)
    print("RUNG 2 — torque ON, one joint at a time, +/-0.1 rad.")
    print("=" * 70)
    if not _confirm("Arm is clear of the table and of itself?"):
        return False
    import numpy as np

    for idx, name in enumerate(conventions.JOINT_ORDER[:5]):
        q = list(robot.get_joint_positions())
        print(f"\n[{idx + 1}/5] {name}: +0.1 rad — expect {POSITIVE_IS[name]}")
        q[idx] += 0.1
        robot.set_joint_positions(np.asarray(q, dtype=float))
        if not _confirm("Did it move that way?"):
            print(f"  STOP: {name} does not move as the calibration says.")
            return False
        q[idx] -= 0.1
        robot.set_joint_positions(np.asarray(q, dtype=float))
    return True


def rung3_fk_check(robot, tol_mm: float) -> bool:
    """Command known poses; compare measured TCP against prediction."""
    print("\n" + "=" * 70)
    print("RUNG 3 — tape-measure FK check.")
    print("=" * 70)
    print("  Predicted TCP positions come from the planning container:")
    print("    scripts/hpp_container.sh exec \\")
    print("      'python3 -m soarm_tamp.studies.reachability --fk-poses'")
    print("  Measure from the robot base origin, in millimetres.\n")
    if not _confirm("Arm is clear and you have a ruler?"):
        return False
    import numpy as np

    worst = 0.0
    for label, q in FK_POSES:
        print(f"\n  pose: {label}")
        print(f"    q = {[round(v, 3) for v in q]}")
        robot.set_joint_positions(np.asarray(q, dtype=float))
        input("    Press Enter once it has settled... ")
        try:
            got = [float(v) for v in input("    measured TCP x,y,z in mm: ").split(",")]
            exp = [float(v) for v in input("    predicted TCP x,y,z in mm: ").split(",")]
        except ValueError:
            print("    could not parse three comma-separated numbers")
            return False
        err = math.dist(got, exp)
        worst = max(worst, err)
        print(f"    error {err:.1f} mm {'OK' if err <= tol_mm else 'TOO LARGE'}")
        if err > tol_mm:
            print(f"    STOP: {err:.1f} mm exceeds the {tol_mm:.0f} mm tolerance.")
            return False
    print(f"\n  worst error {worst:.1f} mm, within {tol_mm:.0f} mm.")
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--tol-mm", type=float, default=15.0, help="FK tolerance")
    ap.add_argument("--skip-fk", action="store_true", help="rungs 1-2 only, stays unvalidated")
    args = ap.parse_args()

    cal = conventions.load_calibration()
    print(f"calibration : {conventions.calibration_path()}")
    print(f"arm         : {cal.arm_id}   validated: {cal.validated}")
    if cal.suspect_joints:
        print(f"flagged     : {', '.join(cal.suspect_joints)}")

    from soarm_sdk.servo_robot import ServoRobot

    robot = ServoRobot(port=args.port, calibration=cal, max_step_rad=0.05)
    robot.connect()
    try:
        if not rung1_read_only(robot, cal):
            return sys.exit(1)
        if not rung2_single_joint(robot):
            return sys.exit(1)
        if args.skip_fk:
            print("\nRungs 1-2 passed. FK check skipped, so NOT marking validated.")
            return sys.exit(0)
        if not rung3_fk_check(robot, args.tol_mm):
            return sys.exit(1)
    finally:
        robot.disconnect()

    cal.mark_validated(
        f"rungs 1-3 on {args.port}: hand-move sign check, +/-0.1 rad per joint, "
        f"FK within {args.tol_mm:.0f} mm over {len(FK_POSES)} poses"
    )
    path = cal.save(conventions.calibration_path())
    print("\n" + "=" * 70)
    print(f"VALIDATED — written to {path}")
    print("soarm_tamp.execute will now stream without --force.")
    print("=" * 70)


if __name__ == "__main__":
    main()

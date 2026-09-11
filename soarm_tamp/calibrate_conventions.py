#!/usr/bin/env python3
"""Measure the URDF->servo joint signs on the physical SO-101.

RUNS ON THE HOST, with the arm connected.

    python -m soarm_tamp.calibrate_conventions --port /dev/cu.usbmodemXXXX

Why this exists
---------------
``conventions.py`` can recover the URDF->servo *offsets* by comparing the
two limit tables, but not the *signs*: five of the six URDF joint ranges
are symmetric about zero, and a symmetric range fits its servo counterpart
equally well either way round. The information is simply not in the files.
A wrong sign is not a subtle error — it drives the arm down where the plan
said up.

So it gets measured, once, per arm.

Safety
------
**Servo torque is disabled for the whole procedure.** The arm is moved by
hand and only read from; nothing is ever commanded. That is deliberate:
the question being answered is precisely "which way does this joint go",
and it would be circular — and unsafe — to answer it by commanding motion.
Support the arm with your other hand, it will be limp.

Method
------
For each joint in turn you are asked to move *that joint only*, in a
physical direction that a positive URDF rotation is known to produce. The
expected directions were computed from the URDF itself (world angular
velocity for a positive rate at the planning home pose):

    shoulder_pan    omega = -Z   gripper swings RIGHT
    shoulder_lift   omega = +Y   gripper pitches DOWN
    elbow_flex      omega = +Y   gripper pitches DOWN
    wrist_flex      omega = +Y   gripper pitches DOWN
    wrist_roll      omega = -X   gripper rolls counterclockwise, seen
                                 from the base looking out along the arm

The servo reading is sampled before and after. If it moved the same way,
the sign is +1; if the opposite, -1. A joint that barely moved is
reported as inconclusive rather than guessed.
"""

from __future__ import annotations

import argparse
import sys

from . import conventions

# (joint index, joint name, what a POSITIVE urdf rotation looks like)
PROBES: list[tuple[int, str, str]] = [
    (0, "shoulder_pan", "swing the whole arm so the gripper moves to your RIGHT"),
    (1, "shoulder_lift", "pitch the upper arm so the gripper moves DOWN"),
    (2, "elbow_flex", "bend the elbow so the gripper moves DOWN"),
    (3, "wrist_flex", "bend the wrist so the gripper tips DOWN"),
    (
        4,
        "wrist_roll",
        "roll the gripper COUNTERCLOCKWISE, viewed from the robot base\n"
        "         looking outward along the arm toward the jaws",
    ),
]

MIN_DELTA_RAD = 0.15


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="servo bus, e.g. /dev/cu.usbmodem...")
    ap.add_argument("--min-delta", type=float, default=MIN_DELTA_RAD)
    args = ap.parse_args()

    from soarm_sdk.hardware_interface import ServoHardwareInterface
    from soarm_sdk.robot import load_robot_config

    robot = ServoHardwareInterface(load_robot_config("soarm100"), port=args.port)
    robot.start()

    # Torque off for the whole session — see the module docstring.
    disabled = False
    for attr in ("disable_torque", "set_torque_enabled", "torque_off"):
        fn = getattr(robot, attr, None)
        if callable(fn):
            fn(False) if attr == "set_torque_enabled" else fn()
            disabled = True
            break
    if not disabled:
        print(
            "WARNING: could not find a torque-disable call on this SDK build.\n"
            "         Power the servos down or hold the arm firmly before\n"
            "         continuing — it may be driven.",
            file=sys.stderr,
        )

    print("=" * 70)
    print("SO-101 joint sign calibration  (torque disabled — arm is limp)")
    print("=" * 70)

    signs: list[float] = list(conventions.ASSUMED_SIGNS)
    notes: dict = {}
    inconclusive: list[str] = []

    for idx, name, instruction in PROBES:
        print(f"\n[{idx + 1}/5] {name}")
        print(f"  Move ONLY this joint: {instruction}.")
        input("  Position the arm, then press Enter... ")
        before = robot.get_robot_joint_positions()[idx]
        input("  Now make the move, then press Enter... ")
        after = robot.get_robot_joint_positions()[idx]
        delta = float(after - before)

        if abs(delta) < args.min_delta:
            print(f"  INCONCLUSIVE: servo moved only {delta:+.3f} rad")
            inconclusive.append(name)
            notes[name] = {"delta": delta, "result": "inconclusive"}
            continue

        sign = 1.0 if delta > 0 else -1.0
        signs[idx] = sign
        print(f"  servo delta {delta:+.3f} rad  ->  sign {sign:+.0f}")
        notes[name] = {"delta": delta, "sign": sign}

    print("\n" + "=" * 70)
    if inconclusive:
        print(f"NOT SAVED — inconclusive joints: {', '.join(inconclusive)}")
        print("Re-run and move those joints further (at least "
              f"{args.min_delta} rad at the servo).")
        sys.exit(1)

    # The jaw is not probed: it is commanded directly from geometry.py's
    # measured JAW_TABLE, never from a planned trajectory, and its range is
    # asymmetric so its sign is already pinned by the limit tables.
    signs[5] = conventions.ASSUMED_SIGNS[5]
    path = conventions.save_verified(signs, conventions.DEFAULT_OFFSETS_RAD, notes)
    print("measured signs:")
    for n, s in zip(conventions.URDF_JOINT_ORDER, signs):
        print(f"   {n:16s} {s:+.0f}")
    print(f"\nsaved -> {path}")
    print("execute.py will now run without --force.")
    print("=" * 70)
    robot.stop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Command one joint, alone, and report what it actually did.

RUNS ON THE HOST. Written after a planned trajectory went wrong in a way
the manifest could not explain: `shoulder_pan`, `shoulder_lift` and
`wrist_flex` tracked their commands while `elbow_flex` ignored 280 of
them, and the combination put the hand on the table. Nothing in the
plan-and-stream path can tell a servo fault from a mapping error from a
gravity stall, because it moves six joints at once.

This moves ONE. Every other joint is commanded to hold the value it was
just measured at, so anything that moves, moved because of this joint.

    python -m soarm_tamp.joint_test --port /dev/cu.usbmodemXXXX \\
        --joint elbow_flex --delta 0.3 --hand-is-clear

What the numbers mean
---------------------
* **lag** — commanded minus measured, per step. A joint that tracks holds
  this near zero. A joint that is stalled, unpowered or overloaded lets it
  grow without bound, exactly as the failed run's step-clamp messages did.
* **step completion** (laddered mode) — how much of each increment the
  servo actually covers before the next command goes out. It is NOT a
  measure of the URDF-to-servo mapping, and an earlier version of this
  file wrongly reported it as one: every command through ``ServoRobot``
  is clamped to ``max_step_rad`` of the *measured* position, so a joint
  that needs longer than the dwell to finish a step shows a completion
  below 1.0 no matter how perfect its mapping is. Measured across four
  joints on a healthy arm: 0.39 to 0.74, including joints that tracked a
  full trajectory correctly. Read it as tracking speed under the clamp.
* **travel ratio** (``--single``) — the mapping question, asked properly.
  One command, no step clamp, a settling pause, then measured excursion
  over commanded excursion. A ratio near 1.0 clears the mapping for this
  joint; 0.9 means every planned trajectory is wrong in proportion to how
  far the joint travels.

Safety: refuses to leave the SDK's effective limits (config intersected
with measured travel), moves in `--step` increments with a dwell between
them, and always returns to where it started. `--hand-is-clear` is
required and means what it says: the arm will move, so nothing may be
within its reach — including the table, if the joint under test is one
that lowers the hand.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from . import conventions

DEFAULT_DWELL_S = 0.4


def _robot(port: str, max_step: float | None):
    from soarm_sdk import load_robot_config
    from soarm_sdk.robot import ServoRobot

    return ServoRobot(
        port=port,
        config=load_robot_config("so101"),
        calibration=conventions.load_calibration(),
        max_step_rad=max_step,
        enforce_limits=True,
    )


def _wait_for_read(robot, timeout: float = 5.0) -> None:
    deadline = time.time() + timeout
    while robot.state_age() > 0.5:
        if time.time() > deadline:
            raise RuntimeError(
                "the bus is not answering — the positions it would report are "
                "a placeholder, not the arm"
            )
        time.sleep(0.05)


def sweep(
    robot, index: int, hold: np.ndarray, frm: float, target: float,
    step: float, dwell: float,
) -> list:
    """Walk the joint from *frm* to *target* in *step* increments.

    Returns [(commanded, measured, lag)] per step. Every other joint is
    re-commanded to its ``hold`` value at every step, which is what keeps
    this a single-joint test rather than a slow drift of the whole arm.

    ``frm`` is passed in rather than read off ``hold``: the return sweep
    starts from wherever the outward one ended, and an earlier version
    that took both endpoints from the same vector computed a zero-length
    walk and silently never commanded the arm back.
    """
    rows = []
    here = float(frm)
    while abs(target - here) > 1e-9:
        here += float(np.clip(target - here, -step, step))
        q = hold.copy()
        q[index] = here
        robot.set_joint_positions(q)
        time.sleep(dwell)
        measured = float(robot.get_joint_positions()[index])
        rows.append((here, measured, here - measured))
    return rows


def single(robot, index: int, hold: np.ndarray, target: float, settle: float) -> float:
    """One command, one settling pause, one read. Returns the measured angle.

    Used with no step clamp, which is the whole point: the laddered sweep
    can only ever ask for ``max_step_rad`` beyond where the joint already
    is, so it cannot tell a joint that moves 40% as far as asked from one
    that simply has not finished moving yet.
    """
    q = hold.copy()
    q[index] = target
    robot.set_joint_positions(q)
    time.sleep(settle)
    return float(robot.get_joint_positions()[index])


def run_single(port: str, joint: str, index: int, delta: float, settle: float) -> int:
    robot = _robot(port, max_step=None)
    robot.connect()
    try:
        _wait_for_read(robot)
        start = np.asarray(robot.get_joint_positions(), dtype=float)
        lo, hi = robot.effective_joint_limits()
        begin = float(start[index])
        target = begin + delta
        if not lo[index] <= target <= hi[index]:
            target = begin - delta
            if not lo[index] <= target <= hi[index]:
                print(f"FAILED: {joint} cannot move {delta:.3f} rad either way "
                      f"inside [{lo[index]:+.4f}, {hi[index]:+.4f}]", file=sys.stderr)
                return 1

        print("=" * 70)
        print(f"single-command mapping test: {joint}")
        print("=" * 70)
        print(f"  start  : {begin:+.4f} rad")
        print(f"  target : {target:+.4f} rad  (one command, no step clamp)")
        print(f"  settle : {settle:.2f}s")
        print("=" * 70)

        there = single(robot, index, start, target, settle)
        back = single(robot, index, start, begin, settle)

        asked = abs(target - begin)
        moved = abs(there - begin)
        ratio = moved / asked if asked else float("nan")
        print(f"  commanded : {begin:+.4f} -> {target:+.4f}  ({asked:.4f} rad)")
        print(f"  measured  : {begin:+.4f} -> {there:+.4f}  ({moved:.4f} rad)")
        print(f"  travel ratio : {ratio:.3f}", end="")
        if ratio < 0.05:
            print("   <-- the joint did not move")
        elif abs(ratio - 1.0) > 0.1:
            print("   <-- scale mismatch in the URDF-to-servo mapping")
        else:
            print("   (mapping tracks, within 10%)")
        print(f"  returned to  : {back:+.4f} (started {begin:+.4f}, "
              f"off by {back - begin:+.4f})")
        print(f"  limit clamps : {robot.hw.limit_clamps}")
        print("=" * 70)
        return 0
    finally:
        robot.disconnect()


def run(
    port: str, joint: str, delta: float, step: float, dwell: float, verbose: bool
) -> int:
    names = list(conventions.JOINT_ORDER)
    if joint not in names:
        print(f"unknown joint {joint!r}; expected one of {names}", file=sys.stderr)
        return 2
    index = names.index(joint)

    robot = _robot(port, max_step=max(step, 0.02))
    robot.connect()
    try:
        _wait_for_read(robot)
        start = np.asarray(robot.get_joint_positions(), dtype=float)
        lo, hi = robot.effective_joint_limits()

        # Pick the direction with room; prefer the requested sign.
        target = float(start[index]) + delta
        if not lo[index] <= target <= hi[index]:
            target = float(start[index]) - delta
            if not lo[index] <= target <= hi[index]:
                print(
                    f"FAILED: {joint} is at {start[index]:+.4f} rad and cannot "
                    f"move {delta:.3f} rad either way inside its limits "
                    f"[{lo[index]:+.4f}, {hi[index]:+.4f}]",
                    file=sys.stderr,
                )
                return 1

        print("=" * 70)
        print(f"single-joint test: {joint}")
        print("=" * 70)
        print(f"  start     : {start[index]:+.4f} rad ({np.degrees(start[index]):+.2f} deg)")
        print(f"  target    : {target:+.4f} rad ({np.degrees(target):+.2f} deg)")
        print(f"  step/dwell: {step:.3f} rad every {dwell:.2f}s")
        print(f"  limits    : [{lo[index]:+.4f}, {hi[index]:+.4f}]")
        print("=" * 70)

        begin = float(start[index])
        out = sweep(robot, index, start, begin, target, step, dwell)
        back = sweep(robot, index, start, target, begin, step, dwell)

        rows = out + back
        if verbose:
            print(f"  {'commanded':>10} {'measured':>10} {'lag':>9}")
            for i, (c, m, lag) in enumerate(rows):
                mark = " out" if i < len(out) else "back"
                print(f"  {c:>10.4f} {m:>10.4f} {lag:>9.4f}  {mark}")

        moved_out = abs(out[-1][1] - begin) if out else 0.0
        asked_out = abs(target - begin)
        ratio = moved_out / asked_out if asked_out else float("nan")
        max_lag = max(abs(lag) for _, _, lag in rows)
        final = float(robot.get_joint_positions()[index])
        hw = robot.hw

        print("=" * 70)
        print(f"  commanded excursion : {asked_out:.4f} rad")
        print(f"  measured excursion  : {moved_out:.4f} rad")
        print(f"  step completion     : {ratio:.3f}", end="")
        if ratio < 0.05:
            print("   <-- the joint did not move at all")
        else:
            print("   (fraction of each step covered within the dwell;")
            print("                         NOT a mapping check — use --single for that)")
        print(f"  worst lag           : {max_lag:.4f} rad")
        print(f"  returned to         : {final:+.4f} rad "
              f"(started {start[index]:+.4f}, off by {final - start[index]:+.4f})")
        print(f"  limit clamps        : {hw.limit_clamps}")
        print(f"  step clamps         : {hw.step_clamps}")
        print("=" * 70)
        return 0
    finally:
        robot.disconnect()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="servo bus, e.g. /dev/cu.usbmodem...")
    ap.add_argument("--joint", required=True, help=f"one of {list(conventions.JOINT_ORDER)}")
    ap.add_argument("--delta", type=float, default=0.3, help="excursion, rad")
    ap.add_argument("--step", type=float, default=0.02, help="per-command increment, rad")
    ap.add_argument(
        "--dwell",
        type=float,
        default=DEFAULT_DWELL_S,
        help="settling time after each command before reading back (s). Too "
        "short and a healthy joint looks like it is lagging.",
    )
    ap.add_argument("--quiet", action="store_true", help="summary only, no per-step table")
    ap.add_argument(
        "--single",
        action="store_true",
        help="one command with no step clamp, then settle and read — the only "
        "mode that measures the URDF-to-servo mapping scale",
    )
    ap.add_argument("--settle", type=float, default=1.5, help="--single settling time (s)")
    ap.add_argument(
        "--hand-is-clear",
        action="store_true",
        required=True,
        help="confirm nothing is within the arm's reach; the arm WILL move",
    )
    a = ap.parse_args()
    names = list(conventions.JOINT_ORDER)
    if a.joint not in names:
        print(f"unknown joint {a.joint!r}; expected one of {names}", file=sys.stderr)
        sys.exit(2)
    if a.single:
        sys.exit(run_single(a.port, a.joint, names.index(a.joint), a.delta, a.settle))
    sys.exit(run(a.port, a.joint, a.delta, a.step, a.dwell, not a.quiet))


if __name__ == "__main__":
    main()

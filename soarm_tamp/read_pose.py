#!/usr/bin/env python3
"""Capture the arm's current joint positions for the planner to start from.

RUNS ON THE HOST, where the servos are. Reads only — it never commands a
position, so the arm stays exactly where it is.

    python -m soarm_tamp.read_pose --port /dev/cu.usbmodemXXXX \\
        --out runs/start_pose.json

Then plan from there instead of from the URDF's zero pose:

    ./scripts/hpp_container.sh tcp --start runs/start_pose.json \\
        --xyz 0.22 0.0 0.05 --out runs/tcp03

Why this exists: a plan whose first waypoint is the zero pose assumes the
arm is at the zero pose. It usually is not, and ``execute.py`` streams the
plan regardless — the servos would slew from wherever they are to the
plan's start along a path nothing checked for collisions. Starting the
plan at the measured pose makes that leg part of the planned, validated
trajectory instead of a gap before it.

The output is JSON in the same URDF convention the manifests use, which
is what makes it readable inside the planning container (no SDK there,
just the numbers).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from . import conventions

N_JOINTS = 6

# A read is only believed if the bus has answered this recently.
# ServoHardwareInterface serves a placeholder 2048 ticks per joint until
# its first successful sync-read, so a silent bus otherwise reports a
# perfectly plausible, perfectly wrong pose — the same trap
# validate_calibration.py documents.
MAX_STATE_AGE_S = 0.5
READ_TIMEOUT_S = 5.0


def capture(port: str, settle: float) -> dict:
    from soarm_sdk import load_robot_config
    from soarm_sdk.robot import ServoRobot

    cal = conventions.load_calibration()
    robot = ServoRobot(
        port=port,
        config=load_robot_config("so101"),
        calibration=cal,
        enforce_limits=True,
    )
    robot.connect()
    try:
        deadline = time.time() + READ_TIMEOUT_S
        while robot.state_age() > MAX_STATE_AGE_S:
            if time.time() > deadline:
                raise RuntimeError(
                    f"no servo read in {READ_TIMEOUT_S:.0f}s (state age "
                    f"{robot.state_age():.1f}s) — the bus is not answering. "
                    "Whatever positions it would report are a placeholder, "
                    "not the arm."
                )
            time.sleep(0.05)

        # The bus being alive is settled by state_age() above. This second
        # read answers a different question: is the arm still MOVING? A
        # pose captured mid-motion is stale by the time it is planned
        # from, so the drift is reported rather than hidden.
        first = list(robot.get_joint_positions())
        time.sleep(settle)
        q = list(robot.get_joint_positions())
        moving = max(abs(a - b) for a, b in zip(first, q))
    finally:
        robot.disconnect()

    return {
        "q": [round(float(v), 6) for v in q[:N_JOINTS]],
        "joint_names": list(conventions.JOINT_ORDER),
        "port": port,
        "arm_id": getattr(cal, "arm_id", None),
        "captured": datetime.now(timezone.utc).isoformat(),
        "drift_between_reads_rad": round(float(moving), 6),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="servo bus, e.g. /dev/cu.usbmodem...")
    ap.add_argument("--out", default=None, help="write JSON here as well as stdout")
    ap.add_argument(
        "--settle",
        type=float,
        default=0.3,
        help="gap between the two confirming reads (s)",
    )
    a = ap.parse_args()

    try:
        pose = capture(a.port, a.settle)
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        sys.exit(1)

    import math

    print("=" * 70)
    print("SO-101 measured pose (URDF convention)")
    print("=" * 70)
    for name, v in zip(pose["joint_names"], pose["q"]):
        lo, hi = conventions.URDF_LIMITS[name]
        flag = "" if lo <= v <= hi else "   <-- outside the URDF limit"
        print(f"  {name:<14} {v:+.4f} rad  {math.degrees(v):+8.2f} deg{flag}")
    print(f"  drift between reads: {pose['drift_between_reads_rad']:.6f} rad")
    print("=" * 70)

    if a.out:
        out = Path(a.out)
        if not out.is_absolute():
            out = Path(__file__).parent.parent / out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(pose, indent=1) + "\n")
        print(f"  written: {out}")


if __name__ == "__main__":
    main()

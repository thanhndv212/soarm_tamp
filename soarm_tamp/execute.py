#!/usr/bin/env python3
"""Replay a planned waypoint manifest on the physical SO-101.

RUNS ON THE HOST, not in the planning container: the servos are on the
host's USB bus and this module never imports pyhpp. Its only input is the
manifest directory written by ``soarm_tamp.plan``.

    python -m soarm_tamp.execute runs/cube03 --dry-run
    python -m soarm_tamp.execute runs/cube03 --port /dev/cu.usbmodemXXXX

What it does beyond "send the waypoints"
----------------------------------------
* **Resamples.** The planner samples paths at a fixed dt, so a short edge
  can come back as two waypoints 0.39 rad apart. Streaming that verbatim
  commands a 22-degree step in one tick. Every segment is interpolated so
  no joint moves more than ``--max-step`` per command.
* **Inserts the gripper.** The plan freezes the jaw — the grasp is a rigid
  constraint, not simulated fingers — so opening and closing is not in the
  trajectory at all. It is inserted here at the phase boundaries the
  manifest labels: close after the grasp segments, open after the dock.
* **Refuses an unvalidated calibration.** The URDF-to-servo mapping lives
  in soarm_sdk now (``soarm_sdk.calibration``), and its direction
  signs are assumed until someone checks them on the arm. Streaming a
  planned trajectory against a guess is how a gripper ends up under the
  table, so this exits instead.

Safety is the SDK's job, not this module's: joint limits and a per-step
bound are enforced inside ``ServoHardwareInterface``, so they hold for
every caller rather than only for trajectories that happen to come through
here. The counters are read back at the end of a run.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

from . import conventions
from .geometry import GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG

N_ARM_JOINTS = 6


def _load_segments(run_dir: Path) -> list[dict]:
    manifest = json.loads((run_dir / "manifest.json").read_text())
    out = []
    for rec in manifest.get("segments", []):
        wp = json.loads((run_dir / rec["waypoint_file"]).read_text())["waypoints"]
        out.append(
            {
                "index": rec.get("index"),
                "kind": rec.get("kind", "unknown"),
                "edge": rec.get("edge_name") or "",
                "q": [row[:N_ARM_JOINTS] for row in wp],
            }
        )
    return out


def _resample(qs: list[list[float]], max_step: float) -> list[list[float]]:
    """Linearly interpolate so no joint moves more than *max_step* per tick."""
    if len(qs) < 2:
        return list(qs)
    out = [list(qs[0])]
    for a, b in zip(qs, qs[1:]):
        biggest = max(abs(y - x) for x, y in zip(a, b))
        n = max(1, math.ceil(biggest / max_step))
        for i in range(1, n + 1):
            t = i / n
            out.append([x + (y - x) * t for x, y in zip(a, b)])
    return out


def _gripper_plan(segments: list[dict]) -> dict[int, str]:
    """Map segment index -> jaw action to perform AFTER that segment.

    The plan's own phase structure supplies the timing: the last grasp
    segment before the cube starts moving is where the jaw must close, and
    the last segment before the release edges is where it must open.
    """
    actions: dict[int, str] = {}
    grasp_idx = [i for i, s in enumerate(segments) if s["kind"] == "grasp"]
    release_idx = [i for i, s in enumerate(segments) if s["kind"] == "release"]
    # Segments for phase 1 (pick) end where the dock phase's edges begin.
    dock_start = next(
        (i for i, s in enumerate(segments) if "spot_b" in s["edge"]), None
    )
    if dock_start is not None and dock_start > 0:
        actions[dock_start - 1] = "close"
    elif grasp_idx:
        actions[grasp_idx[0]] = "close"
    if release_idx:
        actions[release_idx[0] - 1 if release_idx[0] > 0 else 0] = "open"
    return actions


def run(
    run_dir: Path,
    port: str | None,
    dry_run: bool,
    rate_hz: float,
    max_step: float,
    force: bool,
) -> int:
    cal = None
    problems: list[str] = ["no calibration loaded"]
    try:
        cal = conventions.load_calibration()
        problems = conventions.check_ready(cal)
    except FileNotFoundError as exc:
        problems = [str(exc)]
    except ImportError:
        problems = ["soarm_sdk is not importable on this machine"]

    print("=" * 70)
    print("SO-101 trajectory replay")
    print("=" * 70)
    print(f"  manifest    : {run_dir}")
    if cal is not None:
        print(f"  calibration : {conventions.calibration_path()}")
        print(f"  arm         : {cal.arm_id}")
        print(f"  validated   : {cal.validated}")
    print(f"  mode        : {'DRY RUN' if dry_run else 'LIVE HARDWARE'}")

    if problems and not dry_run and not force:
        print("\nREFUSING TO RUN:", file=sys.stderr)
        for pr in problems:
            print(f"  - {pr}", file=sys.stderr)
        print(
            "\n  A planned trajectory is open-loop: nothing notices if the\n"
            "  mapping is wrong until the arm has already moved. Validate\n"
            "  first:\n"
            "    python -m soarm_tamp.validate_calibration --port ...\n"
            "  Override at your own risk with --force.",
            file=sys.stderr,
        )
        return 2

    segments = _load_segments(run_dir)
    if not segments:
        print("no segments in manifest", file=sys.stderr)
        return 1

    jaw = _gripper_plan(segments)
    total_raw = sum(len(s["q"]) for s in segments)
    print(f"  segments    : {len(segments)} ({total_raw} planned waypoints)")
    print(f"  jaw actions : {[(i, a) for i, a in sorted(jaw.items())]}")
    print("=" * 70)

    robot = None
    if not dry_run:
        # Imported here, not at module scope: --dry-run must stay runnable
        # anywhere, including in CI and in the planning container, neither
        # of which has a serial stack.
        import numpy as np

        from soarm_sdk.robot import ServoRobot

        # The SDK enforces joint limits and the per-step bound; this module
        # resamples so those clamps should never actually fire. If they do,
        # the counters at the end say so.
        robot = ServoRobot(
            port=port,
            calibration=cal,
            max_step_rad=max_step,
            enforce_limits=True,
        )
        robot.connect()

    period = 1.0 / rate_hz
    n_sent = 0
    try:
        for seg in segments:
            qs = _resample(seg["q"], max_step)
            print(
                f"[{seg['index']:03d}] {seg['kind']:<7} {len(seg['q']):>4} -> "
                f"{len(qs):>5} pts  {seg['edge']}",
                flush=True,
            )
            for q_urdf in qs:
                n_sent += 1
                if robot is not None:
                    robot.set_joint_positions(np.asarray(q_urdf, dtype=float))
                    time.sleep(period)
            action = jaw.get(seg["index"])
            if action:
                deg = GRIPPER_CLOSED_DEG if action == "close" else GRIPPER_OPEN_DEG
                print(f"        jaw -> {action.upper()} ({deg:+.0f} deg)", flush=True)
                if robot is not None:
                    q = list(robot.get_joint_positions())
                    q[5] = math.radians(deg)
                    robot.set_joint_positions(np.asarray(q, dtype=float))
                    time.sleep(0.6)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    finally:
        if robot is not None:
            hw = robot.hw
            print("=" * 70)
            print(f"  limit clamps : {hw.limit_clamps}", end="")
            print("   <-- plan asked for an unreachable pose" if hw.limit_clamps else "")
            print(f"  step clamps  : {hw.step_clamps}", end="")
            print("   <-- arm lagging the plan" if hw.step_clamps else "")
            robot.disconnect()

    print("=" * 70)
    print(f"  commands sent : {n_sent}")
    print(f"  duration      : ~{n_sent * period:.1f}s at {rate_hz:.0f} Hz")
    print("=" * 70)
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="manifest directory written by soarm_tamp.plan")
    ap.add_argument("--port", default=None, help="servo bus, e.g. /dev/cu.usbmodem...")
    ap.add_argument("--dry-run", action="store_true", help="no hardware; print only")
    ap.add_argument("--rate", type=float, default=30.0, help="command rate (Hz)")
    ap.add_argument(
        "--max-step",
        type=float,
        default=0.02,
        help="max per-joint motion per command, rad (default 0.02 ~ 1.1 deg)",
    )
    ap.add_argument(
        "--force", action="store_true", help="run against an unvalidated calibration"
    )
    a = ap.parse_args()
    d = Path(a.run_dir)
    if not d.is_absolute():
        d = Path(__file__).parent.parent / d
    sys.exit(run(d, a.port, a.dry_run, a.rate, a.max_step, a.force))


if __name__ == "__main__":
    main()

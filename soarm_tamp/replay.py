#!/usr/bin/env python3
"""Replay a recorded waypoint manifest in the viser 3-D viewer.

RUNS INSIDE THE PLANNING CONTAINER (needs pyhpp + pyhpp_viser):

    scripts/hpp_container.sh replay --run runs/cube05

Then open the URL it prints. The container uses host networking, so
viser's port is reachable straight from macOS.

This rebuilds the scene but does NOT replan — it loads the manifest
``soarm_tamp.plan`` already wrote and steps the configurations through the
viewer. That is the point: replay is a pure visualization problem, cheap
and repeatable, and it shows exactly the trajectory that
``soarm_tamp.execute`` would send to the servos rather than a fresh plan
that might differ.

It is the sim-side twin of ``execute.py``: same manifest in, one drives
pixels and the other drives servos.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .plan import CubePickPlaceTask, FREEZE_JOINT_SUBSTRINGS

_HERE = Path(__file__).parent

# pyhpp_viser.Viewer.start(port=8000) — see run() for why this is not 8080.
VISER_DEFAULT_PORT = 8000


def _load_waypoints(run_dir: Path) -> list[tuple[dict, list[list[float]]]]:
    manifest = json.loads((run_dir / "manifest.json").read_text())
    out = []
    for rec in manifest.get("segments", []):
        wp = json.loads((run_dir / rec["waypoint_file"]).read_text())["waypoints"]
        out.append((rec, wp))
    return out


def run(run_dir: Path, fps: float, loops: int, hold: float) -> int:
    segments = _load_waypoints(run_dir)
    if not segments:
        print(f"no segments in {run_dir}", file=sys.stderr)
        return 1
    total = sum(len(w) for _, w in segments)

    print("=" * 70)
    print("SO-101 trajectory replay — viser")
    print("=" * 70)
    print(f"  manifest : {run_dir}")
    print(f"  segments : {len(segments)}  ({total} waypoints)")

    task = CubePickPlaceTask(backend="pyhpp", viewer_type="viser")
    task.setup(
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=FREEZE_JOINT_SUBSTRINGS,
        skip_graph=True,
    )
    task.planner.setup_viewer("viser")
    viewer = task.planner.viewer

    # pyhpp_viser.Viewer.start() defaults to port 8000. long_tamp's own log
    # line says 8080, which is simply wrong and sends you to a dead page —
    # it has no `url` attribute to read, so it falls back to a stale
    # constant. Ask the live server instead, and only guess as a last
    # resort.
    url = getattr(viewer, "url", None)
    if not url:
        srv = getattr(viewer, "server", None) or getattr(viewer, "_server", None)
        port = getattr(srv, "port", None) or VISER_DEFAULT_PORT
        url = f"http://localhost:{port}"

    print("=" * 70)
    print(f"  OPEN THIS -> {url}")
    print("=" * 70)
    print("  Ctrl+C to stop.\n", flush=True)

    task.planner.visualize(segments[0][1][0])
    time.sleep(hold)

    dt = 1.0 / fps
    try:
        loop = 0
        while loops == 0 or loop < loops:
            loop += 1
            for rec, wps in segments:
                label = f"[{rec.get('index'):03d}] {rec.get('kind','?'):<7} {rec.get('edge_name') or ''}"
                print(f"  {label}  ({len(wps)} pts)", flush=True)
                for q in wps:
                    task.planner.visualize(q)
                    time.sleep(dt)
                # A beat at each phase boundary: the grasp and release edges
                # are only a couple of waypoints long and otherwise flash by
                # faster than they can be read.
                time.sleep(hold)
            print(f"  --- loop {loop} done ---\n", flush=True)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", default="runs/cube05", help="manifest directory")
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument(
        "--loops", type=int, default=0, help="0 = loop forever (default)"
    )
    ap.add_argument(
        "--hold", type=float, default=0.6, help="pause at each phase boundary (s)"
    )
    a = ap.parse_args()
    d = Path(a.run)
    if not d.is_absolute():
        d = _HERE.parent / d
    sys.exit(run(d, a.fps, a.loops, a.hold))


if __name__ == "__main__":
    main()

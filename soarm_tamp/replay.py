#!/usr/bin/env python3
"""Replay a recorded waypoint manifest in the viser 3-D viewer.

RUNS INSIDE THE PLANNING CONTAINER (needs pyhpp + pyhpp_viser):

    scripts/hpp_container.sh replay --run runs/cube05

Then open the URL it prints. The container uses host networking, so
viser's port is reachable straight from macOS.

Two modes:

* **replay** (default) — step the recorded waypoints at a fixed rate.
* **``--follow``** — mirror the real arm. ``soarm_tamp.execute`` appends
  every command it issues to ``<run>/live.jsonl``; this tails that file
  and shows each one as it lands. Planning and execution still never
  share a process — the run directory is bind-mounted into the container,
  so a file is all they need. Start this first, then run ``execute``:

      scripts/hpp_container.sh replay --run runs/tcp01 --follow   # here
      python -m soarm_tamp.execute runs/tcp01 --port /dev/cu...   # host

  With no arm to hand, ``execute ... --dry-run --pace`` drives the mirror
  at the speed the real run would take.

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

from .execute import N_ARM_JOINTS, TRACE_NAME
from .plan import CubePickPlaceTask, FREEZE_JOINT_SUBSTRINGS

_HERE = Path(__file__).parent

# pyhpp_viser.Viewer.start(port=8000) — see run() for why this is not 8080.
VISER_DEFAULT_PORT = 8000


def _frame_period(segments, fps: float) -> tuple[float, str]:
    """Seconds per waypoint, and how that was decided.

    A time-parameterized plan's waypoints are samples in TIME, dt apart,
    so showing them dt apart replays the motion at the speed it was
    planned to run — accel and decel included. Playing the same rows at an
    arbitrary fps just rescales the whole thing, which is misleading when
    the point of looking is to check the speed profile.
    """
    dts = {
        float(rec.get("dt") or 0)
        for rec, _ in segments
        if rec.get("time_parameterized") and rec.get("dt")
    }
    if len(dts) == 1:
        dt = dts.pop()
        return dt, f"the plan's own timing (dt={dt:.3f}s, {1 / dt:.0f} fps)"
    return 1.0 / fps, f"{fps:.0f} fps (plan carries no timing)"


def _load_waypoints(run_dir: Path) -> list[tuple[dict, list[list[float]]]]:
    manifest = json.loads((run_dir / "manifest.json").read_text())
    out = []
    for rec in manifest.get("segments", []):
        wp = json.loads((run_dir / rec["waypoint_file"]).read_text())["waypoints"]
        out.append((rec, wp))
    return out


def _tail(path: Path, poll: float, idle_timeout: float):
    """Yield batches of trace records as they are appended to *path*.

    Batches, not records: the arm is commanded at 30 Hz and the viewer
    redraws slower than that, so a follower that rendered every line would
    fall further behind the real arm the longer it ran. The caller draws
    only the newest record of each batch, which keeps the picture on the
    arm instead of on its past.

    A line without a trailing newline is a write caught mid-flight; it is
    held back and re-read rather than parsed.
    """
    deadline = time.time() + idle_timeout
    while not path.exists():
        if time.time() > deadline:
            print(f"  no {path.name} appeared in {idle_timeout:.0f}s", flush=True)
            return
        time.sleep(poll)

    with path.open() as fh:
        buf = ""
        while True:
            chunk = fh.read()
            if chunk:
                deadline = time.time() + idle_timeout
                buf += chunk
                *lines, buf = buf.split("\n")
                batch = []
                for line in lines:
                    if not line.strip():
                        continue
                    try:
                        batch.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
                if batch:
                    yield batch
            else:
                if time.time() > deadline:
                    print(f"  nothing for {idle_timeout:.0f}s; stopping", flush=True)
                    return
                time.sleep(poll)


def follow(run_dir: Path, task, base_q: list[float], poll: float, idle: float) -> int:
    """Mirror the live command stream from soarm_tamp.execute.

    The trace carries the six arm joints — that is what goes to the servos
    and nothing else is measured — so the rest of the configuration (the
    table and the cube) is held at the manifest's own starting values.
    The viewer therefore shows the arm as commanded inside the scene it
    was planned in, which is the comparison worth looking at.
    """
    trace = run_dir / TRACE_NAME
    print(f"  following : {trace}")
    print("  waiting for soarm_tamp.execute to start ...", flush=True)

    q = list(base_q)
    shown = 0
    for batch in _tail(trace, poll, idle):
        for rec in batch:
            if rec.get("event") == "start":
                print(f"  execute started (dry_run={rec.get('dry_run')})", flush=True)
            if rec.get("jaw"):
                print(f"        jaw -> {rec['jaw'].upper()}", flush=True)
            if rec.get("event") == "done":
                print(
                    f"  execute finished after {rec.get('commands')} commands "
                    f"({shown} frames mirrored)"
                )
                return 0
        latest = next(
            (r for r in reversed(batch) if isinstance(r.get("q"), list)), None
        )
        if latest is None:
            continue
        q[:N_ARM_JOINTS] = latest["q"][:N_ARM_JOINTS]
        task.planner.visualize(q)
        shown += 1
        if shown % 50 == 0:
            print(f"  ... {latest.get('i')} commands mirrored", flush=True)
    return 0


def run(
    run_dir: Path,
    fps: float,
    loops: int,
    hold: float,
    follow_live: bool = False,
    poll: float = 0.02,
    idle: float = 120.0,
) -> int:
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

    dt, how = _frame_period(segments, fps)
    print(f"  playback : {how}")

    task.planner.visualize(segments[0][1][0])
    time.sleep(hold)

    if follow_live:
        return follow(run_dir, task, segments[0][1][0], poll, idle)

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
    ap.add_argument(
        "--follow",
        action="store_true",
        help=f"mirror the arm live from <run>/{TRACE_NAME} instead of replaying",
    )
    ap.add_argument("--poll", type=float, default=0.02, help="--follow poll period (s)")
    ap.add_argument(
        "--idle",
        type=float,
        default=120.0,
        help="--follow gives up after this long with no new command (s)",
    )
    a = ap.parse_args()
    d = Path(a.run)
    if not d.is_absolute():
        d = _HERE.parent / d
    sys.exit(run(d, a.fps, a.loops, a.hold, a.follow, a.poll, a.idle))


if __name__ == "__main__":
    main()

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
* **Moves continuously.** Three settings decide whether the arm flows or
  stutters, and they are separate jobs that used to be one number:
  ``--max-step`` samples the path (fidelity), ``--servo-clamp`` caps how
  far the commanded target may lead the measured one (which is what gives
  a servo enough position error to drive against), and ``--sync-tol`` caps
  how far the arm may trail the waypoint stream. Keep the tolerance above
  the sampling step and the arm never has to stop: it is always chasing a
  target a little ahead of it, and the lead is bounded either way.
  ``--speed-scale`` sets each servo's GOAL_SPEED from the plan's own
  velocity instead of the SDK default, so consecutive commands blend
  instead of each running its own accel/decel ramp.
* **Waits for the arm.** A planned path is only the path that was
  collision-checked if the joints move together. They do not by default:
  the SDK clamps every command to ``max_step`` of the *measured*
  position, and a servo covers well under one step per tick, so the plan
  runs ahead and each joint falls behind in proportion to how far it has
  to travel. Measured on hardware, that desynchronisation walked the hand
  onto the table on a path whose every waypoint HPP had certified
  collision-free: ``wrist_flex`` (1.36 rad to travel) ran ahead of
  ``elbow_flex`` (0.54 rad), the hand touched down early, and the contact
  stalled two joints for the rest of the run. So each waypoint is now
  held until the arm is actually within ``--sync-tol`` of it. ``--no-sync``
  restores the open-loop behaviour, which is faster and wrong.
* **Traces what it sent.** Every command is appended to
  ``<run>/live.jsonl`` as it goes out. That file is how the 3-D viewer
  mirrors the real arm: ``replay.py --follow`` runs in the planning
  container, this runs on the host, and the run directory is the one
  thing both can see. It is also the record of what the arm was actually
  told to do, sitting next to the plan it came from. ``--no-trace`` turns
  it off.
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
import queue
import sys
import threading
import time
from pathlib import Path

from . import conventions
from .geometry import GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG

N_ARM_JOINTS = 6

# Live command trace, written into the run directory. replay.py --follow
# reads this; see _Trace.
TRACE_NAME = "live.jsonl"


class _Trace:
    """Append each command to ``<run>/live.jsonl`` as it is issued.

    One JSON object per line, flushed immediately — a follower tailing the
    file over a bind mount sees a command within a frame of it being sent,
    and a half-written line is simply the last one, which the reader
    retries rather than parses.

    The write happens on its own thread, off the control loop that calls
    :meth:`write`. This is pure visualization plumbing for a follower on
    the other side of a bind mount, and a bind mount is exactly the kind
    of filesystem that occasionally stalls a write for tens of
    milliseconds (virtiofs/osxfs sync) — long enough, sitting inside the
    per-command loop, to visibly perturb the servo's pacing. ``write()``
    only enqueues, in memory, and returns; the arm's timing can no longer
    depend on how fast the mirror's disk happens to be right now.

    Never allowed to break a run: if the file cannot be opened or a write
    fails, the trace turns itself off and the servos carry on. Losing the
    picture is not a reason to stop the arm mid-trajectory.
    """

    def __init__(self, run_dir: Path, enabled: bool) -> None:
        self.enabled = enabled
        self._thread: "threading.Thread | None" = None
        self._q: "queue.SimpleQueue" = queue.SimpleQueue()
        if not enabled:
            return
        self._path = run_dir / TRACE_NAME
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            fh = self._path.open("w", buffering=1)
        except OSError as exc:
            print(f"  trace       : disabled ({exc})")
            self.enabled = False
            return
        try:
            while True:
                item = self._q.get()
                if item is None:
                    break
                try:
                    fh.write(json.dumps(item) + "\n")
                except OSError as exc:
                    print(f"  trace       : stopped ({exc})")
                    self.enabled = False
                    break
        finally:
            fh.close()

    def write(self, **fields) -> None:
        if not self.enabled or self._thread is None:
            return
        self._q.put(fields)

    def close(self) -> None:
        if self._thread is None:
            return
        self._q.put(None)
        self._thread.join(timeout=2.0)
        self._thread = None


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
                # A time-parameterized segment's waypoints are dt apart in
                # seconds, so their spacing IS the planned velocity profile.
                "timed": bool(rec.get("time_parameterized")),
                "dt": float(rec.get("dt") or manifest.get("sampling", {}).get("dt", 0)),
            }
        )
    return out


def _resample(
    qs: list[list[float]], max_step: float, dt: float = 0.0
) -> tuple[list[list[float]], list[float]]:
    """Interpolate so no joint moves more than *max_step* per command.

    Returns (waypoints, seconds-to-wait-before-each). When *dt* is given —
    the spacing of a time-parameterized plan — each original interval is
    still subdivided for the step bound, but its dt is divided among the
    sub-steps, so the planner's own accel/decel survives: a slow stretch
    keeps its long dwells and a fast one its short ones. Without dt the
    schedule is empty and the caller falls back to a constant rate, which
    replays a ramped trajectory as if it were constant-speed.
    """
    if len(qs) < 2:
        return list(qs), [0.0] * len(qs)
    out = [list(qs[0])]
    when = [0.0]
    for a, b in zip(qs, qs[1:]):
        biggest = max(abs(y - x) for x, y in zip(a, b))
        n = max(1, math.ceil(biggest / max_step))
        for i in range(1, n + 1):
            t = i / n
            out.append([x + (y - x) * t for x, y in zip(a, b)])
            when.append(dt / n if dt else 0.0)
    return out, when



def _robot_config():
    """The SO-101 config, since that is the arm this package plans for.

    ``ServoRobot`` defaults to ``soarm100.yaml``; both ship the same joint
    limits, but being explicit keeps this honest about which revision is on
    the bench (see the SO-100/SO-101 naming trap in the workspace docs).
    """
    from soarm_sdk import load_robot_config

    return load_robot_config("so101")


def _report_reach(segments: list[dict], cal, max_step: float) -> None:
    """Say how many commands the SDK would clamp, without touching hardware.

    The point of a dry run is to find out whether a plan is executable
    *before* an arm is involved, and "would any of this be clamped?" is the
    sharpest form of that question: a clamped command means the arm cannot
    reach where the plan wants it, so the trajectory silently stops being
    the trajectory that was planned.

    Needs soarm_sdk for the limits, which the planning container does not
    have — so this degrades to a note rather than failing the dry run.
    """
    try:
        import numpy as np

        from soarm_sdk.robot import ServoRobot
    except ImportError:
        print("  reach check : skipped (soarm_sdk not importable here)")
        return

    probe = ServoRobot(port="", config=_robot_config(), calibration=cal)
    lo, hi = probe.effective_joint_limits()

    qs: list[list[float]] = []
    for seg in segments:
        qs.extend(_resample(seg["q"], max_step)[0])
    Q = np.asarray(qs)

    # Match the SDK's own rule: an excursion smaller than half an encoder
    # tick cannot change what the servo does, so it is not a clamp.
    from soarm_sdk.robot.hardware import _CLAMP_EPS_RAD

    outside = (Q < lo - _CLAMP_EPS_RAD) | (Q > hi + _CLAMP_EPS_RAD)
    n = int(outside.any(axis=1).sum())
    source = "config ∩ measured travel" if cal is not None else "config only"
    print(f"  reach check : {n} of {len(Q)} commands would be clamped ({source})")
    if n:
        names = probe.joint_names
        per = outside.sum(axis=0)
        worst = ", ".join(
            f"{names[i]}×{int(c)}" for i, c in enumerate(per) if c
        )
        print(f"                ^ {worst}")
        print("                A clamped command means the plan asked for a pose")
        print("                this arm cannot reach — investigate the mapping,")
        print("                do not widen the limit.")


def _await_arrival(robot, target, tol: float, timeout: float):
    """Hold until the arm is within *tol* of *target*, or *timeout* passes.

    Returns (blocked, worst joint error at the end), where *blocked* means
    the arm was NOT already inside the tolerance when first checked — i.e.
    this waypoint made the stream stop and wait. Counting those is how a
    run reports its own smoothness: a flowing motion blocks on nothing and
    still never drifts more than *tol* off the planned path, because the
    stream cannot outrun the arm by more than that.

    The wait is what makes the streamed path the planned path. Without it
    the next waypoint goes out regardless, and the joints drift apart by
    however much each one lags — which is how the hand ended up on the
    table.

    Pacing is deliberately NOT this function's job. It used to take a
    minimum dwell, which double-counted against the caller's own sleep and
    made every command take twice as long as intended; the caller now
    holds a single wall-clock deadline for that.
    """
    import numpy as np

    deadline = time.time() + timeout
    blocked = None
    while True:
        err = float(np.max(np.abs(np.asarray(robot.get_joint_positions()) - target)))
        if blocked is None:
            blocked = err > tol
        if err <= tol or time.time() > deadline:
            return blocked, err
        time.sleep(0.005)


def _settle(robot, target, tol: float, timeout: float):
    """Hold the final waypoint until the arm actually arrives on it.

    Flowing motion is bought by letting the arm trail the stream by up to
    --sync-tol, which is fine mid-path and wrong at the end: the last
    command goes out while the arm is still closing the gap, and whatever
    is left of it is the arm's final error. Measured: a run that tracked
    beautifully finished 10.2 mm from the commanded pose for exactly this
    reason, against 7.0 mm for a stop-at-every-waypoint run that was much
    less smooth.

    So the goal is re-sent until the arm is inside a tighter tolerance —
    at the servo's default speed, NOT a crawl. An earlier version crept at
    0.15 rad/s on the theory that a slow approach would avoid hunting
    around the setpoint, and it left shoulder_lift 0.052 rad short every
    time. Slow is exactly wrong for these servos: a small error commanded
    slowly does not produce enough torque to break stiction, while the
    same 0.05 rad error at default speed closes (measured travel ratio
    1.197 — it slightly overshoots, which is the opposite failure and a
    much more useful one).

    Returns the worst joint error at the end.
    """
    import numpy as np

    deadline = time.time() + timeout
    err = float("inf")
    while time.time() < deadline:
        err = float(np.max(np.abs(np.asarray(robot.get_joint_positions()) - target)))
        if err <= tol:
            return err
        robot.set_joint_positions(target)
        time.sleep(0.05)
    return err


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
    trace_enabled: bool = True,
    pace: bool = False,
    sync: bool = True,
    sync_tol: float = 0.08,
    sync_timeout: float = 2.0,
    servo_clamp: float = 0.1,
    speed_scale: float = 1.5,
    settle_tol: float = 0.05,
    settle_timeout: float = 2.0,
    plan_timing: bool = True,
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

    # A stale manifest is as dangerous as a bad calibration and fails the
    # same way: everything loads, the numbers look reasonable, and the arm
    # drives a path checked against a world that no longer exists.
    problems += conventions.check_scene(run_dir)

    print("=" * 70)
    print("SO-101 trajectory replay")
    print("=" * 70)
    print(f"  manifest    : {run_dir}")
    if cal is not None:
        print(f"  calibration : {conventions.calibration_path()}")
        print(f"  arm         : {cal.arm_id}")
        print(f"  validated   : {cal.validated}")
    print(f"  mode        : {'DRY RUN' if dry_run else 'LIVE HARDWARE'}")

    if cal is not None:
        for warning in conventions.span_warnings(cal):
            print(f"  note        : {warning}")

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
    if trace_enabled:
        print(f"  trace       : {run_dir / TRACE_NAME}")
    print("=" * 70)

    robot = None
    if dry_run:
        _report_reach(segments, cal, max_step)
    else:
        # Imported here, not at module scope: --dry-run must stay runnable
        # anywhere, including in CI and in the planning container, neither
        # of which has a serial stack.
        import numpy as np

        from soarm_sdk.robot import ServoRobot

        # The SDK enforces joint limits and the per-step bound; this module
        # resamples so those clamps should never actually fire. If they do,
        # the counters at the end say so.
        # The SDK's clamp is deliberately NOT max_step. max_step samples the
        # path; this caps the lead of command over measurement. Tying them
        # together caps the lead at the sampling step, which leaves the
        # servos with too little position error to overcome stiction —
        # measured at 0.02 rad, they complete 70% of each step; at 0.10,
        # 92%.
        robot = ServoRobot(
            port=port,
            config=_robot_config(),
            calibration=cal,
            max_step_rad=max(servo_clamp, max_step),
            enforce_limits=True,
        )
        # connect() now refuses (via ServoHardwareInterface's own
        # verify_eeprom_limits check) if the calibration's recorded EEPROM
        # limits disagree with what the servos report today — the same
        # failure mode that let wrist_flex cap 0.41 rad short of what every
        # layer above believed, caught here instead of as a stalled joint
        # mid-plan. Silent (not skipped — silent, by design) for a
        # calibration that never recorded EEPROM limits in the first place.
        try:
            robot.connect()
        except RuntimeError as exc:
            print(f"REFUSING TO RUN: {exc}", file=sys.stderr)
            return 1

        # Do not command anything until the bus has actually answered.
        # ServoHardwareInterface serves a placeholder 2048 ticks per joint
        # until its first successful sync-read, and every command is
        # clamped against the measured position — so streaming into that
        # window clamps the first command around a fake "arm is at zero"
        # reading. Seen live: wrist_roll sitting at -2.71 rad was commanded
        # toward +0.1 for one tick, a 2.8 rad excursion that only the next
        # command (33 ms later, against a real read) took back.
        deadline = time.time() + 5.0
        while robot.state_age() > 0.5:
            if time.time() > deadline:
                print(
                    "REFUSING TO RUN: no servo read in 5s — the bus is not "
                    "answering, and every command would be clamped against a "
                    "placeholder pose rather than the arm.",
                    file=sys.stderr,
                )
                robot.disconnect()
                return 1
            time.sleep(0.05)

        # Belt-and-suspenders with the dashboard's own pre-execute check
        # (soarm_tamp.dashboard.panels._common.PlanControls._within_servo_limits),
        # and the one that actually matters for the bare CLI path used to
        # verify every fix in this document: check the manifest's own
        # waypoints against what the servos will actually obey, not just
        # whether the calibration's recorded EEPROM agrees with them.
        from .conventions import waypoints_beyond_servo_limits

        live_limits = robot.hw.read_angle_limits()
        joint_ids = _robot_config().get("hardware", {}).get("servo_ids", [1, 2, 3, 4, 5, 6])
        worst = waypoints_beyond_servo_limits(
            [q for seg in segments for q in seg["q"]],
            live_limits, cal, joint_ids,
        )
        if worst:
            print("REFUSING TO RUN: the servos will not obey this plan:", file=sys.stderr)
            for name, (over, value, lo, hi) in sorted(
                worst.items(), key=lambda kv: -kv[1][0]
            ):
                print(
                    f"  {name}: plan reaches {value:+.4f}, servo allows "
                    f"{lo:+.4f}..{hi:+.4f} ({over:.4f} rad past it)",
                    file=sys.stderr,
                )
            print(
                "  A goal past a servo's cap is accepted and silently never "
                "acted on. Widen that servo's EEPROM angle limit, or "
                "re-plan against bounds that respect it.",
                file=sys.stderr,
            )
            robot.disconnect()
            return 1

    period = 1.0 / rate_hz
    started = time.time()
    n_sent = 0
    n_waits = 0
    stalled = 0
    settled: list = []
    worst_lag = 0.0
    # The plan freezes the jaw at whatever it measured at the captured start
    # pose, in EVERY waypoint of EVERY segment — grasp is a rigid constraint
    # to HPP, never something it plans a gripper trajectory for. That value
    # is only ever right at t=0. Once a jaw action fires below, this is what
    # actually tracks the intended gripper angle; every per-waypoint target
    # asks for THIS, not whatever the plan still says. Without it, the very
    # first waypoint of the segment after a close re-commands the plan's
    # frozen open value and undoes the close before the arm has moved at
    # all — measured live: closed at segment 1, reopened one command later
    # at the start of segment 2, the jaw never actually holding anything.
    current_gripper_rad = float(segments[0]["q"][0][5]) if segments else 0.0
    trace = _Trace(run_dir, trace_enabled)
    trace.write(
        event="start", segments=len(segments), rate_hz=rate_hz, dry_run=dry_run
    )
    try:
        for seg in segments:
            use_timing = plan_timing and seg.get("timed") and seg.get("dt")
            qs, schedule = _resample(
                seg["q"], max_step, seg["dt"] if use_timing else 0.0
            )
            if use_timing:
                print(
                    f"        replaying the plan's own timing "
                    f"({sum(schedule):.2f}s of trajectory)",
                    flush=True,
                )
            # One wall-clock schedule for the whole segment. Sleeping each
            # dwell in turn accumulates the OS's overshoot on every short
            # sleep -- measured, a 1.35s trajectory took 2.8s that way.
            # Sleeping until an absolute deadline gives the time back on
            # the next command instead of compounding it.
            seg_started = time.time()
            due = 0.0
            print(
                f"[{seg['index']:03d}] {seg['kind']:<7} {len(seg['q']):>4} -> "
                f"{len(qs):>5} pts  {seg['edge']}",
                flush=True,
            )
            for wp_i, q_urdf in enumerate(qs):
                n_sent += 1
                if robot is not None:
                    target = np.asarray(q_urdf, dtype=float)
                    # Override the plan's frozen placeholder with whatever
                    # the jaw is actually meant to be doing right now — see
                    # current_gripper_rad's own comment above.
                    target[5] = current_gripper_rad
                    # Velocity feedforward: the plan's own speed for this
                    # step, so each servo runs at the trajectory's pace
                    # rather than the SDK default and consecutive commands
                    # blend instead of each ramping up and down. Scaled a
                    # little above nominal so a joint that fell behind can
                    # close the gap; the target is a position, so overshoot
                    # is not a risk.
                    dq = None
                    if speed_scale > 0 and wp_i > 0:
                        prev = np.asarray(qs[wp_i - 1], dtype=float)
                        # prev still carries the plan's frozen gripper value;
                        # matching target's override here keeps the gripper
                        # axis out of this step's delta, which is about the
                        # arm's own pace and has nothing to do with the jaw.
                        prev[5] = current_gripper_rad
                        # rad per second for THIS step: the plan's own dwell
                        # when replaying its timing, else the command period.
                        span = schedule[wp_i] if use_timing else period
                        dq = np.abs(target - prev) / max(span, 1e-3) * speed_scale
                        dq = np.maximum(dq, 0.05)
                    robot.set_joint_positions(target, dq=dq)
                    if sync:
                        blocked, lag = _await_arrival(
                            robot, target, sync_tol, sync_timeout
                        )
                        n_waits += blocked
                        if lag > worst_lag:
                            worst_lag = lag
                        if blocked and lag > sync_tol:
                            stalled += 1
                            if stalled == 1:
                                print(
                                    f"        arm is not keeping up: {lag:.3f} rad "
                                    f"behind after {sync_timeout:.1f}s",
                                    flush=True,
                                )
                # q_urdf is the plan's raw row — for the gripper axis, the
                # frozen placeholder, never what target actually asked for
                # once current_gripper_rad has overridden it. Trace what was
                # actually commanded (dry runs never command anything, so
                # q_urdf is still the right thing to show there).
                traced_q = target if robot is not None else np.asarray(q_urdf)
                trace.write(
                    i=n_sent, seg=seg["index"], q=[round(float(v), 6) for v in traced_q]
                )
                # Pace against one wall-clock deadline for the segment: the
                # plan's own dwell for this step when it is time-
                # parameterized, otherwise the fixed command period.
                # Sleeping each dwell in turn instead accumulates the OS's
                # overshoot on every short sleep; sleeping until an absolute
                # deadline gives that time back on the next command.
                #
                # A dry run normally races through at whatever speed the
                # loop manages, which is useless to watch. --pace makes it
                # take as long as the real thing would, so the mirror shows
                # the trajectory at its true speed with no arm connected.
                due += schedule[wp_i] if use_timing else period
                if robot is not None or pace:
                    remaining = seg_started + due - time.time()
                    if remaining > 0:
                        time.sleep(remaining)
            if robot is not None and settle_tol > 0:
                settle_target = np.asarray(qs[-1], dtype=float)
                settle_target[5] = current_gripper_rad
                final_err = _settle(
                    robot,
                    settle_target,
                    settle_tol,
                    settle_timeout,
                )
                settled.append((seg["index"], final_err))
                print(f"        settled to {final_err:.4f} rad of the "
                      f"segment's last waypoint", flush=True)
            action = jaw.get(seg["index"])
            if action:
                deg = GRIPPER_CLOSED_DEG if action == "close" else GRIPPER_OPEN_DEG
                print(f"        jaw -> {action.upper()} ({deg:+.0f} deg)", flush=True)
                trace.write(seg=seg["index"], jaw=action, deg=deg)
                # Every waypoint from here on asks for this instead of the
                # plan's frozen value — otherwise the very first command of
                # the next segment reopens what this just closed.
                current_gripper_rad = math.radians(deg)
                if robot is not None:
                    q = list(robot.get_joint_positions())
                    q[5] = current_gripper_rad
                    jaw_target = np.asarray(q, dtype=float)
                    robot.set_joint_positions(jaw_target)
                    # A flat sleep(0.6) here (no dq, so the SDK's default
                    # ~0.46 rad/s) leaves a real 45-degree swing short of
                    # arrival whenever it needs more than 0.6s — measured:
                    # the residual carried into the next segment's own sync
                    # check as "the gripper is off path", not because
                    # anything moved wrong, only because this never
                    # confirmed the jaw had actually gotten there yet.
                    jaw_err = _settle(robot, jaw_target, settle_tol, settle_timeout)
                    if jaw_err > settle_tol:
                        print(f"        jaw settled to {jaw_err:.4f} rad "
                              f"(target {settle_tol:.3f})", flush=True)
                elif pace:
                    time.sleep(0.6)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    finally:
        trace.write(event="done", commands=n_sent)
        trace.close()
        if robot is not None:
            hw = robot.hw
            print("=" * 70)
            if sync:
                # A lag a hair over the tolerance is the tolerance doing its
                # job, not a failure — the alarm is for a lag big enough to
                # take the arm off the path that was collision-checked.
                adrift = worst_lag > 3 * sync_tol
                print(f"  stopped to wait on  : {n_waits} of {n_sent} waypoints", end="")
                print("   (0 = the arm never had to stop)" if not n_waits else "")
                if settled:
                    worst_settle = max(e for _, e in settled)
                    print(f"  settled within      : {worst_settle:.4f} rad "
                          f"(target {settle_tol:.3f})", end="")
                    # Only shout when the miss is big enough to matter: the
                    # last fraction is the arm's own droop, not a fault.
                    print("   <-- did not settle"
                          if worst_settle > 1.5 * settle_tol else "")
                print(f"  worst lag in flight : {worst_lag:.4f} rad "
                      f"(tolerance {sync_tol:.3f})", end="")
                print("   <-- ARM OFF THE PLANNED PATH" if adrift else "")
                if stalled and adrift:
                    print(f"  timed out waiting   : {stalled} waypoint(s)")
                    print("                         The arm did not follow the "
                          "planned path; where it")
                    print("                         actually went was never "
                          "collision-checked.")
                elif stalled:
                    print(f"  timed out waiting   : {stalled} waypoint(s), all "
                          "within 3x tolerance")
            print(f"  limit clamps : {hw.limit_clamps}", end="")
            print("   <-- plan asked for an unreachable pose" if hw.limit_clamps else "")
            print(f"  step clamps  : {hw.step_clamps}", end="")
            print("   <-- arm lagging the plan" if hw.step_clamps else "")
            robot.disconnect()

    elapsed = time.time() - started
    print("=" * 70)
    print(f"  commands sent : {n_sent}")
    # Wall clock, not n * period: with --sync the run takes as long as the
    # arm takes, which is the number worth knowing.
    if elapsed >= 0.5:
        print(f"  duration      : {elapsed:.1f}s ({n_sent / elapsed:.1f} commands/s)")
    else:
        # An unpaced dry run sends nothing and sleeps for nothing, so its
        # wall clock says nothing about the run. --pace makes it mean
        # something; so does hardware.
        print("  duration      : n/a (nothing was streamed; try --pace)")
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
        help="max per-joint motion per command, rad (default 0.02 ~ 1.1 deg). "
        "With --sync on, arrival is what keeps the arm on the planned path, "
        "so a LARGER step is safer, not smaller: measured on hardware, 0.02 "
        "rad commands sit near the servos' stiction threshold (70% of each "
        "step completed, 28 clamps) while 0.10 rad commands track (92%, 5).",
    )
    ap.add_argument(
        "--force", action="store_true", help="run against an unvalidated calibration"
    )
    ap.add_argument(
        "--no-trace",
        dest="trace",
        action="store_false",
        help=f"do not write <run>/{TRACE_NAME} (replay.py --follow reads it)",
    )
    ap.add_argument(
        "--no-sync",
        dest="sync",
        action="store_false",
        help="do not wait for the arm to reach each waypoint (open loop). "
        "Faster, and lets the joints desynchronise off the planned path.",
    )
    ap.add_argument(
        "--sync-tol",
        type=float,
        default=0.08,
        help="how close a joint must get to a waypoint before the next one "
        "is sent, rad (default 0.08 ~ 4.6 deg). Above --max-step the motion "
        "flows; below it the arm stops at every waypoint.",
    )
    ap.add_argument(
        "--sync-timeout",
        type=float,
        default=2.0,
        help="give up waiting for a waypoint after this long (s)",
    )
    ap.add_argument(
        "--servo-clamp",
        type=float,
        default=0.1,
        help="max lead of the commanded target over the measured position, "
        "rad. Not the same job as --max-step: this is what gives a servo "
        "enough error to drive against (default 0.1)",
    )
    ap.add_argument(
        "--no-plan-timing",
        dest="plan_timing",
        action="store_false",
        help="ignore a time-parameterized plan's own velocity profile and "
        "stream at a constant --rate instead",
    )
    ap.add_argument(
        "--settle-tol",
        type=float,
        default=0.05,
        help="after the last waypoint of a segment, hold until every joint is "
        "this close to it, rad; 0 disables settling. The default is this "
        "arm's measured floor: whichever joint carries the load stops 0.03-"
        "0.05 rad off and stays there (shoulder_lift one way, elbow_flex the "
        "other). That is gravity droop against finite position-control "
        "stiffness, not pacing — re-commanding cannot fix it, because the "
        "servo already believes it has arrived. Tighten it for a better arm.",
    )
    ap.add_argument(
        "--settle-timeout",
        type=float,
        default=2.0,
        help="give up settling after this long (s)",
    )
    ap.add_argument(
        "--speed-scale",
        type=float,
        default=1.5,
        help="velocity feedforward as a multiple of the plan's own speed; "
        "0 disables it and each command runs at the SDK default speed",
    )
    ap.add_argument(
        "--pace",
        action="store_true",
        help="in --dry-run, sleep as if streaming, so the viser mirror runs at "
        "real speed with no arm connected",
    )
    a = ap.parse_args()
    d = Path(a.run_dir)
    if not d.is_absolute():
        d = Path(__file__).parent.parent / d
    sys.exit(
        run(
            d, a.port, a.dry_run, a.rate, a.max_step, a.force, a.trace,
            a.pace, a.sync, a.sync_tol, a.sync_timeout, a.servo_clamp,
            a.speed_scale, a.settle_tol, a.settle_timeout, a.plan_timing,
        )
    )


if __name__ == "__main__":
    main()

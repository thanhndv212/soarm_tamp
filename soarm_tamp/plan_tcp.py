#!/usr/bin/env python3
"""Plan a single move of the SO-101's TCP to a commanded pose.

RUNS INSIDE THE PLANNING CONTAINER ONLY, exactly like ``soarm_tamp.plan``:

    scripts/hpp_container.sh tcp --xyz 0.22 0.0 0.05 --out runs/tcp01

This is the simplest thing the stack can plan: one Cartesian goal for
``gripper_frame_link``, solved to a joint configuration, then a
collision-free path to it planned by ``long_tamp`` itself —
``GraspSequencePlanner.plan_loop()``, the library's API for "move this
arm to that configuration, holding nothing". It writes the same waypoint manifest
``soarm_tamp.plan`` does, so ``replay.py`` and ``execute.py`` consume it
unchanged — with no gripper commands inserted, because the manifest has no
grasp or release segments for ``execute._gripper_plan`` to key on.

Use it to answer "can the arm get its hand *there*, and what does the
motion look like", which is a question worth asking on its own before any
pick-and-place is wired up.

Scene
-----
The full ``cube_pick_place.yaml`` scene is reused as-is — table and cube
included. They are not manipulated here, they are *obstacles*: a reach
plan that ignores the cube sitting at A would be a reach plan you cannot
trust near A. The collision exclusions ``plan.py`` needs are deliberately
NOT applied; nothing here is supposed to touch anything.

What plans the path
-------------------
``plan_loop`` builds a phase graph scoped to the current (empty) set of
held grasps, takes its loop edge — the transition that moves an arm while
its grasp state does not change — and plans it through
``plan_transition_edge``. That is the same machinery ``plan.py`` uses
between grasps, and it brings what a raw ``directPath`` call does not:
projection onto the edge's constraint leaf, the direct-path-first-then-RRT
strategy, the path optimizers configured in the YAML, and time
parameterization. An earlier version of this module planned the reach with
``ps.directPath`` on the bare pyhpp problem instead, which worked for an
unobstructed straight line and had no answer for anything else.

Why five constraints, not six
-----------------------------
``gripper_frame_link`` hangs off ``gripper_link`` through a FIXED joint,
and the ``gripper`` revolute drives the moving jaw on a sibling branch —
so exactly five joints move the TCP. A six-equation pose goal on a 5-DOF
chain is solvable only where the arm happens to be degenerate, so the
default mask is ``1 1 1 1 1 0``: position pinned, approach axis pinned,
rotation *about* that axis free. That is the same mask the cube's handle
carries and for the same reason (see the README). Pass ``--mask 111111``
to demand the full pose and watch it fail almost everywhere.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from pyhpp.core import ConfigProjector

from long_tamp.planning.path_recorder import PathRecorder
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

# _loader is plan.py's already-parsed YAML (bounds, config layout); reusing
# it keeps one source of scene truth and avoids a second parse.
from . import conventions
from .plan import (
    FREEZE_JOINT_SUBSTRINGS,
    CubePickPlaceTask,
    _loader,
    _quaternion_starts,
)

TASK_NAME = "SO-101: TCP pose reach"

# The URDF's own TCP frame, at the fingertips. +Z is the approach
# direction — see geometry.py's frame-conventions docstring.
TCP_FRAME = "so101/gripper_frame_link"

# The arm's gripper, as named in the YAML. plan_loop needs it to know
# which arm it is moving; nothing is grasped with it here.
GRIPPER = "so101/grasp"

# Joints that actually move TCP_FRAME, in config order. `gripper` is
# excluded on purpose (sibling branch, see the module docstring), so it
# keeps its q_init value through IK without needing a LockedJoint.
TCP_JOINTS = [
    "so101/shoulder_pan",
    "so101/shoulder_lift",
    "so101/elbow_flex",
    "so101/wrist_flex",
    "so101/wrist_roll",
]

# Default goal: dead ahead at 0.22 m, 50 mm up, hand pointing straight
# down. Inside the verified top-down annulus (0.10-0.30 m) and clear of
# the cube at A = (0.22, -0.10).
DEFAULT_XYZ = (0.22, 0.0, 0.05)

# rpy (deg) taking the TCP's local +Z (approach) onto world -Z: a 180 deg
# roll about X. Local +X, the jaw-opening direction, stays on world +X.
DEFAULT_RPY_DEG = (180.0, 0.0, 0.0)

DEFAULT_MASK = "111110"

# IK convergence. The projector is created on the first
# addNumericalConstraintsToConfigProjector call and captures these values
# then, so they have to be set before it.
IK_ERROR_THRESHOLD = 1e-5
IK_MAX_ITER = 60

# What counts as "arrived", for the report the run prints. Tighter than
# anything the servos can hold (their encoder step is ~1.5 mrad) — this
# checks the solver, not the arm.
TCP_POS_TOL_M = 1e-4
TCP_ANG_TOL_RAD = 1e-3


def target_pose(xyz, rpy_deg) -> pin.SE3:
    """The commanded TCP pose in the robot base frame."""
    rpy = np.radians(np.asarray(rpy_deg, dtype=float))
    return pin.SE3(pin.rpy.rpyToMatrix(*rpy), np.asarray(xyz, dtype=float))


def _as_xyzquat(M: pin.SE3) -> list[float]:
    q = pin.Quaternion(M.rotation)
    return [*M.translation.tolist(), q.x, q.y, q.z, q.w]


def _parse_mask(text: str) -> list[bool]:
    bits = [c for c in text if c in "01"]
    if len(bits) != 6:
        raise argparse.ArgumentTypeError(
            f"mask must be 6 bits over (x y z rx ry rz), got {text!r}"
        )
    return [c == "1" for c in bits]


def tcp_pose(robot, q) -> pin.SE3:
    """Forward kinematics for TCP_FRAME at *q*."""
    model, data = robot.model(), robot.data()
    q = np.asarray(q, dtype=float)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[model.getFrameId(TCP_FRAME)].copy()


def pose_error(actual: pin.SE3, target: pin.SE3) -> tuple[float, float, float]:
    """(position error m, approach-axis error rad, roll about it rad).

    Split this way because the default mask constrains the first two and
    frees the third: reporting one lumped 6-D residual would hide whether
    a "large" error is the part we asked for or the part we gave away.
    """
    pos = float(np.linalg.norm(actual.translation - target.translation))
    z_a, z_t = actual.rotation[:, 2], target.rotation[:, 2]
    approach = float(np.arccos(np.clip(float(z_a @ z_t), -1.0, 1.0)))
    R_err = target.rotation.T @ actual.rotation
    roll = float(math.atan2(R_err[1, 0], R_err[0, 0]))
    return pos, approach, roll


def apply_start_pose(task, start_file: Path) -> list[float]:
    """Replace the configuration's arm joints with a measured pose.

    ``soarm_tamp.read_pose`` writes the file on the host; this reads it in
    the container. Only the six robot joints move — the table and the cube
    keep the poses the YAML gives them, since nothing measured them.

    The point is that the plan then *starts where the arm is*. A plan that
    begins at the URDF zero pose leaves the servos to slew there first,
    along a path no collision checker ever saw.

    Refuses a pose outside the planning bounds rather than clamping it
    into range: those bounds are the servo's measured travel intersected
    with the URDF's, so a joint outside them is a reading that does not
    describe this arm, and quietly moving it would put the first waypoint
    somewhere the arm is not.
    """
    data = json.loads(start_file.read_text())
    q_measured = [float(v) for v in data["q"]]

    model = task.robot.model()
    bounds = _loader.joint_bounds_class.all_robot_bounds()
    q = list(task.q_init)
    names = data.get("joint_names") or list(conventions.JOINT_ORDER)
    for name, value in zip(names, q_measured):
        joint = f"so101/{name}"
        lo, hi = bounds[joint]
        if not lo <= value <= hi:
            raise RuntimeError(
                f"measured {joint} = {value:+.4f} rad is outside the planning "
                f"bounds [{lo:+.4f}, {hi:+.4f}]. Move the joint back into "
                "range and capture again; refusing to plan from a pose the "
                "arm cannot be in."
            )
        q[model.joints[model.getJointId(joint)].idx_q] = value

    print(f"   start pose: measured on {data.get('port')} at {data.get('captured')}")
    print(f"   q_start   : {np.round(q[:6], 4)}")
    return q


def solve_tcp_ik(
    task, target: pin.SE3, mask: list[bool], attempts: int, seed: int
) -> tuple[list[float] | None, dict]:
    """Find a valid configuration putting the TCP at *target*.

    Seeds the projector with q_init first — when the goal is reachable
    from where the arm stands, that gives the nearest solution rather than
    a random one — then with random arm postures inside the YAML's bounds,
    keeping the best (smallest worst-joint move from q_init) it finds.

    Only the five TCP joints are randomized. Everything else (the jaw, the
    table and cube freeflyers) stays at its q_init value, which is what
    keeps the objects planted without a single LockedJoint: the projector
    moves only DOFs the constraint's Jacobian touches, and this one's
    touches nothing else.

    The projector is built standalone rather than through
    ``Problem.addNumericalConstraintsToConfigProjector``. That binding
    segfaults outright here: it dereferences ``constraints_``, which
    ``pyhpp::core::Problem``'s ptr constructor — the one
    ``pyhpp::manipulation::Problem`` delegates to — never initializes. A
    standalone projector is also the right shape for the job, since it
    leaves the problem itself unconstrained for the path planning below.
    """
    ps, robot = task.ps, task.robot
    q_init = np.asarray(task.q_init, dtype=float)

    constraint = task.planner.create_placement_constraint(
        ps, "tcp_goal", TCP_FRAME, _as_xyzquat(target), mask
    )
    projector = ConfigProjector(robot, "tcp_ik", IK_ERROR_THRESHOLD, IK_MAX_ITER)
    projector.add(constraint, 0)

    idx, lo, hi = arm_index_and_bounds(robot)

    rng = np.random.default_rng(seed)
    stats = {"attempts": 0, "projected": 0, "valid": 0, "last_error": None}
    best_q, best_move = None, math.inf

    for attempt in range(attempts):
        stats["attempts"] = attempt + 1
        q = q_init.copy()
        if attempt > 0:
            q[idx] = rng.uniform(lo, hi)

        # apply() projects in place. Its return value is NOT the test: it
        # reports the solver's own status, which comes back False even for
        # solutions that land on the constraint to 1e-11 (measured — a
        # joint resting on its bound is enough to do it). isSatisfied() is
        # the honest question, and the FK check in run() is the backstop.
        projector.apply(q)
        residual = projector.residualError()
        if not projector.isSatisfied(q):
            stats["last_error"] = f"residual {residual:.2e}"
            continue
        stats["projected"] += 1

        # The projector must not have moved anything but the five TCP
        # joints. If it ever does, the objects have drifted and the plan
        # would be recorded against a scene that is not the real one.
        drift = np.delete(q - q_init, idx)
        if np.max(np.abs(drift)) > 1e-9:
            raise RuntimeError(
                "IK moved a DOF outside the arm "
                f"(max drift {np.max(np.abs(drift)):.2e}) — the scene the "
                "path would be planned in is not the scene q_init describes."
            )

        valid, report = ps.isConfigValid(q)
        if not valid:
            stats["last_error"] = report
            continue
        stats["valid"] += 1

        move = float(np.max(np.abs(q[idx] - q_init[idx])))
        if move < best_move:
            best_q, best_move = q.tolist(), move

    stats["best_move_rad"] = best_move if best_q is not None else None
    return best_q, stats


def arm_index_and_bounds(robot) -> tuple[list[int], np.ndarray, np.ndarray]:
    """Config indices of the five TCP joints, and their planning bounds.

    Bounds come from the YAML, not the URDF: they are the servo-reachable
    intersection (see the joint_groups comment in cube_pick_place.yaml).
    """
    model = robot.model()
    idx = [model.joints[model.getJointId(n)].idx_q for n in TCP_JOINTS]
    bounds = _loader.joint_bounds_class.all_robot_bounds()
    lo = np.array([bounds[n][0] for n in TCP_JOINTS])
    hi = np.array([bounds[n][1] for n in TCP_JOINTS])
    return idx, lo, hi


def plan_reach(
    task, q_init, q_goal, timeout: float, max_iterations: int
) -> tuple[object, str, str]:
    """Plan the reach with long_tamp's own transition planner.

    ``plan_loop`` is the library's API for exactly this: move one arm to a
    given configuration without changing what it is holding. It builds a
    fresh phase graph for the current held-grasps state (empty here), and
    plans that graph's loop edge.

    Going through the graph rather than calling ``ps.directPath`` directly
    is what buys the parts that matter beyond the easy case: the edge's
    steering method and leaf projection, a direct path tried first with a
    sampling-based planner behind it when the straight line is blocked,
    the optimizers from the YAML's ``optimization`` block, and time
    parameterization of the result.

    Returns (path, edge name, how it was planned).
    """
    seq = GraspSequencePlanner(
        graph_builder=task.graph_builder,
        config_gen=task.config_gen,
        planner=task.planner,
        task_config=task.task_config,
        backend=task.backend,
        graph_constraints=getattr(task, "_graph_constraints", None),
        freeze_joint_substrings=FREEZE_JOINT_SUBSTRINGS,
        run_logger=getattr(task, "run_logger", None),
    )
    result = seq.plan_loop(
        gripper=GRIPPER,
        q_current=list(q_init),
        q_target=list(q_goal),
        timeout_per_edge=timeout,
        max_iterations_per_edge=max_iterations,
        verbose=True,
    )
    if not result.get("success"):
        raise RuntimeError(result.get("message", "plan_loop failed"))

    phase = result["phase_results"][0]
    return phase["paths"][0], phase["edges"][0], "long_tamp plan_loop"


def run(
    out_dir: Path,
    xyz,
    rpy_deg,
    mask: list[bool],
    attempts: int,
    seed: int,
    timeout: float = 60.0,
    max_iterations: int = 10000,
    start_file: Path | None = None,
    backend: str = "pyhpp",
    viewer: str = "none",
) -> bool:
    target = target_pose(xyz, rpy_deg)

    print("=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  frame     : {TCP_FRAME}")
    print(f"  position  : {np.asarray(xyz)} m")
    print(f"  rpy       : {np.asarray(rpy_deg)} deg")
    print(f"  mask      : {''.join('1' if b else '0' for b in mask)} (x y z rx ry rz)")
    print(f"  start     : {start_file or 'URDF zero pose (from the YAML)'}")
    print(f"  out       : {out_dir}")
    print("=" * 70)

    print("\n1. Setting up scene ...")
    task = CubePickPlaceTask(backend=backend, viewer_type=viewer)
    task.setup(
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=FREEZE_JOINT_SUBSTRINGS,
        skip_graph=True,
    )
    q_init = task.q_init
    if not q_init:
        print("FAILED: no initial configuration")
        return False
    if start_file is not None:
        try:
            q_init = apply_start_pose(task, start_file)
        except (OSError, KeyError, RuntimeError) as exc:
            print(f"FAILED: {exc}")
            return False
        valid, report = task.ps.isConfigValid(np.asarray(q_init, dtype=float))
        if not valid:
            print(f"FAILED: the arm's measured pose is not a valid start: {report}")
            return False
        # Everything downstream reads task.q_init: the IK seeds, the frozen
        # joints' held values, and the path's first waypoint.
        task.q_init = q_init
    start = tcp_pose(task.robot, q_init)
    print(f"   scene ready, {len(q_init)} DOF")
    print(f"   TCP now  : {np.round(start.translation, 4)}")

    print("\n2. Solving IK for the commanded pose ...")
    q_goal, stats = solve_tcp_ik(task, target, mask, attempts, seed)
    if q_goal is None:
        print(
            f"FAILED: no valid configuration in {stats['attempts']} attempts "
            f"({stats['projected']} projected, 0 collision-free). "
            f"Last: {stats['last_error']}"
        )
        return False
    reached = tcp_pose(task.robot, q_goal)
    pos_err, approach_err, roll = pose_error(reached, target)
    print(
        f"   solved after {stats['attempts']} attempt(s): "
        f"{stats['projected']} projected, {stats['valid']} collision-free"
    )
    print(f"   TCP goal : {np.round(reached.translation, 4)}")
    print(f"   error    : {pos_err * 1e3:.4f} mm, approach {approach_err:.2e} rad")
    print(f"   free roll: {roll:+.3f} rad about the approach axis")
    print(f"   arm moves: {np.round(np.degrees(stats['best_move_rad']), 2)} deg (worst joint)")
    if pos_err > TCP_POS_TOL_M or (mask[3] and approach_err > TCP_ANG_TOL_RAD):
        print("FAILED: IK converged but not onto the commanded pose")
        return False

    print("\n3. Planning the reach ...")
    try:
        path, edge, how = plan_reach(task, q_init, q_goal, timeout, max_iterations)
    except RuntimeError as exc:
        print(f"FAILED: {exc}")
        return False
    resolved = task.planner.get_path(path) if isinstance(path, int) else path
    print(f"   {how} on edge '{edge}'")
    print(f"   path duration {resolved.length():.4f}s "
          f"({resolved.numberPaths()} sub-paths)")

    recorder = PathRecorder(
        output_dir=str(out_dir),
        planner=task.planner,
        dt=0.05,
        quaternion_starts=_quaternion_starts(len(q_init)),
    )
    # Recorded as a transit, not through record_phase_results: that helper
    # labels a phase with no handle a "release", and execute.py keys the
    # jaw commands off exactly that label. This motion grasps and releases
    # nothing, so calling it a release would open the gripper at the end
    # of a pure reach.
    recorder.record_path(
        path,
        kind="transit",
        edge_name=edge,
        extra={
            # Says the waypoints are dt apart in TIME, not in path length:
            # plan_loop time-parameterizes its result, so their spacing is
            # the planned velocity profile and execute.py can replay it
            # rather than flattening it to a constant rate.
            "time_parameterized": True,
            "dt": 0.05,
            "tcp_frame": TCP_FRAME,
            "tcp_target_xyzquat": _as_xyzquat(target),
            "tcp_reached_xyzquat": _as_xyzquat(reached),
            "tcp_mask": [bool(b) for b in mask],
            "tcp_position_error_m": pos_err,
            "tcp_approach_error_rad": approach_err,
            "planner": how,
        },
    )
    summary = recorder.close()

    print("\n" + "=" * 70)
    print("PLANNING SUCCEEDED")
    print(f"  recorded  : {summary}")
    print(f"  manifest  : {out_dir / 'manifest.json'}")
    print("=" * 70)
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="runs/tcp01", help="manifest output directory")
    ap.add_argument(
        "--xyz", nargs=3, type=float, default=list(DEFAULT_XYZ),
        metavar=("X", "Y", "Z"), help="TCP position in the base frame, metres",
    )
    ap.add_argument(
        "--rpy", nargs=3, type=float, default=list(DEFAULT_RPY_DEG),
        metavar=("R", "P", "Y"), help="TCP orientation, degrees (default: hand down)",
    )
    ap.add_argument(
        "--mask", type=_parse_mask, default=_parse_mask(DEFAULT_MASK),
        help="6 bits over (x y z rx ry rz); default 111110 frees the approach roll",
    )
    ap.add_argument(
        "--start",
        default=None,
        help="JSON from soarm_tamp.read_pose: plan from the arm's measured "
        "pose instead of the URDF zero pose",
    )
    ap.add_argument("--attempts", type=int, default=50, help="IK restarts")
    ap.add_argument(
        "--timeout", type=float, default=60.0, help="planner budget for the edge (s)"
    )
    ap.add_argument("--max-iterations", type=int, default=10000)
    ap.add_argument("--seed", type=int, default=0, help="IK random seed")
    ap.add_argument("--backend", default="pyhpp")
    ap.add_argument("--viewer", default="none", help="viser, gepetto, auto, or none")
    args = ap.parse_args()

    out = Path(args.out)
    if not out.is_absolute():
        out = Path(__file__).parent.parent / out
    start = Path(args.start) if args.start else None
    if start is not None and not start.is_absolute():
        start = Path(__file__).parent.parent / start
    sys.exit(
        0
        if run(
            out, args.xyz, args.rpy, args.mask, args.attempts,
            args.seed, args.timeout, args.max_iterations, start,
            args.backend, args.viewer,
        )
        else 1
    )


if __name__ == "__main__":
    main()

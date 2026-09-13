#!/usr/bin/env python3
"""Plan the SO-101 cube pick-and-place and record it as a waypoint manifest.

RUNS INSIDE THE PLANNING CONTAINER ONLY. pyhpp does not exist on macOS;
see scripts/hpp_container.sh, which creates the container this expects and
can run this module for you:

    scripts/hpp_container.sh plan --out runs/cube01

The output is a manifest directory (long_tamp's PathRecorder format: one
JSON per path segment plus manifest.json). That directory is the entire
contract between planning and the robot — soarm_tamp/execute.py reads it
on the host, where the servos are, and never imports pyhpp. Keeping the
handoff a file rather than a live object is what lets the two halves run
in different places.

Sequence, three phases, mirroring the ikea prototype's grasp/dock/release
shape:

    1. so101/grasp -> cube/top       pick the cube up at A
    2. cube/foot   -> table/spot_b   dock it onto the named spot at B
    3. so101/grasp -> (release)      let go; the cube stays on its spot
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from long_tamp.config.yaml_loader import YamlTaskLoader
from long_tamp.planning.path_recorder import PathRecorder
from long_tamp.tasks import ManipulationTask
from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

from . import conventions

TASK_NAME = "SO-101: cube pick and place"

_HERE = Path(__file__).parent
_YAML_PATH = _HERE / "config" / "cube_pick_place.yaml"

GRASP_SEQUENCE: list[tuple[str, str | None]] = [
    ("so101/grasp", "cube/top"),
    ("cube/foot", "table/spot_b"),
    ("so101/grasp", None),
]

# Collision pairs the checker must be blind to, as (robot subtree, object
# subtree). Grasping *requires* the fingers to occupy the same space as the
# cube, so leaving these on makes every grasp pose invalid by construction
# — which is exactly how it failed first time round, rejecting a
# residual-1e-10 solution for "Collision between so101/gripper_link_1 and
# cube/base_link_0".
#
# The exclusion is deliberately rooted at wrist_roll, whose subtree is
# exactly the hand (gripper_link + the moving jaw). The arm proper still
# gets checked against the cube, and the hand still gets checked against
# the table — only the contact that a grasp is made of is waived.
COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("so101/wrist_roll", "cube/root_joint"),
]

# Locked at their q_init value for the whole plan. Mirrors the YAML's
# freeze_joints; see that key's comment for why each one is here.
FREEZE_JOINT_SUBSTRINGS: list[str] = ["table/root_joint", "so101/gripper"]

_loader = YamlTaskLoader(_YAML_PATH)


class CubePickPlaceTask(ManipulationTask):
    """Single-arm SO-101 pick-and-place, driven by the YAML config."""

    FREEZE_JOINT_SUBSTRINGS = FREEZE_JOINT_SUBSTRINGS

    def __init__(self, backend: str = "pyhpp", viewer_type: str = "auto"):
        super().__init__(
            task_name=TASK_NAME,
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
            viewer_type=viewer_type,
        )
        # Deliberately NOT narrowed with with_grasp_goals(): that filter
        # keeps only objects whose *handle* appears in a goal string, and
        # `table` is only ever the handle side of the dock, never a grasp
        # target. Narrowing would drop it from OBJECTS and take spot_b out
        # of the graph with it, leaving the dock phase unsatisfiable. Same
        # trap the ikea prototype documents.
        self.task_config = _loader.task_config
        self.use_factory = True

    def build_initial_config(self) -> list[float]:
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)


def _quaternion_starts(config_size: int) -> list[int]:
    """Config indices where each object's unit quaternion begins.

    Layout is 6 arm joints, then one 7-wide XYZQUAT block per object in
    the YAML's declaration order (table, cube). The recorder needs these
    to tell a real discontinuity from a quaternion double-cover sign flip.
    """
    n_joints = sum(len(v) for v in _loader._data.get("joint_groups", {}).values())
    n_objects = len(_loader.task_config.OBJECTS)
    expected = n_joints + 7 * n_objects
    if config_size != expected:
        raise RuntimeError(
            f"config size {config_size} != expected {expected} "
            f"({n_joints} joints + {n_objects} objects x 7). The quaternion "
            "offsets below would be wrong, so refusing to record."
        )
    return [n_joints + 7 * i + 3 for i in range(n_objects)]


def run(
    out_dir: Path,
    backend: str = "pyhpp",
    viewer: str = "auto",
    start_file: Path | None = None,
) -> bool:
    task = CubePickPlaceTask(backend=backend, viewer_type=viewer)

    print("=" * 70)
    print(TASK_NAME)
    print("=" * 70)
    print(f"  config    : {_YAML_PATH.name}")
    print(f"  objects   : {task.task_config.OBJECTS}")
    print(f"  grippers  : {task.task_config.GRIPPERS}")
    print(f"  frozen    : {FREEZE_JOINT_SUBSTRINGS}")
    print(f"  start     : {start_file or 'URDF zero pose (from the YAML)'}")
    print(f"  out       : {out_dir}")
    print("=" * 70)

    print("\n1. Setting up scene ...")
    task.setup(
        validation_step=task.task_config.PATH_VALIDATION_STEP,
        projector_step=task.task_config.PATH_PROJECTOR_STEP,
        freeze_joint_substrings=FREEZE_JOINT_SUBSTRINGS,
        skip_graph=True,
    )
    for robot_sub, obj_sub in COLLISION_EXCLUSIONS:
        task.scene_builder.disable_collisions_between_subtrees(
            robot_frame_or_joint=robot_sub, obstacle_root_joint=obj_sub
        )

    q_init = task.q_init
    if not q_init:
        print("FAILED: no initial configuration")
        return False

    if start_file is not None:
        # Imported here, not at module scope: plan_tcp imports _loader and
        # _quaternion_starts from this module, so a top-level import either
        # way round is a cycle.
        from .plan_tcp import apply_start_pose

        try:
            q_init = apply_start_pose(task, start_file)
        except RuntimeError as exc:
            # An out-of-bounds start is an ordinary outcome here, not a bug:
            # the arm's measured travel is wider than the URDF's limits, so a
            # joint resting past them is exactly what a slumped arm looks
            # like. Report it the way every other failure in this file is.
            print(f"\nFAILED: {exc}")
            return False

    print(f"   scene ready, {len(q_init)} DOF")

    print("\n2. Planning ...")
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
    recorder = PathRecorder(
        output_dir=str(out_dir),
        planner=task.planner,
        dt=0.05,
        quaternion_starts=_quaternion_starts(len(q_init)),
    )

    result = seq.plan_sequence(
        grasp_sequence=GRASP_SEQUENCE, q_init=q_init, verbose=True
    )

    # Record unconditionally, success or not: a phase that failed part-way
    # still moved the robot, and those segments are what a resume would
    # build on. See PathRecorder.record_phase_results' own docstring.
    recorder.record_phase_results(seq.phase_results)
    summary = recorder.close()
    conventions.write_scene(out_dir)

    print("\n" + "=" * 70)
    if result.get("success"):
        print("PLANNING SUCCEEDED")
    else:
        print(f"PLANNING FAILED: {result.get('error', 'unknown')}")
    print(f"  recorded  : {summary}")
    print(f"  manifest  : {out_dir / 'manifest.json'}")
    print("=" * 70)
    return bool(result.get("success"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="runs/cube01", help="manifest output directory")
    ap.add_argument("--backend", default="pyhpp")
    ap.add_argument("--viewer", default="auto", help="viser, gepetto, auto, or none")
    ap.add_argument(
        "--start",
        type=Path,
        default=None,
        help="JSON pose from soarm_tamp.read_pose: plan from where the arm "
        "actually is. Without it the plan begins at the YAML's zero pose, and "
        "the servos slew there first along a path nothing collision-checked.",
    )
    args = ap.parse_args()
    out = Path(args.out)
    if not out.is_absolute():
        out = _HERE.parent / out
    start = args.start
    if start is not None and not start.is_absolute():
        start = _HERE.parent / start
    sys.exit(0 if run(out, args.backend, args.viewer, start) else 1)


if __name__ == "__main__":
    main()

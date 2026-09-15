# Writing a new task

A task for this pipeline is two pieces:

1. A YAML config (`long_tamp.config.yaml_loader` format) describing the
   scene: objects, their grasp/dock handles, the grippers involved, and
   which joints to freeze during planning.
2. A small Python module that subclasses `long_tamp.tasks.ManipulationTask`,
   points a `YamlTaskLoader` at that config, and defines a
   `GRASP_SEQUENCE` — the ordered list of `(handle, target)` pairs the
   phase graph should satisfy, `target=None` meaning release.

`plan.py` (`CubePickPlaceTask`, driven by `config/cube_pick_place.yaml`) is
the reference implementation of that shape — copy it as the starting point
for a new task. `plan_tcp.py` is the degenerate case: no grasp sequence at
all, just a single joint-configuration goal, useful as a task-independent
smoke test of the pipeline regardless of what you're building — which is
also why it's the right place to start reading this.

Everything downstream of a manifest — `replay.py`, `execute.py`, the
dashboard, calibration, and the execution-tuning knobs — is task-agnostic:
it reads a `runs/<name>/` directory and doesn't know or care what produced
it. Write your task, get a manifest into `runs/`, and the rest of this
document applies unchanged.

## Step 1: confirm the pipeline with a TCP reach

Before authoring a task, `plan_tcp.py` proves the whole pipeline —
container, planner, manifest handoff, host execution — end to end with no
task-specific setup at all. It's the simplest thing the stack can plan: one
Cartesian goal for `gripper_frame_link`, solved to a joint configuration,
then a path to it planned by `long_tamp` itself —
`GraspSequencePlanner.plan_loop()`, the library's API for "move this arm to
that configuration, holding nothing".

```bash
./scripts/hpp_container.sh tcp --out runs/tcp01 \
    --xyz 0.22 0.0 0.05 --rpy 180 0 0
```

The default `--rpy 180 0 0` points the hand straight down. The default
`--mask 111110` frees rotation about the approach axis (see [5-DOF grasp
constraint](#5-dof-grasp-constraint) below): exactly five joints move the
TCP, since `gripper_frame_link` hangs off `gripper_link` by a FIXED joint
and the `gripper` revolute drives the jaw on a sibling branch. Five
constraints on five joints is a square problem; six is solvable only where
the arm is degenerate.

Going through the planning graph (rather than a raw straight-line path)
buys projection onto the edge's constraint leaf, a sampling planner as
fallback, the optimizers from the config's `optimization` block, and time
parameterization — visible in the manifest, same motion planned both ways:

| | waypoints | per-step joint delta (rad) |
|---|---|---|
| raw `ps.directPath` | 8 over 0.3146 **rad** | 0.157 × 7 — constant speed, instant start/stop |
| `plan_loop` | 28 over 1.3477 **s** | 0.000 → 0.114 → 0.000, a symmetric ramp |

A time-parameterized segment is marked `time_parameterized` in the
manifest; `execute.py` replays the waypoint spacing as *time* rather than
flattening it to a constant rate (`--no-plan-timing` opts out). Measured on
the same plan: 1.4 s replaying the profile, 2.5 s flattened.

**Coverage is 17 of 20**, measured on top-down targets across the reachable
annulus: IK solves all 20; both the straight-line route and `plan_loop`
plan 17. The three failures cluster at r ≈ 0.11–0.12 m, where the IK
solution folds the arm in tight and `plan_loop` hits a self-collision
(`shoulder_link` vs `gripper_link`/`moving_jaw`) on every retry — not a
missing fallback, since `computePath` *is* the sampling planner and the
goal configurations themselves pass `isConfigValid`.

## Watching the real arm in the viewer

`execute.py` appends every command it issues to `<run>/live.jsonl`, and
`replay.py --follow` tails that file and renders it — planning and
execution still never share a process; the file is the whole channel.

```bash
./scripts/hpp_container.sh replay --run runs/tcp01 --follow   # container
python -m soarm_tamp.execute runs/tcp01 --port /dev/cu...     # host
```

Start the follower first; it waits for the trace to appear and stops when
the run reports itself done, rendering only the newest command of each
batch so the picture stays on the arm rather than drifting behind it. With
no arm to hand, `--dry-run --pace` streams nothing to the servos but takes
as long as the real run would, driving the mirror at true speed.
`--no-trace` turns the file off; it is otherwise always written, and is the
record of what the arm was actually told to do next relative to the plan
it came from.

## Step 2: build your task

Once the pipeline is confirmed, write the two pieces:

**The YAML config.** Follows `config/cube_pick_place.yaml`'s shape: object
definitions, each with a URDF/SRDF pair and a handle (a grasp or dock
frame, with a `<mask>` — see [5-DOF grasp constraint](#5-dof-grasp-constraint)
below), the grippers involved, and `freeze_joints` for anything that should
stay locked at its initial value for the whole plan (mirrors
`FREEZE_JOINT_SUBSTRINGS` on the task class).

**The task module.** Subclass `long_tamp.tasks.ManipulationTask`:

```python
from long_tamp.config.yaml_loader import YamlTaskLoader
from long_tamp.tasks import ManipulationTask

_YAML_PATH = _HERE / "config" / "my_task.yaml"
_loader = YamlTaskLoader(_YAML_PATH)

class MyTask(ManipulationTask):
    FREEZE_JOINT_SUBSTRINGS = [...]

    def __init__(self, backend="pyhpp", viewer_type="auto"):
        super().__init__(
            task_name="my task",
            backend=backend,
            FILE_PATHS=_loader.file_paths,
            joint_bounds=_loader.joint_bounds_class,
            viewer_type=viewer_type,
        )
        self.task_config = _loader.task_config
        self.use_factory = True

    def build_initial_config(self) -> list[float]:
        return _loader.build_initial_config(objects=self.task_config.OBJECTS)
```

Then define `GRASP_SEQUENCE`, the phases the plan must satisfy in order —
`plan.py`'s is a good template:

```python
GRASP_SEQUENCE: list[tuple[str, str | None]] = [
    ("so101/grasp", "cube/top"),      # pick up at A
    ("cube/foot",   "table/spot_b"),  # dock at B
    ("so101/grasp", None),            # release
]
```

Don't narrow the loaded objects with `with_grasp_goals()`-style filtering
unless every object you need in the graph is also a grasp target — `plan.py`
keeps `table` in `OBJECTS` for exactly this reason: it's only ever the
handle side of a dock, never grasped, and filtering it out would take
`spot_b` out of the graph with it. Watch collision exclusions too: grasping
*requires* the fingers to occupy the same space as the object, so every
object your gripper closes on needs an entry in `COLLISION_EXCLUSIONS`
(rooted at the hand's own subtree, e.g. `wrist_roll`, not the whole arm) —
otherwise every grasp pose is invalid by construction.

**Invoking it.** `plan.py` and `plan_tcp.py` each have a named subcommand
in `scripts/hpp_container.sh` (`plan`, `tcp`) that runs their module inside
the container. There's no generic `--config` flag on a shared entry
point — a new task is a new `plan_<name>.py` module, invoked one of two
ways:

```bash
# while the task is still new: the generic escape hatch, no script changes
./scripts/hpp_container.sh exec python3 -u -m soarm_tamp.plan_mytask --out runs/mytask01

# once it's stable: a named subcommand, matching plan/tcp's shape
#   scripts/hpp_container.sh:
#     mytask) ensure; shift
#             docker exec -i "$NAME" bash -c \
#               "${ENVSETUP}; cd ${CHOME}/devel/soarm-ws/soarm_tamp && python3 -u -m soarm_tamp.plan_mytask $*" ;;
./scripts/hpp_container.sh mytask --out runs/mytask01
```

Output is the same manifest format `plan.py` and `plan_tcp.py` write —
`replay.py`, `execute.py`, and the dashboard take it unchanged.

## 5-DOF grasp constraint

The SO-101 has five arm joints. A full 6-DOF grasp constraint is six
equations, so it is solvable only where the system happens to be
degenerate. A handle should therefore free one rotational DOF — the
reference example's cube handle carries `<mask>1 1 1 1 1 0</mask>`, freeing
rotation about the approach axis, the DOF a parallel-jaw grip does not care
about for a symmetric object. A 600k-sample sweep found ~170° of usable yaw
at every candidate grasp spot with this mask, so it costs no reachability.

## Reachable workspace

At x = 0.22 m with the hand vertical, IK solves for TCP z from 0.03 to
0.08 m and finds nothing at 0.10 m or above; tilting the approach 30° off
vertical finds nothing at any of those heights at that radius. On a 5-DOF
arm the achievable wrist orientation is coupled to position, so "move it
higher for safety" runs out quickly — 0.08 m is the ceiling for a top-down
reach at that radius. The verified top-down-reachable annulus (hand within
10° of vertical) is radius 0.10–0.30 m from the base axis. Keep new task
geometry inside it, or expect `plan_loop` failures like the 3-of-20 gap
noted above.

## Execution tuning

Streaming a HPP-certified path to the servos open-loop is not enough on its
own — waypoints can outrun the arm, or land too close together to clear
stiction. The knobs that fix this:

| knob | job | default |
|---|---|---|
| `--max-step` | how finely the path is sampled (fidelity) | 0.02 |
| `--servo-clamp` | how far the command may *lead* the measurement | 0.10 |
| `--sync-tol` | how far the arm may trail the stream | 0.08 |
| `--speed-scale` | GOAL_SPEED as a multiple of the plan's own velocity | 1.5 |

Keep `--sync-tol` above `--max-step`: the arm then always chases a target a
little ahead of it instead of stopping at every waypoint, and the lead
keeps the servos above their stiction threshold. A last-centimetre gap
(0.03–0.05 rad) under load is gravity droop against finite position-control
stiffness, not a tuning error — `--settle-tol` defaults to 0.05 to reflect
that floor.

`joint_test.py --single` isolates one joint to check its URDF-to-servo
mapping without the step clamp or path fidelity questions.

The full incident this tuning came from — a first run that drove the hand
into the table, the measurements behind each knob above, and why
"settle slowly" makes it worse, not better — is in
[`docs/execution-tuning.md`](execution-tuning.md).

## Planning from where the arm actually is

```bash
python -m soarm_tamp.read_pose --port /dev/cu.usbmodemXXXX \
    --out runs/start.json                                    # host
./scripts/hpp_container.sh tcp --start runs/start.json \
    --xyz 0.22 0.0 0.08 --out runs/tcp06                     # container
```

A plan whose first waypoint is the URDF zero pose assumes the arm is at the
zero pose. It is not, and `execute.py` streams the plan regardless — the
servos slew to the plan's start along a path nothing checked. With
`--start`, that leg is part of the validated trajectory instead of a gap
before it. `plan.py` takes the same `--start` flag.

## Adding a dashboard tab for the new task

The dashboard's plan/replay/execute wiring is shared, not per-tab: every
planning tab is a `Panel` (from `soarm_sdk.dashboard.app`) built around one
`PlanControls` instance (`soarm_tamp/dashboard/panels/_common.py`), which
wraps `capture_start`, `plan`, `play`/`stop_play`, `replay`/`stop_replay`,
and `execute` — the calibration-sync and servo-limit checks, the container
job, and the manifest player all live there once, not per task.
`tcp.py`/`pickplace.py` are the reference panels; a new one follows the
same shape:

```python
"""<Task name> tab."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from soarm_sdk.dashboard.app import Panel

from ._common import Console, PlanControls

RUN_DIR = "runs/dash_mytask"
START_FILE = "runs/dash_start.json"


def build_mytask_panel(fk_update=None, fk_update_ghost=None) -> Panel:
    def _build(server: Any, ctx: Any) -> None:
        _build_mytask(server, ctx, fk_update, fk_update_ghost)

    return Panel("My Task", _build)


def _build_mytask(server, ctx, fk_update, fk_update_ghost=None) -> None:
    root = Path(__file__).resolve().parents[3]

    with server.gui.add_folder("Goal"):
        ...  # your task's own goal controls (sliders, dropdowns, ...)

    with server.gui.add_folder("Plan"):
        plan_btn = server.gui.add_button("Plan", color="green")

    log_md = server.gui.add_markdown("")
    console = Console(log_md)
    console.clear()
    ctrl = PlanControls(
        server, ctx, console, run_dir=root / RUN_DIR,
        fk_update=fk_update, fk_update_ghost=fk_update_ghost,
    )

    @plan_btn.on_click
    def _plan(_: Any) -> None:
        # First element matches the hpp_container.sh subcommand for your
        # plan script (the "exec python3 -m soarm_tamp.plan_mytask" form
        # works here too, once wrapped in the same arg-list shape).
        args = ["mytask", "--out", RUN_DIR, "--viewer", "none"]
        ctrl.plan(args, label="planning mytask")
```

`fk_update` drives the live 3-D mirror; `fk_update_ghost` poses the
translucent preview mesh during `play()` without fighting the live arm for
the same pixels — both come from the `DashboardApp` the panel is registered
on, not from the panel itself.

**Registering it.** `soarm_tamp/dashboard/app.py` builds its one profile
(`plan_and_run`) from `_register_plan_and_run(app)`. Add a line there to
put the new tab alongside TCP Plan and Pick & Place:

```python
from .panels.mytask import build_mytask_panel

app.register(
    build_mytask_panel(fk_update=app.fk_update, fk_update_ghost=app.fk_update_ghost)
)
```

If the new task should be its own dashboard instead — a separate console
script rather than one more tab — add an entry to `profiles()` in the same
module, following `DashboardProfile`'s shape (`name`, `title`, `register`,
`description`); `soarm_sdk.cli.dashboard.PROFILES` is the pattern this
mirrors on the `soarm_sdk` side.

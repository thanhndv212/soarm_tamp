# soarm_tamp

Long-horizon TAMP planning (`long_tamp` on HPP) driving a **physical
SO-101** arm. First task: pick a 25 mm cube from A and place it at B.

Planning and execution deliberately never share a process. `pyhpp` exists
only inside the HPP container; the servos exist only on the host. The
contract between them is a **waypoint manifest on disk**:

```
  plan (container, pyhpp)  ->  runs/<name>/manifest.json  ->  execute (host, soarm_sdk)
```

## Status

> **Viser runs on port 8000**, not the 8080 that long_tamp's own
> "Viser viewer started" log line prints — that message falls back to a
> stale constant. `replay.py` reports the real one.

| Stage | State |
|---|---|
| Scene, grasp semantics, masked 5-DOF handle | working |
| Planning: 3 phases, 6 segments, 0 seam violations | working |
| Trajectory within servo-reachable bounds | verified, 0 clamped |
| TCP-pose reach (`plan_tcp.py`) | working; 0.0000 mm goal error, 0 clamped |
| Live viser mirror of a run (`replay.py --follow`) | working; verified against a paced dry run |
| **Run on hardware** | **done** — TCP within 7.0 mm / 2.2° of the commanded pose |
| Joint sign calibration | validated on the arm |

Everything here has now been run on the physical SO-101. The first live
attempt drove the hand into the table; what that cost and what it taught
is in *Streaming a trajectory the arm can actually follow* below — read
it before the first run on a new arm.

## Running it

```bash
# 0. the simplest thing this can plan: move the TCP to a pose
./scripts/hpp_container.sh tcp --out runs/tcp01 --xyz 0.22 0.0 0.05

# 1. plan the pick-and-place (creates/starts the container as needed)
./scripts/hpp_container.sh plan --out runs/cube01 --viewer none

# 2. watch it in the 3-D viewer, then open http://localhost:8000
./scripts/hpp_container.sh replay --run runs/cube01

# 3. check what would be sent, no hardware needed
python -m soarm_tamp.execute runs/cube01 --dry-run

# 4a. ONCE per arm: seed a calibration from measured travel (offline)
soarm-seed-calibration \
  --lerobot ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/thanh_arm.json \
  --arm-id thanh_arm

# 4b. ONCE per arm: confirm it on the arm (torque off first, then small moves)
python -m soarm_tamp.validate_calibration --port /dev/cu.usbmodemXXXX

# 5. run it
python -m soarm_tamp.execute runs/cube01 --port /dev/cu.usbmodemXXXX

# 5b. ... and watch the arm in the 3-D viewer while it runs (see below)
./scripts/hpp_container.sh replay --run runs/cube01 --follow     # start first
python -m soarm_tamp.execute runs/cube01 --port /dev/cu.usbmodemXXXX
```

Regenerate the scene assets after changing `geometry.py`:

```bash
python -m soarm_tamp.build_assets
```

## Reaching a TCP pose

`plan_tcp.py` is the simplest thing the stack can plan: one Cartesian
goal for `gripper_frame_link`, solved to a joint configuration, then a
path to it planned by **`long_tamp` itself** —
`GraspSequencePlanner.plan_loop()`, the library's API for "move this arm
to that configuration, holding nothing". It builds a phase graph scoped
to the current (empty) set of held grasps and plans that graph's loop
edge (`Loop | f`) through `plan_transition_edge`.

Going through the graph is what buys the machinery the library exists
for: projection onto the edge's constraint leaf, a direct path tried
first with a sampling planner behind it, the optimizers from the YAML's
`optimization` block, and **time parameterization**. The difference is
visible in the manifest — the same motion, planned both ways:

| | waypoints | per-step joint delta (rad) |
|---|---|---|
| raw `ps.directPath` | 8 over 0.3146 **rad** | 0.157 × 7 — constant speed, instant start and stop |
| `plan_loop` | 28 over 1.3477 **s** | 0.000 → 0.114 → 0.000, a symmetric ramp |

A time-parameterized segment is marked `time_parameterized` in the
manifest, and `execute.py` replays the waypoint spacing as *time* rather
than flattening it to a constant rate (`--no-plan-timing` opts out).
Measured on the same plan: 1.4 s replaying the profile, 2.5 s flattened.

Output is the same manifest `plan.py` writes, so `replay.py` and
`execute.py` take it unchanged — with no jaw commands, because the
segment is recorded as a transit. (Recording it through
`record_phase_results` would label it a *release*, since its phase has no
handle, and `execute.py` keys the jaw off exactly that label.)

```bash
./scripts/hpp_container.sh tcp --out runs/tcp01 \
    --xyz 0.22 0.0 0.05 --rpy 180 0 0
```

The default `--rpy 180 0 0` points the hand straight down. The default
`--mask 111110` frees rotation about the approach axis, for the same
reason the cube's handle does (see *Why a 5-DOF mask*): exactly five
joints move the TCP, since `gripper_frame_link` hangs off `gripper_link`
by a FIXED joint and the `gripper` revolute drives the jaw on a sibling
branch. Five constraints on five joints is a square problem; six is
solvable only where the arm is degenerate.

**Coverage is 17 of 20**, measured on top-down targets across the
reachable annulus: IK solves all 20, and both the old straight-line route
and `plan_loop` plan 17 of them. Going through the library did not extend
that — the same three fail, all at r ≈ 0.11–0.12 m where the IK solution
folds the arm in tight, and `plan_loop` fails inside
`TransitionPlanner.computePath` on a self-collision (`shoulder_link` vs
`gripper_link` / `moving_jaw`) on all 10 of its retries. Worth digging
into: `computePath` *is* the sampling planner, so this is not a missing
fallback, and the goal configurations themselves pass `isConfigValid`.

## Watching the real arm in the viewer

`execute.py` appends every command it issues to `<run>/live.jsonl`, and
`replay.py --follow` tails that file and shows it. Planning and execution
still never share a process: the container bind-mounts this workspace, so
a file is the whole channel — the same shape as the manifest handoff.

```bash
./scripts/hpp_container.sh replay --run runs/tcp01 --follow   # container
python -m soarm_tamp.execute runs/tcp01 --port /dev/cu...     # host
```

Start the follower first; it waits for the trace to appear and stops when
the run reports itself done. It renders only the newest command of each
batch it reads, so the picture stays on the arm rather than drifting
behind it. With no arm to hand, `--dry-run --pace` streams nothing to the
servos but takes as long as the real run would, which drives the mirror
at true speed. `--no-trace` turns the file off; it is otherwise always
written, and is the record of what the arm was actually told to do next
to the plan it came from.

## Streaming a trajectory the arm can actually follow

The first live run put the hand on the table along a path whose every
waypoint HPP had certified collision-free. Nothing was wrong with the
plan. What was wrong was the assumption that streaming waypoints makes
the arm follow them.

**What happened.** `execute.py` sent a waypoint every tick regardless of
where the arm was, and the SDK clamps each command to `--max-step` of the
*measured* position. A servo covers well under one step per tick, so the
arm fell behind on command one and never recovered, each joint by a
different amount — `wrist_flex` had 1.36 rad to travel, `elbow_flex` 0.54.
They desynchronised, the arm stopped tracing the straight line that had
been collision-checked, the hand touched down early, and the contact
stalled two joints for the rest of the run. Final state: two joints
0.5–2.6 rad short of their goals, hand on the table.

**Two independent causes, both now addressed.**

1. *Nothing waited for the arm.* `--sync` (on by default) holds each
   waypoint until every joint is within `--sync-tol` of it. Lag is then
   bounded by the tolerance instead of growing without limit, and a run
   that genuinely cannot keep up says so — `worst lag` and
   `timed out waiting` in the summary — rather than discovering it by
   contact.
2. *The steps were too small to move the servos.* Measured on
   `wrist_roll`: commands 0.02 rad apart complete 70% of each step and
   clamp 28 times; 0.10 rad apart complete 92% and clamp 5 times. Small
   increments sit near the servos' stiction threshold. This is why the
   joint appeared to stop dead at −2.07 rad mid-run while a *single*
   0.4 rad command moved it straight through that point with a travel
   ratio of 0.986.

   Note the tension: a bigger step is better for the servos, but coarser
   sampling only preserves a *straight* joint-space path (what
   `plan_tcp.py` produces). A curved path from the constraint-graph
   planner needs fine sampling for fidelity, so it will meet the stiction
   problem — there, keep `--sync-tol` above `--max-step` so the motion
   flows with bounded lag instead of stopping at every waypoint.

**Making it smooth.** Waiting at every waypoint works but stutters — the
arm stops 7 times in 7 waypoints. Three separate jobs had been collapsed
into one number, and splitting them is what makes the motion flow:

| knob | job | default |
|---|---|---|
| `--max-step` | how finely the path is sampled (fidelity) | 0.02 |
| `--servo-clamp` | how far the command may *lead* the measurement | 0.10 |
| `--sync-tol` | how far the arm may trail the stream | 0.08 |
| `--speed-scale` | GOAL_SPEED as a multiple of the plan's own velocity | 1.5 |

Keeping the tolerance above the sampling step means the arm never has to
stop: it always chases a target a little ahead of it, the lead keeps the
servos above their stiction threshold, and the clamp bounds how far off
the checked path it can get. The velocity feedforward is what stops each
command running its own accel/decel ramp — without it, consecutive
commands do not blend.

Measured across four hardware runs of the same motion:

| run | stopped to wait | settled within | TCP error |
|---|---|---|---|
| stop-at-every-waypoint | 7 of 7 | — | 7.0 mm |
| flowing, no settle | 0 of 57 | — | 10.2 mm |
| flowing + slow settle | 0 of 57 | 0.052 rad | — |
| flowing + settle at speed | **0 of 57** | **0.034 rad** | **9.5 mm** |

**Settle at speed, not slowly.** After the last waypoint the arm is still
up to `--sync-tol` behind, so the goal is re-sent until it arrives. The
first version crept at 0.15 rad/s to avoid hunting and left
`shoulder_lift` 0.052 rad short every time — a small error commanded
slowly cannot break stiction, while the same 0.05 rad error at default
speed closes and slightly overshoots (ratio 1.197). Same lesson as the
step size, in a different disguise.

**The floor is the arm.** Whichever joint carries the load stops 0.03–0.05
rad short and stays there — `shoulder_lift` (−2.99°) moving one way,
`elbow_flex` (−1.97°) the other, about 1 cm at the TCP either way. That is
gravity droop against finite position-control stiffness: re-commanding
does not close it, because the servo already believes it has arrived.
Hence `--settle-tol` defaults to 0.05 rather than pretending tighter is
achievable, and closing that last centimetre needs gravity compensation
or feedback from something other than the servos' own encoders.

**One more trap, fixed.** `execute.py` used to stream its first command
before the bus had answered. `ServoHardwareInterface` serves a
placeholder 2048 ticks per joint until its first successful sync-read, so
that command was clamped against a fake "arm is at zero" pose — seen
live, `wrist_roll` sitting at −2.71 rad was commanded toward +0.1 for one
tick. It now waits for a real read and refuses to stream without one.

**Diagnosing a joint that will not follow.** `joint_test.py` moves one
joint while holding every other at its measured value:

```bash
python -m soarm_tamp.joint_test --port ... --joint elbow_flex \
    --delta 0.3 --single --hand-is-clear
```

`--single` is the mode that answers the mapping question — one command,
no step clamp, settle, then measured excursion over commanded. Near 1.0
clears the URDF-to-servo mapping for that joint (measured: shoulder_pan
0.982, elbow_flex 0.946, wrist_roll 0.992 — the mapping is fine, and the
travel-span mismatch the calibration check flags is not what broke the
run). The laddered mode measures something else entirely — how much of
each step the servo completes before the next command — and must not be
read as a mapping check.

## Planning from where the arm actually is

```bash
python -m soarm_tamp.read_pose --port /dev/cu.usbmodemXXXX \
    --out runs/start.json                                    # host
./scripts/hpp_container.sh tcp --start runs/start.json \
    --xyz 0.22 0.0 0.08 --out runs/tcp06                     # container
```

A plan whose first waypoint is the URDF zero pose assumes the arm is at
the zero pose. It is not, and `execute.py` streams the plan regardless —
the servos slew to the plan's start along a path nothing checked. With
`--start`, that leg is part of the validated trajectory instead of a gap
before it.

## Where the hand can actually go

At x = 0.22 m with the hand vertical, IK solves for TCP z from 0.03 to
0.08 m and finds nothing at 0.10 m or above; tilting the approach 30° off
vertical finds nothing at any of those heights at that radius. On a
5-DOF arm the achievable wrist orientation is coupled to position, so
"move it higher for safety" runs out quickly — 0.08 m is the ceiling for
a top-down reach at that radius.

**A gap worth knowing:** the table is modelled as a thin slab, so the
whole half-space *beneath* it reads as free space. HPP will call a pose
with the hand below the table surface collision-free; it only catches a
path that crosses the slab. Combined with the fingertips extending ~8 mm
past the TCP frame and the collision box being recessed 3 mm, a hand
resting on the real table can be "valid" in the model.

## The physical setup

The arm is bolted to a flat surface which is the `z = 0` plane. The cube
starts at **A = (0.22, −0.10)** and must end at **B = (0.22, +0.10)**,
both in metres in the robot base frame, both inside the verified
top-down-reachable annulus (radius 0.10–0.30 m).

Use a **25 mm** cube. That is not arbitrary — `studies/reachability.py`
measures the jaw opening against the `gripper` joint angle, and 25 mm sits
where the jaws have clearance to approach (29 mm at +10°) and room to
squeeze past contact (19.8 mm at 0°).

## Why a 5-DOF mask

The SO-101 has five arm joints. A full 6-DOF grasp constraint is six
equations, so it is solvable only where the system happens to be
degenerate. The cube's handle therefore carries `<mask>1 1 1 1 1 0</mask>`,
freeing rotation about the approach axis — the DOF a parallel-jaw grip on
a cube does not care about anyway. A 600k-sample sweep found ~170° of
usable yaw at every candidate spot, so this costs no reachability.

## Joint conventions — read this before running on hardware

The planning URDF, `soarm_sdk` and lerobot each use a different joint-angle
zero, and until now no code related any of them to the URDF's. That mapping
now lives in `soarm_sdk.calibration`, shared with RL deployment
rather than reimplemented here.

It is **seeded offline** from measured travel plus the URDF's joint limits.
The seed cannot recover the direction signs — a travel range says how far a
joint moves, not which end is which — so it is written `validated: false`
and `execute.py` refuses to stream against it. `validate_calibration.py`
settles the signs with the arm limp (torque off, read only), then
re-confirms under power, then requires a tape-measure FK check before
marking the file validated.

Joint limits and a per-step bound are enforced inside the SDK, so they
apply to every caller, not just to trajectories that come through here.

## Layout

| File | Runs where | Purpose |
|---|---|---|
| `geometry.py` | either | Measured gripper/workspace constants. Single source of truth. |
| `build_assets.py` | either | Generates `generated/` URDF+SRDF from the vendored SO-101. |
| `config/cube_pick_place.yaml` | container | The `long_tamp` task config. |
| `plan.py` | container | Plans the pick-and-place and writes the manifest. |
| `plan_tcp.py` | container | Plans a single TCP-pose reach into the same manifest format. |
| `replay.py` | container | Replays a manifest in the viser 3-D viewer, or `--follow`s a live run. |
| `conventions.py` | host | Finds the calibration; derives planning bounds. |
| `validate_calibration.py` | host | Confirms the calibration on the real arm. |
| `execute.py` | host | Resamples the manifest and streams it to the servos. |
| `studies/reachability.py` | container | Reproduces every constant in `geometry.py`. |

`generated/` is build output — edit `build_assets.py`, not those files.

## Growing this

The scene is authored so the obvious next steps are config edits:

- **More cubes**: add another object to the YAML with its own handle and
  a `foot` gripper, and extend `GRASP_SEQUENCE` in `plan.py`.
- **More destinations**: add handles to `table.srdf` (`spot_c`, …) and
  the matching `valid_pairs` entries.
- **Stacking**: put the destination handle on a *cube* rather than the
  table — the docking mechanism is already the one that makes "place at a
  named spot" deterministic, and it does not care whether the spot is
  furniture or another block.

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
| Joint sign calibration | **not done — needs the arm** |
| Run on hardware | **not done — arm was not connected** |

Everything up to the hardware boundary has been run and verified. The last
two rows need the physical arm; `execute.py` refuses to stream until the
signs are measured (see *Joint conventions* below).

## Running it

```bash
# 1. plan (creates/starts the container as needed)
./scripts/hpp_container.sh plan --out runs/cube01 --viewer none

# 2. watch it in the 3-D viewer, then open http://localhost:8000
./scripts/hpp_container.sh replay --run runs/cube01

# 3. check what would be sent, no hardware needed
python -m soarm_tamp.execute runs/cube01 --dry-run

# 4. ONCE per arm: measure the joint signs (torque off, arm moved by hand)
python -m soarm_tamp.calibrate_conventions --port /dev/cu.usbmodemXXXX

# 5. run it
python -m soarm_tamp.execute runs/cube01 --port /dev/cu.usbmodemXXXX
```

Regenerate the scene assets after changing `geometry.py`:

```bash
python -m soarm_tamp.build_assets
```

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

The planning URDF and `soarm_sdk` do not share a joint convention.
Offsets are recoverable by comparing the two limit tables
(`shoulder_lift` ≈ −π/2, `elbow_flex` ≈ +π/2, others ~0). **Signs are
not.** Five of six URDF ranges are symmetric about zero, and a symmetric
range fits its servo counterpart equally well either way round — the
information is not in the files.

A wrong sign drives the arm into the table rather than over it, so
`execute.py` refuses to run until `calibrate_conventions.py` has measured
them. That tool disables servo torque and only reads, so the arm cannot
move under power while the question is open.

## Layout

| File | Runs where | Purpose |
|---|---|---|
| `geometry.py` | either | Measured gripper/workspace constants. Single source of truth. |
| `build_assets.py` | either | Generates `generated/` URDF+SRDF from the vendored SO-101. |
| `config/cube_pick_place.yaml` | container | The `long_tamp` task config. |
| `plan.py` | container | Plans and writes the waypoint manifest. |
| `replay.py` | container | Replays a manifest in the viser 3-D viewer. |
| `conventions.py` | host | URDF↔servo mapping, and safe planning bounds. |
| `calibrate_conventions.py` | host | Measures the joint signs on the real arm. |
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

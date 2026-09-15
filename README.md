# soarm_tamp

The layer for designing, building, and validating a long-horizon TAMP
mission (`long_tamp` on HPP) on a physical **SO-101** arm: plan in the HPP
container, then execute and watch it run on the host. Calibration (mapping
this arm's ticks to the planning URDF's frame) is a separate, one-time
prerequisite — see [Calibration](#calibration) — done before working with
this package, not one of its pipeline stages.

It ships with two worked examples — a minimal single-goal reach
(`plan_tcp.py`) and a full grasp-sequence **pick-and-place** (`plan.py`),
picking a cube up from point A and placing it at point B. The
pick-and-place is the **reference example**: read [Example: cube
pick-and-place](#example-cube-pick-and-place) first, and copy its shape for
a new mission. Writing a new one is covered in
[`docs/writing-a-new-task.md`](docs/writing-a-new-task.md) — read it
yourself, or point an AI coding agent at it.

## Installation

This package has no hard dependencies by design: planning and execution run
in environments that must not share a dependency set.

| Side | Needs | How |
|---|---|---|
| Container (planning) | `long_tamp` + `pyhpp` | built into the image `scripts/hpp_container.sh` manages |
| Host (execution + dashboard) | `soarm_sdk` | `pip install soarm_tamp[host]` |

From a checkout, for host-side work:

```bash
pip install -e ".[host]"
```

## Dashboard

A Viser plan-and-run dashboard, built on `soarm_sdk`'s dashboard shell:

```bash
soarm-tamp-dashboard --port 8080     # after pip install -e ".[host]"
python -m soarm_tamp.dashboard       # from a checkout, same thing
```

| Tab | Purpose |
|---|---|
| Start Up | connect to the arm, live 3-D mirror |
| Execution Watchdog | read-only: flags calibration/model drift before you plan |
| TCP Plan | one Cartesian goal — the cheapest end-to-end pipeline check |
| Pick & Place | the reference example: the full cube pick-and-place task |

Press the tabs in that order — each one only makes sense once the one
before it works. Plan/play/execute in the dashboard talk to the same
container and manifest contract as the CLI scripts above. TCP Plan is
task-agnostic (any Cartesian goal); Pick & Place is wired specifically to
the cube example's geometry. Adding a tab for a new task is covered in
[`docs/writing-a-new-task.md`](docs/writing-a-new-task.md#adding-a-dashboard-tab-for-the-new-task).

## Calibration

The planning URDF, `soarm_sdk`, and lerobot each use a different
joint-angle zero. That mapping lives in `soarm_sdk.calibration`, shared
with RL deployment rather than reimplemented here.

It is **seeded offline** from measured travel plus the URDF's joint limits,
once per arm:

```bash
soarm-seed-calibration \
  --lerobot ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/thanh_arm.json \
  --arm-id thanh_arm
```

The seed cannot recover the direction signs — a travel range says how far a
joint moves, not which end is which — so it is written `validated: false`
and `execute.py` refuses to stream against it until validation completes:

```bash
python -m soarm_tamp.validate_calibration --port /dev/cu.usbmodemXXXX
```

This settles the signs with the arm limp (torque off, read only),
re-confirms under power, then requires a tape-measure FK check before
marking the file validated. `soarm-dashboard-calibration` (from `soarm_sdk`)
provides the same workflow as a guided, four-step Viser dashboard instead of
a CLI script.

Joint limits and a per-step bound are enforced inside the SDK, so they
apply to every caller, not just to trajectories that come through here. See
[`docs/joint-limits-architecture.md`](docs/joint-limits-architecture.md)
for the full layer map of where a "how far can this joint go" number lives.

## Example: cube pick-and-place

The reference example: pick up a cube at point A and dock it at point B, a
three-phase grasp sequence (`GRASP_SEQUENCE` in `plan.py`):

```
1. so101/grasp -> cube/top       pick the cube up at A
2. cube/foot   -> table/spot_b   dock it onto the named spot at B
3. so101/grasp -> (release)      let go; the cube stays on its spot
```

Defined in `config/cube_pick_place.yaml` (the task config) and `plan.py`
(the `CubePickPlaceTask` that drives it) — see
[`docs/writing-a-new-task.md`](docs/writing-a-new-task.md) for how those two
pieces work together, and to build a different task the same way.

**Physical setup.** The arm is bolted to a flat surface which is the
`z = 0` plane. The cube starts at **A = (0.22, −0.10)** and must end at
**B = (0.22, +0.10)**, both in metres in the robot base frame, both inside
the verified top-down-reachable annulus (radius 0.10–0.30 m, |A| = |B| =
0.242 m).

Use a **30 mm** cube — matched to the jaws, not arbitrary:
`studies/reachability.py` measures jaw opening against the `gripper` joint
angle. The planner freezes the jaw at 20° (36.5 mm open, 6.5 mm clear of
the cube) for approach; execution commands 5° (25.2 mm) to close — 4.8 mm
past contact, so the position-controlled servo grips by stalling against
its torque limit rather than by reaching a precise "closed" angle.
Changing these constants needs `python -m soarm_tamp.build_assets` to
regenerate `generated/` before they take effect.

**Status:** planned and dry-run verified (3 phases, 6 segments, 0 seam
violations, 0 of 623 commands clamped) but not yet executed on the physical
arm — the "run on hardware" milestone so far covers the generic TCP-reach
example (within 7.0 mm / 2.2° of the commanded pose), not this task. A
manifest records a **scene fingerprint** of these constants (cube size,
grasp frame, pick/place points); `execute.py` refuses to run one whose
fingerprint no longer matches `geometry.py`, so an edit here can't silently
execute against stale geometry.

### Extending this example

- **More cubes** — add another object to the YAML with its own handle and
  a `foot` gripper, and extend `GRASP_SEQUENCE` in `plan.py`.
- **More destinations** — add handles to `table.srdf` (`spot_c`, …) and
  the matching `valid_pairs` entries.
- **Stacking** — put the destination handle on a *cube* rather than the
  table. The docking mechanism that makes "place at a named spot"
  deterministic already doesn't care whether the spot is furniture or
  another block.

## Documentation

- [`docs/writing-a-new-task.md`](docs/writing-a-new-task.md) — full
  tutorial: confirming the pipeline, authoring a task's YAML config and
  plan script, execution tuning, and adding a dashboard tab for it.
- [`docs/execution-tuning.md`](docs/execution-tuning.md) — the full
  hardware-streaming incident behind the tuning defaults in the tutorial
  above.
- [`docs/joint-limits-architecture.md`](docs/joint-limits-architecture.md)
  — where every joint-limit number lives across the stack, and why that
  matters.
- [`CHANGELOG.md`](CHANGELOG.md) — notable changes, [Keep a
  Changelog](https://keepachangelog.com/en/1.1.0/) format.

## Development

```bash
ruff check soarm_tamp tests   # lint
pytest                        # tests
```

## License

MIT — see [LICENSE](LICENSE).

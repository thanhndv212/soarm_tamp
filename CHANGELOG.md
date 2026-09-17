# Changelog

All notable changes to `soarm-tamp` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Initial development. Nothing released yet, and nothing has run on the
physical arm yet — see "Not yet done" below.

### Added

- **The dashboard's `--rerun` flag**, feeding its streaming interface's
  telemetry to a spawned Rerun viewer (`soarm_sdk.monitoring.blueprint`'s
  by-servo/by-channel layouts) alongside the TCP/pick-and-place controls,
  on the same connection. `build_app()` also takes `rerun=` directly, for
  a caller building a dashboard programmatically rather than through the
  CLI.

### Changed

- **`__main__.py` now registers its `plan_and_run` profile against
  `soarm_sdk.cli.dashboard.launch()`** instead of hand-rolling a second
  copy of the device/baud/urdf/stream argument definitions and
  auto-device-selection logic. That copy had already fallen behind once —
  it had no `--rerun` flag until one was ported over by hand — because
  `soarm_sdk`'s version of the same logic wasn't reusable outside that
  package until now. Streaming stays on by default (`--no-stream` to opt
  out), since the TCP/pick-and-place panels need the persistent interface
  regardless.

### Fixed

- **`validate_calibration` could not complete any of its three rungs.**
  Rung 1 needs the arm limp, but `ServoRobot.connect()` enables torque and
  the SDK exposed no way to release it, so the script fell back to asking the
  operator to cut the servo supply — which takes the bus down too, leaving
  the position reads the check depends on returning a placeholder. It now
  calls `ServoRobot.disable_torque()` (new in soarm-sdk) and re-enables
  before rung 2. Rungs 2 and 3 issued a single `set_joint_positions` per
  target, but the `max_step_rad=0.05` this script passes clamps every write
  to a small delta from the *measured* position — so the arm moved 0.05 rad
  and stopped, and rung 3 had the operator tape-measuring a pose it never
  reached. Both now step to the target and report if it is not reached.
- **A dead bus read no longer masquerades as a stationary arm.**
  `ServoHardwareInterface` serves 2048 ticks per joint until its first
  successful sync-read, so a silent bus produced constant, plausible angles;
  each rung compares two reads, and two reads of the same stale cache differ
  by zero, which surfaced as "you did not move it far enough". The run now
  aborts up front unless the arm has actually been read.

### Added

- **First successful run on the physical SO-101.** A planned TCP-pose
  reach executed open-loop to within **7.0 mm and 2.2°** of the commanded
  pose, every joint inside 2.02° of its planned angle. Getting there took
  two fixes and corrected two wrong diagnoses; the whole chain is in the
  README under *Streaming a trajectory the arm can actually follow*.
- **`execute.py --sync` (on by default): wait for the arm at each
  waypoint.** The first live attempt drove the hand into the table on a
  path whose every waypoint HPP had certified collision-free. Cause: the
  stream sent a waypoint per tick regardless of where the arm was, while
  the SDK clamps every command to `--max-step` of the *measured*
  position. Each joint fell behind in proportion to its travel
  (`wrist_flex` 1.36 rad vs `elbow_flex` 0.54), the arm stopped tracing
  the line that had been checked, and contact with the table stalled two
  joints. Sync bounds the lag by `--sync-tol` and reports `worst lag` and
  `timed out waiting`, so a run that cannot keep up says so instead of
  discovering it by collision. Measured: worst lag 0.675 rad without the
  step-size fix below, 0.035 rad with it.
- **Continuous motion instead of stop-and-go.** The synchronised run above
  stopped at all 7 of its waypoints. `--max-step` (path sampling),
  `--servo-clamp` (how far the command may lead the measurement) and
  `--sync-tol` (how far the arm may trail) are three different jobs that
  had been one number; split, with the tolerance above the sampling step,
  the arm never stops — it chases a target slightly ahead of itself, which
  also keeps the servos above their stiction threshold. Added
  `--speed-scale`, which sets each servo's GOAL_SPEED from the plan's own
  velocity so consecutive commands blend instead of each running its own
  accel/decel ramp. Measured on hardware: 0 of 57 waypoints blocked, worst
  in-flight lag 0.047 rad.
- **`--settle-tol`: arrive on the goal, not near it.** Flowing motion is
  bought by letting the arm trail the stream, which is wrong at the end of
  a segment — the last command goes out while the arm is still closing the
  gap. The goal is now re-sent until the arm arrives. Settling *at speed*
  matters: creeping at 0.15 rad/s left `shoulder_lift` 0.052 rad short
  every time, while the default speed closes the same error (and slightly
  overshoots, ratio 1.197). The default tolerance, 0.05 rad, is this
  arm's measured floor — whichever joint carries the load stops 0.03-0.05
  rad short and stays there (`shoulder_lift` one way, `elbow_flex` the
  other, ~1 cm at the TCP). That is gravity droop against finite
  position-control stiffness, not pacing: re-commanding cannot close it
  because the servo already believes it has arrived.
- **`execute.py` no longer commands before the bus has answered.**
  `ServoHardwareInterface` serves a placeholder 2048 ticks per joint until
  its first successful sync-read, and every command is clamped against the
  measured position — so the first command was clamped around a fake "arm
  is at zero" pose. Seen live: `wrist_roll` at −2.71 rad commanded toward
  +0.1 for one tick, taken back 33 ms later by the next command. It now
  waits for a real read and refuses to stream without one. Same trap
  `validate_calibration` already guarded against.
- **Step size matters more than it looks.** On `wrist_roll`, commands
  0.02 rad apart complete 70% of each step (28 clamps); 0.10 rad apart
  complete 92% (5 clamps). Small increments sit near the servos' stiction
  threshold. This produced a convincing false diagnosis mid-session — the
  joint stopped dead at −2.07 rad and looked mechanically blocked, while a
  single 0.4 rad command moved it through that point with a travel ratio
  of 0.986. `--max-step` now documents the trade-off, including why a
  curved path cannot simply take the same setting.
- **`read_pose.py`: plan from the arm's measured pose.** `plan_tcp --start
  <json>` replaces the configuration's arm joints with what the servos
  report, so the leg from the arm's real pose to the plan's first waypoint
  is part of the validated trajectory instead of an unchecked slew before
  it. Refuses a pose outside the planning bounds rather than clamping it.
- **`joint_test.py`: move one joint and report what it did.** Written
  because nothing in the plan-and-stream path can separate a servo fault
  from a mapping error from a gravity stall — it moves six joints at once.
  `--single` (one command, no step clamp, settle, read) is the only mode
  that measures the URDF-to-servo mapping: shoulder_pan 0.982, elbow_flex
  0.946, wrist_roll 0.992, which cleared the mapping and, with it, the
  travel-span mismatch that the calibration gate flags. The laddered mode
  measures per-step completion and an earlier version of the file wrongly
  reported that as a mapping ratio.
- **Reach envelope, measured.** At x = 0.22 m, top-down IK solves for TCP
  z in 0.03–0.08 m and fails at 0.10 m and above; a 30°-tilted approach
  fails at every height tried at that radius. Also recorded: the table is
  a thin slab in the model, so the half-space below it reads as free
  space and a hand resting on the real table can be "valid".
- **The reach is planned by `long_tamp`, not by a raw pyhpp call.**
  `plan_tcp.py` first shipped planning its path with `ps.directPath` on
  the bare manipulation problem, with `skip_graph=True` and a
  straight steering method swapped in — which worked for an unobstructed
  straight line and bypassed the machinery this repo exists for. It now
  calls `GraspSequencePlanner.plan_loop()`, which builds a phase graph for
  the current held-grasps state and plans its loop edge (`Loop | f`)
  through `plan_transition_edge`: leaf projection, direct path with a
  sampling planner behind it, the YAML's optimizers, and time
  parameterization. Same motion, both ways: 8 waypoints over 0.3146 rad
  at a constant 0.157 rad/step, versus 28 over 1.3477 s ramping
  0.000 → 0.114 → 0.000. Coverage is unchanged at 17 of 20 sampled
  targets; the three failures (r ≈ 0.11-0.12 m) now fail inside
  `TransitionPlanner.computePath` on a self-collision rather than on the
  straight line, which is worth a look since `computePath` is the
  sampling planner and the goal configurations pass `isConfigValid`.
- **`execute.py` replays a plan's velocity profile.** A time-parameterized
  segment is marked in the manifest, and its waypoint spacing is treated
  as time: each interval is still subdivided to respect `--max-step`, but
  its dwell is divided among the sub-steps, so the planner's accel and
  decel survive instead of being flattened to a constant rate. Measured on
  the same plan: 1.4 s replaying the profile against 2.5 s without
  (`--no-plan-timing`). Two pacing bugs fixed on the way — per-command
  `sleep()` accumulated the OS's overshoot (a 1.35 s trajectory took
  2.8 s), and `--sync` waited each dwell twice, once in the arrival floor
  and once in the sleep. Both are now one wall-clock deadline per segment.
- **`plan_tcp.py`: plan a move of the TCP to a commanded pose.** The
  simplest thing the stack can plan — no grasp, no object, no constraint
  graph. `scripts/hpp_container.sh tcp --xyz 0.22 0.0 0.05` solves IK
  against a 5-DOF-masked pose constraint on `gripper_frame_link` and plans
  a straight joint-space move to it, writing the same manifest `plan.py`
  does. Verified: 0.0000 mm / 5.4e-06 rad goal error, and `execute
  --dry-run` reports 0 of 145 commands clamped.

  Two pyhpp traps had to be worked around, both worth knowing before
  reaching for these bindings again:

  - `Problem.addNumericalConstraintsToConfigProjector` **segfaults** on a
    manipulation problem. It dereferences `constraints_`, which
    `pyhpp::core::Problem`'s pointer constructor — the one
    `pyhpp::manipulation::Problem` delegates to — never initializes. A
    standalone `ConfigProjector` does the same job and leaves the problem
    unconstrained for path planning, which is what a free reach wants.
  - `ps.steeringMethod(sm)` on a manipulation problem **silently** leaves
    the graph steering method in place; pyhpp overloads that name three
    ways on the subclass and Boost.Python picks the wrong one. Calling the
    base class explicitly (`core.Problem.steeringMethod(ps, sm)`) installs
    it. Without that, `directPath` raises "The constraint graph should be
    set to use the steeringMethod::Graph".

  No fallback when the straight line is blocked, and that is deliberate:
  HPP's own planners refuse a graph-less manipulation problem
  (`checkProblem()` throws "No graph in the problem."), and a hand-rolled
  detour via random waypoints rescued 0 of 5 blocked goals in 200 tries
  each — 0 again when the waypoints were biased to hold the TCP 120 mm
  above the table. Measured coverage: of 20 top-down targets across the
  reachable annulus, IK solved 20 and the straight line validated 17.
- **Live viser mirror of a run.** `execute.py` appends every command it
  issues to `<run>/live.jsonl`, and `replay.py --follow` tails that file
  and shows the arm as it moves. The two halves still never share a
  process — the run directory is bind-mounted into the container, so the
  live channel is a file, the same shape as the manifest handoff. The
  trace is also the record of what the arm was actually told to do.
  `--no-trace` disables it; `execute --dry-run --pace` drives the mirror
  at true speed with no arm connected (verified: 145 commands mirrored).
- **`--dry-run` now reports how many commands would be clamped.** The plan
  for this package always called for that check — "dry-run on cube05 must
  report 0 clamped" — but clamp counters live in the SDK and are only read
  back after a *live* run, so a dry run could never answer it. It now
  computes the same limits `ServoRobot` would enforce (including the
  intersection with a calibration's measured travel) and reports against
  them. It found a real defect immediately: 200 of 366 commands on cube05
  would have been clamped, from a frame error in the SDK's declared limits.
  Fixed there; all runs now report 0.
- Both `execute.py` and `validate_calibration.py` now load the **`so101`**
  config explicitly rather than taking `ServoRobot`'s `soarm100` default.
  They ship identical limits, but this package plans against the SO-101
  URDF and should say so.

### Changed

- Import paths updated for `soarm_sdk`'s package reorganization
  (`soarm_sdk.frame_calibration` → `soarm_sdk.calibration.frame`,
  `soarm_sdk.servo_robot` → `soarm_sdk.robot`). Cosmetic only: the SDK keeps
  deprecation shims at the old paths, so this would have kept working
  untouched. Behaviour is unchanged — `execute.py --dry-run` produces a
  byte-identical plan summary before and after.
- Deliberately **not** adopted from the reorganized SDK: `NullRobot` and
  `soarm_sdk.trajectory.resample()`. Both are used on the `--dry-run` path,
  which `execute.py` keeps runnable inside the HPP planning container where
  `soarm_sdk` is not installed at all; importing either would reintroduce
  exactly the dependency the deferred import at `execute.py:157` avoids.

- **Hardware access consolidated onto `soarm_sdk`**; the lerobot path is
  gone. The URDF-to-servo mapping moved into
  `soarm_sdk.frame_calibration`, so it is shared with RL deployment rather
  than reimplemented here, and joint-limit and step-size clamping are
  enforced inside the SDK for every caller.
- `conventions.py` shrank to the planning-specific part: finding the
  calibration, refusing an unvalidated one, and deriving planning bounds
  from measured travel. Its previous hand-derived offsets and assumed
  signs are superseded.
- `calibrate_conventions.py` replaced by `validate_calibration.py`, which
  walks three escalating rungs — torque-off hand check, single-joint moves
  under power, then a tape-measure FK check — and marks the calibration
  validated only if all three pass.

### Added

- Long-horizon TAMP planning (`long_tamp` on HPP) for a physical SO-101,
  first task being a 25 mm cube picked from A = (0.22, −0.10) and docked
  on a named spot at B = (0.22, +0.10).
- `geometry.py` — the measured SO-101 gripper and workspace constants that
  everything else derives from: jaw opening vs joint angle, grasp
  centerline offset, grasp depth, top-down reachable annulus.
- `build_assets.py` — generates the HPP scene into `generated/`: the
  SO-101 URDF with absolute mesh paths, an SRDF carrying the measured
  grasp frame, and the cube and table with their grasp semantics.
- `config/cube_pick_place.yaml` — the `long_tamp` task config.
- `plan.py` — plans the three-phase sequence (pick → dock → release) and
  records it as a `PathRecorder` waypoint manifest.
- `replay.py` — replays a recorded manifest in the viser 3-D viewer
  without replanning, so what you watch is what `execute.py` would send.
- `conventions.py` — finds the arm's calibration, refuses an unvalidated
  one, and derives planning bounds from its measured travel.
- `validate_calibration.py` — confirms a seeded calibration on the real
  arm across three escalating rungs, starting with torque disabled.
- `execute.py` — resamples a manifest and streams it to the servos via
  `soarm_sdk`.
- `studies/reachability.py` — reproduces every constant in `geometry.py`
  from the URDF and meshes, so they can be re-derived rather than trusted.
- `scripts/hpp_container.sh` — creates and drives the planning container.
  It uses its own container name rather than the existing
  `hpp-agimus-arm64`, which has no mount for this workspace; adding one
  would mean recreating that container and discarding its writable layer.
- `LICENSE` (MIT) and this file.

### Notes on the design

- **The cube's grasp handle is masked to 5 DOF** (`1 1 1 1 1 0`, freeing
  rotation about the approach axis). The SO-101 has five arm joints, so a
  full 6-DOF grasp is six constraints and only solvable where the system
  happens to be degenerate. A 600k-sample sweep found ~170° of usable yaw
  at every candidate spot, so freeing that DOF costs no reachability.
- **Planning bounds are the servo-reachable ranges, not the URDF's.** The
  URDF lets `wrist_roll` reach 2.841 rad but the servo stops at 2.79, and
  the first successful plan put 41 waypoints past that stop. Clamping at
  send time would deform the path rather than follow it, so the
  restriction lives in the config where the planner can respect it.
- **Planning and execution never share a process.** `pyhpp` exists only in
  the container, the servos only on the host; the contract between them is
  a manifest on disk.

### Not yet done

- The calibration is seeded but not validated. The direction signs are
  assumed +1 and have never been checked on this arm, so `execute.py`
  refuses to stream until `validate_calibration.py` has confirmed them.
- Nothing has been run on hardware. Everything up to that boundary has
  been verified: 3 phases, 6 segments, 0 seam violations, 0 commands
  clamped in a dry run.
- `wrist_roll` sweeps 5.53 rad (317°) and touches both bounds. The plan is
  valid, but that is a near-full wrist revolution between pick and place —
  slow on hardware, and how cables get wound. Worth biasing the freed DOF
  toward yaw continuity before the first hardware run.

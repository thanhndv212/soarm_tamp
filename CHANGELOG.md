# Changelog

All notable changes to `soarm-tamp` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Initial development. Nothing released yet, and nothing has run on the
physical arm yet — see "Not yet done" below.

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

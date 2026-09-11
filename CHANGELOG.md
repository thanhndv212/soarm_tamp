# Changelog

All notable changes to `soarm-tamp` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Initial development. Nothing released yet, and nothing has run on the
physical arm yet — see "Not yet done" below.

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
- `conventions.py` — the URDF↔servo joint mapping, plus
  `safe_planning_bounds()`.
- `calibrate_conventions.py` — measures the joint signs on the real arm
  with servo torque disabled, reading only.
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

- Joint signs have never been measured on the physical arm. They are not
  derivable from the models — five of six URDF ranges are symmetric about
  zero — so `execute.py` refuses to stream until
  `calibrate_conventions.py` has run.
- Nothing has been run on hardware. Everything up to that boundary has
  been verified: 3 phases, 6 segments, 0 seam violations, 0 commands
  clamped in a dry run.
- `wrist_roll` sweeps 5.53 rad (317°) and touches both bounds. The plan is
  valid, but that is a near-full wrist revolution between pick and place —
  slow on hardware, and how cables get wound. Worth biasing the freed DOF
  toward yaw continuity before the first hardware run.

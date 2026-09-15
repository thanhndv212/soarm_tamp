# Joint limit architecture: current state, industry practice, and recommendations

**Arm:** thanh_arm (SO-101, 6× STS3215)
**Date:** 2026-09-14
**Status:** one confirmed live bug (gripper), one latent (elbow_flex), fix proposed

This document exists because a single motion — a TCP plan that asked `wrist_flex`
to move 0.41 rad past a number nobody had told the software about — took most
of a debugging session to trace. The arm didn't error. It didn't clamp
visibly. It just stopped moving that one joint and kept going on the other
five, which looks exactly like bad tuning and is actually a servo silently
refusing a command. This document maps every place a "how far can this joint
go" number now lives, cross-references it against how professional robot
stacks solve the same problem, and lists what's still wrong.

---

## 1. The layer map

Seven places declare or derive a joint limit for this arm, plus one register
that isn't a limit but behaves like one when ignored. They form a stack.
Physically, only the bottom layer is real; every layer above it is somebody's
*belief* about the bottom layer, and the entire debugging session was several
of those beliefs turning out to be stale.

| # | Layer | Lives in | Loaded by | Can override |
|---|-------|----------|-----------|--------------|
| 0 | **Mechanism** — the physical hard stop | the arm itself | nothing reads it directly | everything |
| 1 | **Servo EEPROM** `MIN/MAX_ANGLE_LIMIT` (registers 9, 11) | firmware, written once at bench setup, *outside this repo* | nothing, until this session added `read_angle_limits()` | every software layer |
| 2 | **`ServoRobot`/`ServoHardwareInterface._apply_safety()`** | [`soarm_sdk/robot/hardware.py`](../../soarm_sdk/src/soarm_sdk/robot/hardware.py) `_limit_lo/_limit_hi`, `_max_step_rad` | `ServoRobot.__init__`, every call to `set_joint_positions` | the command actually sent |
| 3 | **`effective_limits()` policy** | [`soarm_sdk/calibration/limits.py`](../../soarm_sdk/src/soarm_sdk/calibration/limits.py) | `ServoRobot.effective_joint_limits()` | layer 5's declared bounds |
| 4 | **Calibration** (`tick_min/max`, `zero_offset_ticks`, `direction_sign`) | `~/.soarm_sdk/calibration.json` | `RobotCalibration.load()` | feeds layers 3 and 5 |
| 5 | **`safe_planning_bounds()`** | [`soarm_tamp/conventions.py`](../../soarm_tamp/conventions.py) (`URDF_LIMITS` ∩/⊂ layer 4) | soarm_tamp scripts, the dashboard | advisory only |
| 6 | **Planner YAML** `joint_groups` | [`config/cube_pick_place.yaml`](../soarm_tamp/config/cube_pick_place.yaml) | `plan.py` `_YAML_PATH`, inside the HPP container | what the planner is allowed to search |
| 7 | **URDF `<limit>`** | `so101_new_calib.urdf`, duplicated in `so101.yaml` and `conventions.URDF_LIMITS` | HPP, `load_robot_config()` | the model's own opinion |

Reading the table gives the wrong intuition — it looks like layer 7 (the
model) should be authoritative and layer 1 (a register) should be a detail.
It's the reverse. **Priority runs bottom-up: layer 1 always wins, whether or
not any software layer agrees with it.** A servo given a goal past its own
EEPROM cap does not ask layer 6 or 7 for permission; it accepts the byte,
does nothing, and reports success. That inversion — the layer with the least
visibility has the most authority — is the whole failure mode below.

Not a position limit, but entangled with these because it produces the same
symptom (a joint falling behind): `--max-step` (path resampling granularity),
`--servo-clamp` (how far a command may lead the last *measured* position),
`--sync-tol` / `--sync-timeout` (how long the executor waits for arrival
before moving on). These live in [`soarm_tamp/execute.py`](../soarm_tamp/execute.py)
and produce lag, not clamps — worth naming so a "joint fell behind" report
doesn't get diagnosed as a position-limit bug when it's a pacing one.

### How a value actually reaches the servo

```
URDF <limit>  ──┐
                ├─▶ safe_planning_bounds()  ──▶  planner YAML  ──▶  HPP plans
Calibration  ───┤        (soarm_tamp)          (config/*.yaml)     within this
 tick_min/max   │                                                   window
 zero_offset ───┼─▶ effective_limits()  ──▶  ServoRobot._apply_safety()
 direction_sign │     (soarm_sdk)              clamps EVERY command here
                │
Servo EEPROM  ──┴────────────────────────────────────────▶  the servo obeys
MIN/MAX_ANGLE      (never consulted by any layer above —     THIS, not the
_LIMIT (reg 9/11)   until this session's read_angle_limits)   clamped value
```

Layers 5–7 decide what gets *planned*. Layers 1–3 decide what gets
*commanded*. They meet only at layer 4 — the calibration file. If the
calibration's `tick_min/tick_max` don't match what's actually in the servo's
EEPROM, every layer above is planning against a window that doesn't match
the window layer 1 will actually allow, and nothing anywhere compares them.

---

## 2. What's wrong right now

### 2.1 Confirmed, live: the gripper cannot close

`execute.py` commands the jaw closed at `GRIPPER_CLOSED_DEG = 5°` (`+0.0873
rad`). The gripper servo's EEPROM floor is `+0.3373 rad` (1271 ticks). Tested
on hardware:

```
commanding CLOSED = +0.0873 rad
gripper ended     = +0.3419 rad   (1274 ticks, 0 mA)
```

Zero current draw, stopped three ticks above the firmware floor — the exact
signature of a silent firmware clamp, not a mechanical stall. Every
pick-and-place grasp will silently fail to close on the object. It hasn't
shown up yet because the sessions so far only ran TCP-only plans (no grasp
segment). The planner also freezes the jaw at `0.1745 rad` during planning —
also below the floor — so the grasp geometry itself is checked against a jaw
opening the arm cannot produce.

### 2.2 Latent, unresolved: `elbow_flex`

`phantom_range()` reports the planner is allowed down to `-1.4235 rad`
against a servo floor of `-0.4771 rad` — 0.95 rad of range the stack
believes exists and the firmware forbids. Unlike `wrist_flex`, this one is
**not yet confirmed** to be an artificial cap:

| | `wrist_flex` (confirmed artificial) | `elbow_flex` (unresolved) |
|---|---|---|
| current at the stop | 0–6 mA | 52–104 mA |
| load at the stop | 0% | 16.4%, sustained |
| stop point vs. push force | fixed, same tick every time | moved — 2271 gently, 2207 pushed harder |

The current/load signature says gravity and friction, not a firmware wall —
but the incremental probe used to test `wrist_flex` couldn't drive far enough
against gravity to reach the EEPROM floor and prove it either way. No plan
run so far enters that range, and `_within_servo_limits()` (§4) now blocks
one that would, so it's contained but not resolved.

### 2.3 Cosmetic: four joints' planner bounds are stale-narrow

`stale_bounds()` shows `shoulder_pan`, `shoulder_lift`, `wrist_roll`, and
`gripper` planned more conservatively than the arm's measured travel allows —
lost workspace, not a safety issue.

### 2.4 A gap in this session's own fix

`_within_servo_limits()` (§4) is wired into the **dashboard's** `PlanControls.execute()`
only. A bare `python -m soarm_tamp.execute run_dir --port ...` — the exact
path used to verify every fix in this session — has no servo-limit
preflight. The check needs to move into `execute.py` itself so both paths
are covered.

---

## 3. How professional stacks solve this

None of this is a new problem; it is the standard failure mode of any robot
whose motion stack has more than one place that thinks it owns "how far can
this joint go." Three ecosystems were checked against this arm's situation.

### 3.1 ros2_control: an explicit hard/soft hierarchy, validated at load

ros2_control's `joint_limits` model names the two tiers this document has
been calling layer-1-and-below vs. everything else: **hard limits** (the
physical/firmware boundary) and **soft limits** (where a controller *starts*
constraining motion, strictly inside the hard ones). The framework enforces
an invariant at load time: a soft limit that isn't strictly inside its hard
limit is a configuration error, not a runtime surprise. A global
`enforce_command_limits` flag makes the enforcement mandatory rather than
opt-in per call site. [ros2_control docs](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/joint_limiting.html)

**The gap this exposes here:** nothing in this stack validates that layer 5's
bounds sit inside layer 1's at load time. `unreachable_bounds()` (added this
session) is the first check that does this, and only for the planner YAML —
it isn't run automatically, and there's no equivalent check that layer 3's
`effective_limits()` output is inside layer 1.

### 3.2 Universal Robots: a locked safety tier, and a hard stop on disagreement

UR draws the same hard/soft line at the product level, not just in software.
**Safety Configuration** limits are checksummed and require restarting the
safety system to change; **application soft limits** are freely adjustable
per program and must live inside them. Critically: if the arm's *actual*
joint position is outside the configured safety limit at power-on, the
controller enters **Recovery mode** and refuses to run any program until a
person manually moves the arm back inside range using Freedrive.
[UR software safety limits](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-usr-man/complianceUR16e/H_g5_sections/firstuse/Tolerances_g5_en.htm)

**The pattern worth copying:** *disagreement between the believed state and
the actual state halts, it does not clamp.* This is exactly the fix already
applied for the wrist — `_within_servo_limits()` refuses to execute rather
than silently letting the arm drift — but UR applies the same refusal at
*startup*, before any plan exists, which is the gap in §2.4: the check should
run the moment a connection is established, not only right before a specific
execute.

### 3.3 Franka Emika: real exceptions, and a firmware/library version check

`libfranka` raises a real, typed exception on a joint-limit violation rather
than printing a warning or silently saturating — a caller cannot
accidentally ignore it the way a print-to-stdout can be scrolled past. Separately,
`libfranka` refuses to talk to a robot whose firmware version doesn't match
what the library was compiled to expect, producing an explicit
"incompatible library version" error rather than proceeding with mismatched
assumptions about what the hardware will do.

**The direct analogy:** the calibration file *is* this arm's "compiled
assumption" about the servos. The bug this session found is structurally
identical to a firmware/library version mismatch — the calibration's travel
numbers were recorded before a re-zero, describing a tick frame the servos
no longer report in, and nothing checked that the assumption was still
current before trusting it.

### 3.4 MoveIt: generate once, only ever narrow

MoveIt's `joint_limits.yaml` is generated as an exact copy of the URDF's
limits by the Setup Assistant; a user may then edit it to be *more*
conservative, never wider. [MoveIt joint limits](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html)
There is exactly one direction a derived config is allowed to move away from
its source.

**Where this stack violates that rule:** `conventions.URDF_LIMITS` and
`soarm_sdk/configs/so101.yaml` are two independently hand-maintained copies
of the same six numbers (verified identical today, by accident of manual
diligence, not by construction). And the planner YAML narrowed past
`safe_planning_bounds()` was fine (§2.3); the version narrowed *past* the
servo EEPROM by more than the calibration knew about (§2.1, §2.2) is exactly
the forbidden direction, and nothing enforced the rule that would have
caught it.

---

## 4. What was fixed this session

- **`ServoHardwareInterface.read_angle_limits()`** — the first thing in this
  codebase to read layer 1 at all.
- **`conventions.phantom_range()`** — diffs what the stack believes (layer 5)
  against what the servo firmware allows (layer 1).
- **`conventions.unreachable_bounds()`** — the dangerous half of
  `stale_bounds()`: bounds too *wide*, which produce silently unexecutable
  plans, as distinct from too *narrow*, which only cost reach.
- **`PlanControls._within_servo_limits()`** — checks a manifest's actual
  waypoints against the live servo limits before a dashboard execute; refuses
  and names the joint rather than letting it stutter.
- **`execute.py` `_Adrift`** — if a joint falls more than 3× `sync_tol`
  behind mid-run, the run now aborts with the joint name and offset instead
  of continuing to stream commands into a gap that cannot close.
- **`wrist_flex` EEPROM cap widened**, 3046 → 3546 ticks, after the true stop
  was measured empirically (walked to it in small steps, watching current,
  never driven into it). Verified end-to-end: two independent TCP plans
  executed clean, 0 limit clamps, 0 step clamps, full-rate execution.
- **Planner YAML and calibration corrected** to match, via `with_travel()` —
  the sanctioned API that updates measured travel without touching the
  pose-anchored zero.
- **`elbow_flex` probed, not changed** — no limit-shaped stop found from the
  reachable test pose; EEPROM left exactly as measured, `(936, 3160)`.

## 5. Recommendations

1. **Fix the gripper the same way** (§2.1) — measure the true closed-jaw
   stop, widen the EEPROM floor to just above it, correct the calibration
   and `GRIPPER_CLOSED_DEG` to match. This is the one item in this document
   that will fail on the next grasp attempt if left alone.

2. **Move `_within_servo_limits()` into `execute.py` itself** (§2.4), so the
   check runs on every path, dashboard or bare CLI — the ros2_control
   pattern of validation the caller cannot opt out of.

3. **Run a phantom-range check at connection time, not just before
   execute** — the UR pattern. The moment `ServoHardwareInterface.start()`
   succeeds, diff the calibration against `read_angle_limits()` and refuse
   to proceed (or at minimum print loudly) on any layer-1 disagreement,
   before a plan is even requested.

4. **Collapse the duplicate URDF limit copies** (§3.4) — `so101.yaml` and
   `conventions.URDF_LIMITS` should be generated from one source, the way
   MoveIt's `joint_limits.yaml` is generated from the URDF rather than
   retyped. Two hand-copies that happen to agree today is not a guarantee
   they agree tomorrow.

5. **Version the calibration against the EEPROM it was measured from** — the
   Franka pattern. Record each joint's EEPROM `MIN/MAX_ANGLE_LIMIT` *inside*
   the calibration file at the time of the ROM sweep. A future
   `phantom_range()` check can then also catch "the servo's own limits
   changed since this calibration was written," not only "the calibration
   never matched the servo," which is the more likely failure mode for a
   hand-modified EEPROM like this session's.

6. **Re-run the ROM sweep for `elbow_flex`** (§2.2) using `rom_sweep`'s wheel
   mode, which drives to an actual stall rather than this session's
   torque-limited incremental probe — the only way to settle whether its
   0.95 rad of phantom range is real.

7. **Adopt MoveIt's one-directional rule explicitly**: any config derived
   from another (planner YAML from `safe_planning_bounds`, `so101.yaml` from
   the URDF) may only be narrowed relative to its source, never widened.
   `unreachable_bounds()` already detects a violation of this rule for the
   planner YAML; the same check is worth generalizing to every derived
   layer in the table in §1.

---

## 6. Implementation

Six changes, each closing one gap from §5, in the order they were built. All
of it lands with backward compatibility as a hard requirement: every
calibration file on disk before this session has neither of the two new
schema fields, and must keep loading and keep working exactly as before
until someone opts a joint in by recording its EEPROM limits.

### 6.1 Extend the calibration schema (closes §5 rec. 5)

`JointCalibration` gains three fields —
[`eeprom_min_ticks`, `eeprom_max_ticks`](../../soarm_sdk/src/soarm_sdk/calibration/frame.py),
`eeprom_recorded_at` — all defaulted so an existing file round-trips through
`RobotCalibration.load()` unchanged. `with_eeprom_limits(min, max)` writes
them, alongside a timestamp, the same shape as every other `with_*` method
on the class. `eeprom_mismatch(live_min, live_max)` compares recorded
against live and returns `None` — deliberately overloaded to mean both "they
agree" and "nothing was ever recorded" — the same gate `measured_is_trusted`
already uses for travel acceptance, so a legacy calibration is inert rather
than broken.

*Justification:* the calibration's `tick_min/tick_max` already records where
the *mechanism* stops; nothing recorded where the *servo's firmware* stops,
which is a different fact and, on this arm, a stricter one. Franka's
firmware/library version check is the direct analogue — a stored assumption
about the hardware, checked against the hardware before it's trusted.

### 6.2 Verify at connection time, not at execute time (closes §5 rec. 3)

`ServoHardwareInterface.start()` now calls a new `_check_eeprom_limits()`
right after the first successful read and *before* torque is enabled — the
safest possible point, since nothing has moved yet. It reads
`read_angle_limits()`, diffs every joint against its calibration's recorded
EEPROM, and **raises** on any disagreement, naming every mismatched joint at
once. A calibration that never recorded EEPROM limits for a joint is
silently exempt — legacy files are not newly blocked.

A `verify_eeprom_limits: bool = True` constructor flag (threaded through
`ServoRobot` too) opts out, for exactly one caller: the widen tool (§6.5)
must be able to connect to a servo *in order to* correct the very mismatch
this check exists to catch.

*Justification:* this is Universal Robots' Recovery-mode pattern —
disagreement between believed and actual state halts the system before any
program runs, rather than being clamped away mid-motion. It is also the
single highest-leverage change in this list: every other fix here still
required a human to notice something was wrong and go looking; this one
means the next `wrist_flex`-shaped bug refuses to connect instead.

### 6.3 Generalize the narrowing check (closes §5 rec. 7)

A new primitive, `conventions.assert_narrows(source, derived, names)`,
states the rule directly: *derived* may claim less range than *source*,
never more. `phantom_range()` (servo vs. the stack's belief) and
`unreachable_bounds()` (the arm vs. the planner YAML) are now both thin
callers of it, sharing one comparison instead of two independent ones that
could silently diverge. `stale_bounds()` is untouched — it reports *both*
directions for a person to read, which `assert_narrows`'s stricter one-way
contract isn't the right shape for.

*Justification:* MoveIt's rule for `joint_limits.yaml` — generated from a
URDF, only ever edited narrower — named as one reusable check rather than
one bespoke comparison per boundary, so a new layer in this stack (§1) gets
this property by calling the primitive, not by remembering to reimplement
it correctly.

### 6.4 Defense in depth on the execute path itself (closes §5 rec. 2)

The manifest-vs-servo comparison that used to live only in the dashboard's
`PlanControls._within_servo_limits()` is now `conventions.waypoints_beyond_servo_limits()`
— one function, called from *both* the dashboard (before it even spawns
`execute.py`) and from `execute.py`'s own `run()` (right after connecting,
before the streaming loop starts). The bare
`python -m soarm_tamp.execute run_dir --port ...` — the exact command used
to verify every fix in this document — now refuses to stream a manifest the
live servos will not obey, with the same per-joint report the dashboard
already gave.

*Justification:* ros2_control's `enforce_command_limits` is global, not
per-caller — the check has to be impossible to bypass by choosing a
different entry point, and until this change that was exactly what the bare
CLI did by accident.

### 6.5 Formalize the correction procedure (closes §5 rec. 1 & partially 6)

`soarm_sdk.calibration.widen_limit` (console script `soarm-widen-limit`)
turns the scratch script this session used on `wrist_flex` into a real,
tested tool: back up the servo's current EEPROM limits to JSON, widen
toward the URDF's own bound (never further) so the mechanism gets the last
word, walk toward it in small steps watching present current, stop the
moment progress stalls or current exceeds a ceiling, set the limit a margin
short of whatever was actually found, lock EEPROM, and record the result
via `with_eeprom_limits()` — so the connect-time check in §6.2 has
something current to check against immediately. Direction-generic (`min`
or `max`), so it is the same tool for `wrist_flex`'s ceiling and any
future joint's floor.

*Justification:* the original fix was correct but existed only as this
conversation's memory and a one-off script. `soarm-widen-limit` is exercised
by four unit tests against a simulated mechanical stop (§6.6) — including a
regression test for a direction-inference bug the tests themselves caught
during development — so the *next* joint gets the same rigor without a
human re-deriving the procedure from a transcript.

### 6.6 Tests

82 new tests, all offline (no serial port): calibration schema round-trips
and backward compatibility, the connect-time check's four outcomes (agree /
mismatch / never-recorded / failed-read), `assert_narrows` and its two
callers against this arm's real numbers, and `probe_and_set_limit` against
a fake servo with a configurable mechanical stop. That fake caught two real
bugs before hardware ever saw this code: a direction-inference heuristic
that broke exactly at the tick equal to the stop (the one value the probe
most needs to get right), and a current-blocked signal that was true for
every step approaching a stop rather than only the step the stop actually
clipped. 554 tests pass across both packages.

---

### 6.7 `shoulder_pan` and `elbow_flex`, attempted

Both `phantom_range` findings from §6.6's validation were run through
`soarm-widen-limit`. Neither resulted in an EEPROM change — for two
different, instructive reasons, both caught by the tool's own safety net
(`final = min(before, stop ± margin)`, never widens past what was already
trusted) rather than by luck:

- **`shoulder_pan`**: the probe found its practical stop at tick 771
  (−1.781 rad) under both a gentle push and, on immediate retry, a firm one
  (`--dq 0.8`) — the same result twice, at different force, is a real
  finding. But the existing EEPROM floor (713, −1.870 rad) already permits
  *more* range than that, so there is no artificial firmware cap here to
  correct — the gap `phantom_range` reports is between the calibration's
  ROM-sweep travel record (wheel-mode, continuous torque, reached 668) and
  what a position-mode probe can independently verify, not a firmware
  restriction. No change needed.
- **`elbow_flex`**: probed from the same FK-clearance-checked pose used for
  the earlier attempt (`shoulder_lift=−1.40`, chosen to keep the gripper off
  the table across the *whole* elbow sweep). At `--dq 0.8` it stalled after
  3 steps having barely moved from its starting position — nowhere near the
  EEPROM floor (−0.477 rad) it was headed for. Table clearance was verified;
  *self*-clearance (the forearm against the upper arm or base at this
  shoulder configuration) was not, and a stall this immediate, this far from
  the target, is the signature of hitting something geometric rather than
  the joint's own limit. Continuing to push harder into an unverified
  self-collision is exactly the wrong response, so the attempt stopped
  there. Servo checked healthy after (36°C, no status fault). This needs a
  pose someone has verified is self-collision-clear through the joint's
  full range — ROM-sweep's wheel mode, or a person watching — not another
  guess from table-clearance FK alone.

---

*Servo EEPROM backups taken this session:*
`servo_eeprom_backup_20260914.json`, `servo_eeprom_backup_elbow_20260914.json`,
`~/.soarm_sdk/calibration.backup-20260914.json`.

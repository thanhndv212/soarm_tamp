# Streaming a trajectory the arm can actually follow

A plan certified collision-free by HPP is not automatically a plan the
servos can track. This is the incident that established why, and the
tuning knobs it produced. The short version lives in the
[README's Execution tuning section](../README.md#execution-tuning); this is
the full account, with the measurements behind each conclusion.

## What happened

The first live run put the hand on the table along a path whose every
waypoint HPP had certified collision-free. Nothing was wrong with the plan.
What was wrong was the assumption that streaming waypoints makes the arm
follow them.

`execute.py` sent a waypoint every tick regardless of where the arm was, and
the SDK clamps each command to `--max-step` of the *measured* position. A
servo covers well under one step per tick, so the arm fell behind on command
one and never recovered, each joint by a different amount — `wrist_flex` had
1.36 rad to travel, `elbow_flex` 0.54. They desynchronised, the arm stopped
tracing the straight line that had been collision-checked, the hand touched
down early, and the contact stalled two joints for the rest of the run.
Final state: two joints 0.5–2.6 rad short of their goals, hand on the table.

## Two independent causes, both now addressed

1. **Nothing waited for the arm.** `--sync` (on by default) holds each
   waypoint until every joint is within `--sync-tol` of it. Lag is then
   bounded by the tolerance instead of growing without limit, and a run
   that genuinely cannot keep up says so — `worst lag` and
   `timed out waiting` in the summary — rather than discovering it by
   contact.
2. **The steps were too small to move the servos.** Measured on
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

## Making it smooth

Waiting at every waypoint works but stutters — the arm stops 7 times in 7
waypoints. Three separate jobs had been collapsed into one number, and
splitting them is what makes the motion flow:

| knob | job | default |
|---|---|---|
| `--max-step` | how finely the path is sampled (fidelity) | 0.02 |
| `--servo-clamp` | how far the command may *lead* the measurement | 0.10 |
| `--sync-tol` | how far the arm may trail the stream | 0.08 |
| `--speed-scale` | GOAL_SPEED as a multiple of the plan's own velocity | 1.5 |

Keeping the tolerance above the sampling step means the arm never has to
stop: it always chases a target a little ahead of it, the lead keeps the
servos above their stiction threshold, and the clamp bounds how far off the
checked path it can get. The velocity feedforward is what stops each command
running its own accel/decel ramp — without it, consecutive commands do not
blend.

Measured across four hardware runs of the same motion:

| run | stopped to wait | settled within | TCP error |
|---|---|---|---|
| stop-at-every-waypoint | 7 of 7 | — | 7.0 mm |
| flowing, no settle | 0 of 57 | — | 10.2 mm |
| flowing + slow settle | 0 of 57 | 0.052 rad | — |
| flowing + settle at speed | **0 of 57** | **0.034 rad** | **9.5 mm** |

## Settle at speed, not slowly

After the last waypoint the arm is still up to `--sync-tol` behind, so the
goal is re-sent until it arrives. The first version crept at 0.15 rad/s to
avoid hunting and left `shoulder_lift` 0.052 rad short every time — a small
error commanded slowly cannot break stiction, while the same 0.05 rad error
at default speed closes and slightly overshoots (ratio 1.197). Same lesson
as the step size, in a different disguise.

## The floor is the arm

Whichever joint carries the load stops 0.03–0.05 rad short and stays there
— `shoulder_lift` (−2.99°) moving one way, `elbow_flex` (−1.97°) the other,
about 1 cm at the TCP either way. That is gravity droop against finite
position-control stiffness: re-commanding does not close it, because the
servo already believes it has arrived. Hence `--settle-tol` defaults to
0.05 rather than pretending tighter is achievable, and closing that last
centimetre needs gravity compensation or feedback from something other than
the servos' own encoders.

## One more trap, fixed

`execute.py` used to stream its first command before the bus had answered.
`ServoHardwareInterface` serves a placeholder 2048 ticks per joint until its
first successful sync-read, so that command was clamped against a fake
"arm is at zero" pose — seen live, `wrist_roll` sitting at −2.71 rad was
commanded toward +0.1 for one tick. It now waits for a real read and
refuses to stream without one.

## Diagnosing a joint that will not follow

`joint_test.py` moves one joint while holding every other at its measured
value:

```bash
python -m soarm_tamp.joint_test --port ... --joint elbow_flex \
    --delta 0.3 --single --hand-is-clear
```

`--single` is the mode that answers the mapping question — one command, no
step clamp, settle, then measured excursion over commanded. Near 1.0 clears
the URDF-to-servo mapping for that joint (measured: shoulder_pan 0.982,
elbow_flex 0.946, wrist_roll 0.992 — the mapping is fine, and the
travel-span mismatch the calibration check flags is not what broke the
run). The laddered mode measures something else entirely — how much of each
step the servo completes before the next command — and must not be read as
a mapping check.

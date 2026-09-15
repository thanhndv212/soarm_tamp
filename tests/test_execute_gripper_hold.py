"""Reproduces the reported bug: the jaw closes, then reopens on the very
next command.

HPP freezes the gripper at one constant value across every waypoint of
every segment — grasp is a rigid constraint to the planner, not something
it plans jaw motion for. ``execute.py`` injects the actual open/close as a
one-off command between segments. Before the fix, the very next segment's
per-waypoint loop re-sent that segment's *own* frozen (still-open) gripper
value on its first command, undoing the close before the arm had moved at
all with anything gripped.

Uses a fake robot that records every commanded position, so the whole
gripper column across the run can be inspected directly — this is not
detectable from the printed "jaw -> CLOSE" line alone, which fires
correctly on both the broken and fixed code.
"""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from soarm_tamp import execute

OPEN_RAD = 0.5167  # matches the real manifest's frozen pre-grasp reading
GRIPPER_IDX = 5


class _RecordingRobot:
    """A robot that arrives instantly and records every commanded position."""

    def __init__(self, port, config, calibration, max_step_rad, enforce_limits):
        del port, config, calibration, max_step_rad, enforce_limits
        self._q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, OPEN_RAD])
        self.joint_names = [
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper",
        ]
        self.commands: list[np.ndarray] = []

        class _HW:
            limit_clamps = 0
            step_clamps = 0

            def read_angle_limits(self) -> dict:
                return {}

        self.hw = _HW()

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def state_age(self) -> float:
        return 0.0

    def get_joint_positions(self) -> np.ndarray:
        return self._q.copy()

    def set_joint_positions(self, target, dq=None) -> None:
        del dq
        self._q = np.asarray(target, dtype=float).copy()
        self.commands.append(self._q.copy())


def _write_manifest(run_dir: Path) -> None:
    """Segment 0 (grasp, triggers a close after it), segment 1 (transit,
    several waypoints, gripper frozen OPEN throughout — exactly the shape
    of the real manifest that exposed the bug)."""
    arm_move = [[0.0] * 5 + [OPEN_RAD], [0.3] * 5 + [OPEN_RAD]]
    (run_dir / "wp0.json").write_text(json.dumps({"waypoints": arm_move}))

    transit = [[0.3] * 5 + [OPEN_RAD], [0.5] * 5 + [OPEN_RAD], [0.7] * 5 + [OPEN_RAD]]
    (run_dir / "wp1.json").write_text(json.dumps({"waypoints": transit}))

    (run_dir / "manifest.json").write_text(json.dumps({
        "segments": [
            {
                "index": 0, "kind": "grasp", "edge_name": "so101/grasp > cube/top | f_01",
                "waypoint_file": "wp0.json", "time_parameterized": False,
            },
            {
                "index": 1, "kind": "transit", "edge_name": "cube/foot > table/spot_b | 0-1",
                "waypoint_file": "wp1.json", "time_parameterized": False,
            },
        ],
        "sampling": {"dt": 0.0},
    }))


def test_the_jaw_stays_closed_through_the_next_segment(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_manifest(run_dir)

    robot_holder: list[_RecordingRobot] = []

    class _Capturing(_RecordingRobot):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            robot_holder.append(self)

    with patch("soarm_sdk.robot.ServoRobot", _Capturing):
        rc = execute.run(
            run_dir, port="/dev/fake", dry_run=False, rate_hz=30.0,
            max_step=0.5,  # coarse: this test is about the gripper column,
            force=True,    # not about resampling fidelity
            sync=False,    # no serial bus to actually arrive against
            settle_tol=0.0,
        )
    assert rc == 0
    robot = robot_holder[0]

    from soarm_tamp.geometry import GRIPPER_CLOSED_DEG
    import math
    closed_rad = math.radians(GRIPPER_CLOSED_DEG)

    # Every command sent during segment 1 — after the close — must carry
    # the closed gripper angle, never the plan's frozen open one.
    seg1_grippers = [round(float(c[GRIPPER_IDX]), 4) for c in robot.commands[2:]]
    assert seg1_grippers, "segment 1 sent no commands to inspect"
    assert all(g == round(closed_rad, 4) for g in seg1_grippers), (
        f"segment 1 reopened the jaw: {seg1_grippers} "
        f"(expected every value to be the closed angle {closed_rad:.4f})"
    )
    assert not any(g == round(OPEN_RAD, 4) for g in seg1_grippers), (
        "found the plan's frozen open value in segment 1 — the exact bug"
    )


def test_trace_reflects_what_was_actually_commanded_not_the_frozen_plan(
    tmp_path: Path,
) -> None:
    """The trace used to log the plan's raw row even after the jaw override
    changed what was actually sent — harmless for the arm joints (identical
    either way), but on the gripper axis it silently hid the very bug above:
    reading the trace showed the plan's frozen value throughout, even once
    the fix made the real command diverge from it."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_manifest(run_dir)

    with patch("soarm_sdk.robot.ServoRobot", _RecordingRobot):
        rc = execute.run(
            run_dir, port="/dev/fake", dry_run=False, rate_hz=30.0,
            max_step=0.5, force=True, sync=False, settle_tol=0.0,
        )
    assert rc == 0

    rows = [
        json.loads(line)
        for line in (run_dir / "live.jsonl").read_text().splitlines()
        if line.strip()
    ]
    seg1_traced = [round(r["q"][GRIPPER_IDX], 4) for r in rows if r.get("seg") == 1]
    assert seg1_traced, "segment 1 traced no waypoints"
    assert not any(g == round(OPEN_RAD, 4) for g in seg1_traced), (
        f"trace shows the plan's frozen open value during segment 1: "
        f"{seg1_traced} — the trace has to show what was actually sent"
    )


class _SlowGripperRobot(_RecordingRobot):
    """Every joint arrives instantly except the gripper, which closes the
    gap by a fixed fraction each tick — cheap stand-in for a servo that
    takes real time to complete a large move.

    ``commands`` (inherited) records the resulting *measured* position after
    each partial step, which is what ``_await_arrival``/``_settle`` actually
    read back — not what was asked for. ``requested`` records the raw
    target argument every call, which is what a retry loop holding one goal
    should show as constant.
    """

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.requested: list[np.ndarray] = []

    def set_joint_positions(self, target, dq=None) -> None:
        del dq
        target = np.asarray(target, dtype=float)
        self.requested.append(target.copy())
        new = target.copy()
        new[GRIPPER_IDX] = self._q[GRIPPER_IDX] + 0.15 * (
            target[GRIPPER_IDX] - self._q[GRIPPER_IDX]
        )
        self._q = new
        self.commands.append(self._q.copy())


def test_jaw_action_waits_for_the_gripper_to_actually_arrive(tmp_path: Path) -> None:
    """A flat sleep(0.6) after commanding the jaw doesn't know whether a
    large swing actually completed. With a gripper that visibly takes many
    ticks to close, the run must keep re-commanding it (via _settle) rather
    than moving on after one shot."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_manifest(run_dir)

    robot_holder: list[_SlowGripperRobot] = []

    class _Capturing(_SlowGripperRobot):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            robot_holder.append(self)

    with patch("soarm_sdk.robot.ServoRobot", _Capturing):
        rc = execute.run(
            run_dir, port="/dev/fake", dry_run=False, rate_hz=30.0,
            max_step=0.5, force=True, sync=False,
            settle_tol=0.01, settle_timeout=2.0,
        )
    assert rc == 0
    robot = robot_holder[0]

    # Find the jaw command (the one setting the gripper axis alone, target
    # equal to closed_rad while the arm axes stay put) and confirm several
    # further commands to the SAME target followed it — that is _settle
    # retrying, not a one-shot command trusted on faith.
    from soarm_tamp.geometry import GRIPPER_CLOSED_DEG
    import math
    closed_rad = math.radians(GRIPPER_CLOSED_DEG)

    # The jaw action's own first request for the closed angle...
    jaw_idx = next(
        i for i, r in enumerate(robot.requested)
        if round(float(r[GRIPPER_IDX]), 4) == round(closed_rad, 4)
    )
    retries = robot.requested[jaw_idx:]
    assert len(retries) > 2, (
        f"only {len(retries)} request(s) asked for the closed angle — a "
        "gripper converging 15%/tick from 0.52 to 0.09 rad needs several "
        "retries to get within tolerance, not one shot trusted on faith"
    )
    # ...held constant across every retry: _settle repeating one goal,
    # not the plan moving on and asking for something new each time.
    assert {round(float(r[GRIPPER_IDX]), 6) for r in retries} == {round(closed_rad, 6)}
    # And the measured position actually converges over those retries,
    # proving this is a real wait-for-arrival, not a no-op loop.
    measured = [float(c[GRIPPER_IDX]) for c in robot.commands[jaw_idx:jaw_idx + len(retries)]]
    assert measured[-1] < measured[0], "gripper never actually closed further while waiting"
    assert abs(measured[-1] - closed_rad) < 0.02, "did not converge within settle_tol"

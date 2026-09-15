"""Reproduces the reported failure: a joint that cannot move streams
further and further behind the plan, spamming step-clamp warnings, instead
of the run stopping.

Uses a fake robot whose measured position never advances no matter what is
commanded — the hardware analogue of a joint stalled under load or
disconnected — so the test needs no serial port and is deterministic.
"""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from soarm_tamp import execute


class _FrozenRobot:
    """A robot whose measured position never moves, regardless of commands.

    Standing in for a joint stalled under load (e.g. gravity torque a
    lifted cube adds at some pose) or a bad connection: every command is
    accepted, nothing happens, ``get_joint_positions`` always reports the
    same pose it started at.
    """

    def __init__(self, port, config, calibration, max_step_rad, enforce_limits):
        del port, config, calibration, max_step_rad, enforce_limits
        self._q = np.zeros(6)
        self.joint_names = [
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper",
        ]

        class _HW:
            limit_clamps = 0
            step_clamps = 0

            def read_angle_limits(self) -> dict:
                # No live servo limits known to this fake — the new
                # servo-limit preflight in execute.py must no-op rather
                # than block, same as when a real read fails.
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
        del target, dq  # commanded, but the joint never actually gets there


def _write_manifest(run_dir: Path) -> None:
    """One segment, a straight 1.0 rad move every joint cannot complete."""
    wp_path = run_dir / "wp0.json"
    wp_path.write_text(json.dumps({"waypoints": [[0.0] * 6, [1.0] * 6]}))
    (run_dir / "manifest.json").write_text(json.dumps({
        "segments": [{
            "index": 0,
            "kind": "move",
            "edge_name": "stalled_joint",
            "waypoint_file": "wp0.json",
            "time_parameterized": False,
        }],
        "sampling": {"dt": 0.0},
    }))


def test_stalled_joint_aborts_instead_of_drifting_forever(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_manifest(run_dir)

    with patch("soarm_sdk.robot.ServoRobot", _FrozenRobot):
        rc = execute.run(
            run_dir,
            port="/dev/fake",
            dry_run=False,
            rate_hz=30.0,
            max_step=0.02,
            force=True,  # no calibration in this sandbox; irrelevant to the bug
            sync=True,
            sync_tol=0.08,
            sync_timeout=0.02,  # keep the test fast; behavior is timeout-independent
            servo_clamp=0.1,
            speed_scale=1.5,
            settle_tol=0.0,
            settle_timeout=0.1,
            plan_timing=False,
        )

    # Before the fix this returned 0 having streamed all 50 resampled
    # waypoints of the 1.0 rad move into a joint that measurably never
    # moved, growing to a full radian of unclosed gap while logging a
    # "step clamp" warning on nearly every one of them — exactly the
    # symptom reported from hardware. The fix stops the run once the gap
    # passes 3x sync_tol (0.24 rad) rather than continuing to add distance
    # to a gap that was never going to close.
    assert rc == 3

    live = json.loads(
        "[" + ",".join((run_dir / "live.jsonl").read_text().splitlines()) + "]"
    )
    commands = [r for r in live if "q" in r]
    # 0.24 rad at 0.02 rad/waypoint is 12 steps; allow a little slack for
    # the exact rounding, but this must stop well short of all 50.
    assert 10 <= len(commands) <= 16, (
        f"expected the run to abort after ~12 waypoints, sent {len(commands)}"
    )

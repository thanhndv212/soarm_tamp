"""Shared wiring for the planning tabs.

Both tabs do the same three things — plan in the container, look at it in
the viewer, run it on the arm — so the differences stay in the panel and
the plumbing lives here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, List, Optional

from ..container import ContainerJob, available, kill_in_container
from ..player import ManifestPlayer
from ..runner import ExecutionJob

__all__ = ["Console", "PlanControls", "VIEWER_PATTERN"]

#: Matches the viewer process *inside* the container, not the docker
#: exec client on this side of it.
VIEWER_PATTERN = "soarm_tamp.replay"

#: Where the container-side viewer publishes.
VIEWER_PORT = 8000


def _wait_for_port_free(timeout: float = 8.0) -> bool:
    """Block until nothing is listening on the viewer port."""
    import socket
    import time

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with socket.socket() as sk:
            sk.settimeout(0.3)
            if sk.connect_ex(("127.0.0.1", VIEWER_PORT)) != 0:
                return True
        time.sleep(0.3)
    return False


class Console:
    """A bounded scrollback rendered into one markdown handle."""

    def __init__(self, handle: Any, limit: int = 14) -> None:
        self._handle = handle
        self._limit = limit
        self._lines: List[str] = []

    def clear(self) -> None:
        self._lines.clear()
        self._render()

    def say(self, line: str) -> None:
        self._lines.append(line)
        del self._lines[: max(0, len(self._lines) - self._limit)]
        self._render()

    def _render(self) -> None:
        body = "\n".join(self._lines) if self._lines else "*idle*"
        try:
            self._handle.content = f"```\n{body}\n```"
        except Exception:
            pass


class PlanControls:
    """Plan / replay / execute for one run directory."""

    def __init__(
        self,
        server: Any,
        ctx: Any,
        console: Console,
        *,
        run_dir: Path,
        fk_update: Optional[Callable] = None,
    ) -> None:
        self.server = server
        self.ctx = ctx
        self.console = console
        self.run_dir = Path(run_dir)
        self.fk_update = fk_update
        self.plan_job: Optional[ContainerJob] = None
        self.replay_job: Optional[ContainerJob] = None
        self.exec_job: Optional[ExecutionJob] = None
        self.player: Optional[ManifestPlayer] = None

    # -- the mirror and the file must agree -----------------------------

    def _calibration_synced(self, action: str) -> bool:
        """Refuse *action* while the 3-D view and the saved calibration differ.

        Everything below this line converts between ticks and URDF radians,
        and none of it reads the calibration the mirror is rendering. The
        planner runs in a container and can see only the file; ``capture``,
        ``ExecutionJob`` and ``ManifestPlayer`` each load their own copy from
        it. So an unsaved edit does not mean "not applied yet" — it means the
        arm you are watching and the arm being planned for are different
        arms, and the mirror is the half that looks right.

        Better to stop here than to produce a trajectory that is collision-
        checked against a pose the arm was never in.
        """
        try:
            drift = self.ctx.calibration_drift()
        except Exception:
            return True  # an older context with no such notion; do not block
        if not drift:
            return True
        detail = ", ".join(f"{n} {d:+.1f}°" for n, d in drift)
        self.console.say(f"cannot {action}: the calibration has unsaved changes")
        self.console.say(f"  the 3-D view is showing {detail} vs. the saved file")
        self.console.say("  Save it in the Calibration tab, then try again")
        return False

    def _bounds_current(self) -> bool:
        """Warn when the planner's YAML bounds no longer match the arm.

        A warning rather than a refusal: unlike an unsaved calibration,
        stale bounds do not make the plan describe a different arm, they
        make it reach for angles this one cannot hold. That is worth
        stopping for when it is too wide and merely wasteful when it is too
        narrow, and the message says which — but a run that is deliberately
        conservative is still a run worth having.
        """
        try:
            from ...conventions import calibration_path, stale_bounds
            from soarm_sdk.calibration.frame import RobotCalibration

            cfg = Path(__file__).resolve().parents[2] / "config" / "cube_pick_place.yaml"
            if not cfg.exists():
                return True
            bad = stale_bounds(RobotCalibration.load(calibration_path()), cfg)
        except Exception:
            return True
        if not bad:
            return True
        self.console.say("WARNING: planner bounds are stale vs. this calibration")
        for line in bad:
            self.console.say(f"  {line}")
        self.console.say("  regenerate with conventions.format_bounds_yaml()")
        return False

    # -- starting where the arm actually is -----------------------------

    def capture_start(self, start_file: Path) -> bool:
        """Write the arm's measured pose for the container-side planner.

        A plan that begins at the model's zero pose leaves the servos to
        slew there first, along a path no collision checker ever saw. Both
        planners take ``--start`` to avoid that; this is the host half.

        Needs the serial port, which the live mirror is holding, so the
        mirror stands down for the read and comes straight back.
        """
        if not self._calibration_synced("capture a start pose"):
            return False

        import json
        import math

        from ...conventions import URDF_LIMITS
        from ...read_pose import capture

        port = self.ctx.device_h.value
        if not port:
            self.console.say("no serial port set in the sidebar")
            return False
        was_polling = bool(getattr(self.ctx, "polling", False))
        try:
            if was_polling:
                self.ctx.stop_polling()
            pose = capture(port, 0.3)
            start_file = Path(start_file)
            start_file.parent.mkdir(parents=True, exist_ok=True)
            start_file.write_text(json.dumps(pose, indent=1) + "\n")

            # A limp arm rests where the URDF says it cannot be, and the
            # planner refuses such a pose rather than clamping it. Say so
            # here, where it is fixable, not in the planner's output.
            bad = [
                f"{n} {math.degrees(v):+.1f} deg"
                for n, v in zip(pose["joint_names"], pose["q"])
                if not URDF_LIMITS[n][0] <= v <= URDF_LIMITS[n][1]
            ]
            self.console.say(f"captured pose -> {start_file.name}")
            if bad:
                self.console.say("OUT OF BOUNDS: " + ", ".join(bad))
                self.console.say("lift the arm into range; planning will refuse this")
                return False
            return True
        except Exception as exc:
            self.console.say(f"capture failed: {exc}")
            return False
        finally:
            if was_polling:
                self.ctx.start_polling()

    # -- planning ------------------------------------------------------

    def plan(self, args: List[str], *, label: str) -> bool:
        if not self._calibration_synced("plan"):
            return False
        self._bounds_current()  # warns into the console; does not block
        ok, why = available()
        if not ok:
            self.console.say(f"cannot plan: {why}")
            return False
        if self.plan_job is not None and self.plan_job.running:
            self.console.say("a plan is already running")
            return False
        self.console.clear()
        self.console.say(f"{label} ...")
        self.plan_job = ContainerJob(
            args,
            on_line=self._plan_line,
            on_done=lambda rc: self.console.say(
                "PLANNING SUCCEEDED" if rc == 0 else f"planning failed (rc={rc})"
            ),
        )
        self.plan_job.start()
        return True

    def _plan_line(self, line: str) -> None:
        # The planner is chatty at INFO; surface the parts a person watching
        # a button actually wants, not the whole log.
        keep = (
            "SUCCESS", "FAILED", "Path found", "PLANNING", "recorded",
            "manifest", "Traceback", "Error", "error", "refus", "outside",
            "goal error", "clamped", "seam",
        )
        if any(k in line for k in keep):
            self.console.say(line.strip()[:150])

    # -- playback in this dashboard's own 3-D view ----------------------

    def play(self, *, speed: float = 1.0, loop: bool = False) -> None:
        """Animate the manifest in the view already on this page.

        No second viser, no second port, no container: the manifest is JSON
        on this side and the arm is already loaded here. It also means the
        replayed pose and the measured pose go through exactly the same
        calibrated tick pipeline, which is what makes comparing them mean
        anything.
        """
        if not self._calibration_synced("play"):
            return
        if self.fk_update is None:
            self.console.say("no 3-D view attached to this panel")
            return
        if not (self.run_dir / "manifest.json").exists():
            self.console.say(f"no manifest at {self.run_dir.name} — plan one first")
            return
        if self.player is not None and self.player.running:
            self.console.say("already playing (stop it first)")
            return
        self.player = ManifestPlayer(
            self.run_dir,
            self.fk_update,
            joint_ids=list(self.ctx.joint_ids),
            speed=speed,
            loop=loop,
            on_line=self.console.say,
        )
        self.player.start()

    def stop_play(self) -> None:
        if self.player is not None and self.player.running:
            self.player.stop()
        else:
            self.console.say("nothing playing")

    # -- the container's full-scene viewer (cube, table, grasp frames) ---

    def replay(self, *, follow: bool = False) -> None:
        if not self.run_dir.exists():
            self.console.say(f"no run at {self.run_dir} — plan one first")
            return
        if self.replay_job is not None and self.replay_job.running:
            self.console.say("viewer already running (stop it first)")
            return
        # A viewer from an earlier session may still be alive inside the
        # container holding port 8000, in which case this one binds nothing
        # and dies. Clear the ground first.
        stale = kill_in_container(VIEWER_PATTERN)
        if stale:
            # pkill returns before the process dies, and the new viewer
            # cannot bind :8000 until the old one has actually let go.
            # Starting immediately loses that race and fails with a
            # traceback that says nothing about why.
            self.console.say(f"cleared {stale} stale viewer process(es); waiting")
            _wait_for_port_free()
        args = ["replay", "--run", str(self.run_dir)]
        if follow:
            args.append("--follow")
        self.replay_job = ContainerJob(
            args, on_line=self._replay_line, kill_pattern=VIEWER_PATTERN
        )
        self.replay_job.start()
        self.console.say("viewer starting — open http://localhost:8000")

    def _replay_line(self, line: str) -> None:
        if "8000" in line or "OPEN THIS" in line or "Traceback" in line:
            self.console.say(line.strip()[:150])

    def stop_replay(self) -> None:
        # Always reach into the container, even when this panel has no job
        # of its own: the viewer that is holding port 8000 may have been
        # started by an earlier dashboard, or from a shell.
        if self.replay_job is not None and self.replay_job.running:
            killed = self.replay_job.stop()
        else:
            killed = kill_in_container(VIEWER_PATTERN)
        self.console.say(
            f"viewer stopped ({killed} process(es))" if killed else "no viewer running"
        )

    # -- execution -----------------------------------------------------

    def execute(self, *, dry_run: bool, port: Optional[str], **kwargs: Any) -> None:
        if not self._calibration_synced("execute"):
            return
        if not (self.run_dir / "manifest.json").exists():
            self.console.say(f"no manifest at {self.run_dir} — plan one first")
            return
        if self.exec_job is not None and self.exec_job.running:
            self.console.say("an execution is already running")
            return
        if not dry_run and not port:
            self.console.say("no serial port set in the sidebar")
            return
        self.exec_job = ExecutionJob(
            self.ctx,
            self.run_dir,
            port=port,
            dry_run=dry_run,
            fk_update=self.fk_update,
            joint_ids=list(self.ctx.joint_ids),
            on_line=self.console.say,
            on_done=lambda rc: self.console.say(
                "execution finished" if rc == 0 else f"execution returned {rc}"
            ),
            **kwargs,
        )
        self.exec_job.start()

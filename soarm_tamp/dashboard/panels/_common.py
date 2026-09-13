"""Shared wiring for the planning tabs.

Both tabs do the same three things — plan in the container, look at it in
the viewer, run it on the arm — so the differences stay in the panel and
the plumbing lives here.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, List, Optional

from ..container import ContainerJob, available, kill_in_container
from ..runner import ExecutionJob

__all__ = ["Console", "PlanControls", "VIEWER_PATTERN"]

#: Matches the viewer process *inside* the container, not the docker
#: exec client on this side of it.
VIEWER_PATTERN = "soarm_tamp.replay"


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

    # -- planning ------------------------------------------------------

    def plan(self, args: List[str], *, label: str) -> bool:
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

    # -- viewer --------------------------------------------------------

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
            self.console.say(f"cleared {stale} stale viewer process(es)")
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

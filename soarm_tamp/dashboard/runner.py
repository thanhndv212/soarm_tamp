"""Execute a manifest on the arm from inside the dashboard process.

The serial port is exclusive and the dashboard's live mirror is holding it.
``execute.run()`` opens its own, so the two cannot overlap: the mirror is
stopped for the duration and restarted afterwards, leaving exactly one
owner of the bus at any moment.

Losing the mirror during the one part anybody wants to watch would be a
poor trade, so while execute has the port this follows ``<run>/live.jsonl``
— the trace execute appends every command to — and drives the 3-D view
from that instead. Same picture, no second connection.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

__all__ = ["ExecutionJob"]


class ExecutionJob:
    """Run one manifest, keeping the 3-D view fed from the command trace."""

    def __init__(
        self,
        ctx: Any,
        run_dir: Path,
        *,
        port: Optional[str],
        dry_run: bool,
        fk_update: Optional[Callable[[Dict[int, int]], None]] = None,
        joint_ids: Optional[List[int]] = None,
        on_line: Optional[Callable[[str], None]] = None,
        on_done: Optional[Callable[[int], None]] = None,
        **execute_kwargs: Any,
    ) -> None:
        self._ctx = ctx
        self.run_dir = Path(run_dir)
        self._port = port
        self._dry_run = dry_run
        self._fk_update = fk_update
        self._joint_ids = joint_ids or [1, 2, 3, 4, 5, 6]
        self._on_line = on_line
        self._on_done = on_done
        self._kwargs = execute_kwargs
        self.returncode: Optional[int] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            raise RuntimeError("an execution is already running")
        self._stop.clear()
        self.returncode = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _say(self, msg: str) -> None:
        if self._on_line is not None:
            try:
                self._on_line(msg)
            except Exception:
                pass

    def _run(self) -> None:
        from ..execute import run as execute_run

        trace = self.run_dir / "live.jsonl"
        # Start clean: a trace left by a previous run would be replayed as
        # though it were this one's.
        try:
            if trace.exists():
                trace.unlink()
        except OSError:
            pass

        was_polling = False
        try:
            if not self._dry_run:
                was_polling = bool(getattr(self._ctx, "polling", False))
                if was_polling:
                    self._say("releasing the serial port for execute ...")
                    self._ctx.stop_polling()

            follower = threading.Thread(target=self._follow, args=(trace,), daemon=True)
            follower.start()

            self._say(f"executing {self.run_dir.name} "
                      f"({'dry run' if self._dry_run else 'LIVE'}) ...")
            rc = execute_run(
                self.run_dir,
                None if self._dry_run else self._port,
                self._dry_run,
                **self._kwargs,
            )
            self.returncode = int(rc)
            self._say(f"execute returned {rc}")
        except Exception as exc:
            self.returncode = -1
            self._say(f"FAILED: {exc}")
        finally:
            self._stop.set()
            if was_polling:
                self._say("restoring the live mirror ...")
                try:
                    self._ctx.start_polling()
                except Exception as exc:  # pragma: no cover
                    self._say(f"could not restart polling: {exc}")
            if self._on_done is not None:
                try:
                    self._on_done(self.returncode if self.returncode is not None else -1)
                except Exception:
                    pass

    def _follow(self, trace: Path) -> None:
        """Tail the command trace and push each command through FK."""
        if self._fk_update is None:
            return
        from ..conventions import JOINT_ORDER
        from soarm_sdk.calibration.frame import RobotCalibration
        from ..conventions import calibration_path

        try:
            cal = RobotCalibration.load(calibration_path())
        except Exception:
            return
        by_name = {j.name: j for j in cal.joints}

        pos = 0
        while not self._stop.is_set():
            if not trace.exists():
                self._stop.wait(0.05)
                continue
            try:
                with trace.open() as fh:
                    fh.seek(pos)
                    chunk = fh.readlines()
                    pos = fh.tell()
            except OSError:
                self._stop.wait(0.05)
                continue
            if not chunk:
                self._stop.wait(0.02)
                continue
            # Only the newest command matters: rendering every one of a
            # backlog just walks the view through history at the wrong speed.
            for raw in reversed(chunk):
                try:
                    rec = json.loads(raw)
                except Exception:
                    continue
                q = rec.get("q") or rec.get("target") or rec.get("command")
                if not q:
                    continue
                ticks = {}
                for sid, name, value in zip(self._joint_ids, JOINT_ORDER, q):
                    j = by_name.get(name)
                    if j is not None:
                        ticks[sid] = int(round(j.to_ticks(float(value))))
                if ticks:
                    try:
                        self._fk_update(ticks)
                    except Exception:
                        pass
                break

"""Execute a manifest on the arm from inside the dashboard process.

The serial port is exclusive and the dashboard's live mirror is holding it.
``execute.run()`` opens its own, so the two cannot overlap: the mirror is
stopped for the duration and restarted afterwards, leaving exactly one
owner of the bus at any moment.

Losing the mirror during the one part anybody wants to watch would be a
poor trade, so while execute has the port this follows ``<run>/live.jsonl``
— the trace execute appends every command to — and drives the 3-D view
from that instead. Same picture, no second connection.

``execute.run()`` itself runs in a **subprocess**, not a thread in this
process. It used to be an in-process function call on a background
thread, sharing the GIL with this dashboard's Viser server and with the
thread tailing the trace for the mirror. That coupling was invisible until
someone watched closely: ``execute.run()``'s sync loop polls the servo bus
every 5 ms and paces every command against a wall-clock deadline, and
those are exactly the timings a few milliseconds of FK math and Viser
network sends — done from another thread, but still the same interpreter —
can blur. On hardware that showed up as the mirror stuttering and the arm
itself moving less smoothly than the same manifest run from a plain
terminal. A subprocess gives ``execute.run()`` its own GIL, so nothing
this process does can perturb its timing; the trace file is still the only
thing that crosses the boundary, same as before.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

__all__ = ["ExecutionJob"]

#: soarm_tamp/soarm_tamp/dashboard/runner.py -> soarm_tamp/ (the outer repo
#: dir, on sys.path so ``-m soarm_tamp.execute`` resolves regardless of how
#: this process itself was started).
_SOARM_TAMP_ROOT = Path(__file__).resolve().parents[2]

#: How often the mirror is allowed to repaint from the trace, in seconds.
#: The trace can grow much faster than this (every command, ~30/s); Viser
#: repaints are not free, and the 3-D view does not need to redraw faster
#: than a person can see it move to look smooth. Bounding it here — rather
#: than at the source — keeps the trace itself a complete, high-rate record
#: for anything else that wants to read it.
_MIRROR_MIN_INTERVAL_S = 1.0 / 15.0


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
        self._proc: Optional[subprocess.Popen] = None
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

    def stop(self) -> None:
        """Terminate the execute subprocess, if one is running.

        There was no way to do this safely before: a ``KeyboardInterrupt``
        only ever reaches the main thread, so an in-process
        ``execute.run()`` running on a background thread could not be
        interrupted at all. A real subprocess can just be terminated.
        """
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()

    def _say(self, msg: str) -> None:
        if self._on_line is not None:
            try:
                self._on_line(msg)
            except Exception:
                pass

    def _argv(self) -> List[str]:
        """Build the ``python -m soarm_tamp.execute`` command line.

        Mirrors ``execute.py``'s own argparse flags one-for-one so this
        stays a thin translation of *execute_kwargs*, not a second place
        that has opinions about defaults — those stay in ``execute.py``.
        """
        argv = [
            sys.executable, "-m", "soarm_tamp.execute", str(self.run_dir.resolve()),
        ]
        if self._dry_run:
            argv.append("--dry-run")
        elif self._port:
            argv += ["--port", self._port]

        k = self._kwargs
        if "rate_hz" in k:
            argv += ["--rate", str(k["rate_hz"])]
        if "max_step" in k:
            argv += ["--max-step", str(k["max_step"])]
        if k.get("force"):
            argv.append("--force")
        if k.get("pace"):
            argv.append("--pace")
        if k.get("trace_enabled") is False:
            argv.append("--no-trace")
        if k.get("sync") is False:
            argv.append("--no-sync")
        if "sync_tol" in k:
            argv += ["--sync-tol", str(k["sync_tol"])]
        if "sync_timeout" in k:
            argv += ["--sync-timeout", str(k["sync_timeout"])]
        if "servo_clamp" in k:
            argv += ["--servo-clamp", str(k["servo_clamp"])]
        if "speed_scale" in k:
            argv += ["--speed-scale", str(k["speed_scale"])]
        if "settle_tol" in k:
            argv += ["--settle-tol", str(k["settle_tol"])]
        if "settle_timeout" in k:
            argv += ["--settle-timeout", str(k["settle_timeout"])]
        if k.get("plan_timing") is False:
            argv.append("--no-plan-timing")
        return argv

    def _run(self) -> None:
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

            env = os.environ.copy()
            env["PYTHONPATH"] = os.pathsep.join(
                [str(_SOARM_TAMP_ROOT), env.get("PYTHONPATH", "")]
            ).rstrip(os.pathsep)

            self._proc = subprocess.Popen(
                self._argv(),
                cwd=str(_SOARM_TAMP_ROOT),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert self._proc.stdout is not None
            for line in self._proc.stdout:
                self._say(line.rstrip("\n"))
            rc = self._proc.wait()
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
        """Tail the command trace and push each command through FK.

        Rate-limited to :data:`_MIRROR_MIN_INTERVAL_S`: the trace can grow
        much faster than the view needs to repaint, and pushing every line
        the instant it lands bought nothing visually while adding FK work
        and Viser sends on top of whatever ``execute.run()`` (now a
        subprocess, so no longer sharing this process's GIL either way, but
        still no reason to spend the cycles) is doing.
        """
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
        last_push = 0.0
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
            now = time.monotonic()
            wait_left = _MIRROR_MIN_INTERVAL_S - (now - last_push)
            if wait_left > 0:
                # Sleep off the remainder rather than spinning back to the
                # top of the loop: pos has already advanced past this
                # backlog, so looping immediately would just reopen and
                # reseek the file over and over until the window opens.
                self._stop.wait(wait_left)
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
                        last_push = now
                    except Exception:
                        pass
                break

"""Run planning commands in the HPP container from the host dashboard.

``scripts/hpp_container.sh`` already owns creating and reusing the
container; this only wraps it so a GUI thread can stream its output and
know when it finished, without blocking the Viser main loop.
"""

from __future__ import annotations

import os
import subprocess
import threading
from pathlib import Path
from typing import Callable, List, Optional, Sequence

__all__ = ["SCRIPT", "ContainerJob", "available", "container_name", "kill_in_container"]

#: soarm_tamp/scripts/hpp_container.sh, resolved from this file.
SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "hpp_container.sh"


def container_name() -> str:
    """Match the name hpp_container.sh uses, override included."""
    return os.environ.get("SOARM_TAMP_CONTAINER", "hpp-soarm-tamp")


def kill_in_container(pattern: str) -> int:
    """``pkill -f pattern`` inside the container. Returns processes matched.

    Terminating the host-side ``docker exec`` does **not** stop what it
    started: the client goes away and the process inside keeps running,
    still holding whatever it had — for the viewer, port 8000, which then
    makes every later replay fail to bind. The kill has to happen on the
    other side of the container boundary.
    """
    name = container_name()
    try:
        before = subprocess.run(
            ["docker", "exec", name, "pgrep", "-fc", pattern],
            capture_output=True, text=True, timeout=10, check=False,
        )
        n = int((before.stdout or "0").strip() or 0)
    except Exception:
        n = 0
    try:
        subprocess.run(
            ["docker", "exec", name, "pkill", "-f", pattern],
            capture_output=True, timeout=10, check=False,
        )
    except Exception:
        pass
    return n


def available() -> tuple[bool, str]:
    """Whether the container script and docker are usable."""
    if not SCRIPT.exists():
        return False, f"missing {SCRIPT}"
    try:
        p = subprocess.run(
            ["docker", "info"], capture_output=True, timeout=10, check=False
        )
    except FileNotFoundError:
        return False, "docker is not on PATH"
    except subprocess.TimeoutExpired:
        return False, "docker did not respond"
    if p.returncode != 0:
        return False, "docker is not running"
    return True, "ready"


class ContainerJob:
    """One ``hpp_container.sh`` invocation, run on its own thread.

    Output lines are handed to *on_line* as they arrive so a panel can show
    progress; planning a pick-and-place takes a few seconds and a frozen
    button for that long reads as a hang.
    """

    def __init__(
        self,
        args: Sequence[str],
        *,
        on_line: Optional[Callable[[str], None]] = None,
        on_done: Optional[Callable[[int], None]] = None,
        cwd: Optional[Path] = None,
        kill_pattern: Optional[str] = None,
    ) -> None:
        self.args = list(args)
        #: What to pkill inside the container when stopping. Without it,
        #: stop() only detaches the client and leaves the real process up.
        self.kill_pattern = kill_pattern
        self._on_line = on_line
        self._on_done = on_done
        self._cwd = cwd or SCRIPT.parent.parent
        self.lines: List[str] = []
        self.returncode: Optional[int] = None
        self._proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            raise RuntimeError("job already running")
        self.lines.clear()
        self.returncode = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        cmd = [str(SCRIPT), *self.args]
        try:
            self._proc = subprocess.Popen(
                cmd,
                cwd=str(self._cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert self._proc.stdout is not None
            for line in self._proc.stdout:
                line = line.rstrip("\n")
                self.lines.append(line)
                if self._on_line is not None:
                    try:
                        self._on_line(line)
                    except Exception:
                        pass
            self.returncode = self._proc.wait()
        except Exception as exc:  # pragma: no cover - environment dependent
            self.lines.append(f"[dashboard] {exc}")
            self.returncode = -1
        finally:
            if self._on_done is not None:
                try:
                    self._on_done(self.returncode if self.returncode is not None else -1)
                except Exception:
                    pass

    def stop(self) -> int:
        """Stop the command, on both sides of the container boundary.

        Returns how many container-side processes were matched, so a caller
        can tell "stopped it" from "there was nothing to stop".
        """
        killed = 0
        if self.kill_pattern:
            killed = kill_in_container(self.kill_pattern)
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
        return killed

    def tail(self, n: int = 12) -> str:
        return "\n".join(self.lines[-n:])

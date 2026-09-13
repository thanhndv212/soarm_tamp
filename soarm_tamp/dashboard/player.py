"""Play a manifest into the dashboard's own 3-D view.

``soarm_tamp.replay`` renders inside the container, which means a second
viser server on a second port, a second scene load, and a process on the
far side of the container boundary that outlives the client that started
it. None of that is needed to watch the arm move: the manifest is JSON on
this side, and the dashboard already has the SO-101 loaded and a calibrated
tick pipeline feeding it.

So this reads the waypoints directly and drives the same view the live arm
drives. One port, one scene, no container — and the replayed pose and the
measured pose are rendered by exactly the same code, which is what makes
comparing them meaningful.

The container-side viewer keeps its place for the full planning scene
(cube, table, grasp frames); this is for the arm's motion.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

__all__ = ["ManifestPlayer", "load_segments", "ticks_for"]


def load_segments(run_dir: Path) -> List[dict]:
    """Manifest segments in order, each with its waypoints loaded."""
    run_dir = Path(run_dir)
    manifest = json.loads((run_dir / "manifest.json").read_text())
    out: List[dict] = []
    for seg in manifest.get("segments", []):
        name = seg.get("waypoint_file") or seg.get("path_file")
        if not name:
            continue
        data = json.loads((run_dir / name).read_text())
        wps = data.get("waypoints") or []
        if not wps:
            continue
        out.append(
            {
                "label": seg.get("edge_name") or seg.get("step_label") or "segment",
                "kind": seg.get("kind", ""),
                "dt": float(seg.get("dt") or manifest.get("sampling", {}).get("dt", 0.05)),
                "waypoints": wps,
            }
        )
    return out


def ticks_for(
    cal: Any, joint_ids: Sequence[int], names: Sequence[str], q: Sequence[float]
) -> Dict[int, int]:
    """Map a configuration's arm joints to servo ticks for the 3-D view."""
    by_name = {j.name: j for j in cal.joints}
    ticks: Dict[int, int] = {}
    for sid, name, value in zip(joint_ids, names, q):
        joint = by_name.get(name)
        if joint is not None:
            ticks[sid] = int(round(joint.to_ticks(float(value))))
    return ticks


class ManifestPlayer:
    """Animate a manifest through a callback, on its own thread."""

    def __init__(
        self,
        run_dir: Path,
        fk_update: Callable[[Dict[int, int]], None],
        *,
        joint_ids: Sequence[int],
        speed: float = 1.0,
        loop: bool = False,
        on_line: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.run_dir = Path(run_dir)
        self._fk = fk_update
        self._joint_ids = list(joint_ids)
        self.speed = max(0.05, float(speed))
        self.loop = loop
        self._on_line = on_line
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _say(self, msg: str) -> None:
        if self._on_line is not None:
            try:
                self._on_line(msg)
            except Exception:
                pass

    def start(self) -> None:
        if self.running:
            raise RuntimeError("already playing")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        from soarm_sdk.calibration.frame import RobotCalibration

        from ..conventions import JOINT_ORDER, calibration_path

        try:
            segments = load_segments(self.run_dir)
        except Exception as exc:
            self._say(f"cannot read {self.run_dir.name}: {exc}")
            return
        if not segments:
            self._say(f"no waypoints in {self.run_dir.name}")
            return
        try:
            cal = RobotCalibration.load(calibration_path())
        except Exception as exc:
            self._say(f"no calibration: {exc}")
            return

        total = sum(len(s["waypoints"]) for s in segments)
        self._say(f"playing {self.run_dir.name}: {len(segments)} segment(s), "
                  f"{total} waypoints at {self.speed:g}x")
        while not self._stop.is_set():
            for seg in segments:
                if self._stop.is_set():
                    break
                self._say(f"  [{seg['kind'] or 'seg'}] {seg['label']} "
                          f"({len(seg['waypoints'])} pts)")
                period = seg["dt"] / self.speed
                for q in seg["waypoints"]:
                    if self._stop.is_set():
                        break
                    try:
                        self._fk(ticks_for(cal, self._joint_ids, JOINT_ORDER, q))
                    except Exception:
                        pass
                    time.sleep(period)
            if not self.loop:
                break
        self._say("playback finished" if not self._stop.is_set() else "playback stopped")

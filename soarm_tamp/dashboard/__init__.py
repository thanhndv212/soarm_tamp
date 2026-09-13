"""Viser dashboard for soarm_tamp: plan in the container, run on the arm.

Built on :class:`soarm_sdk.dashboard.DashboardApp` rather than forking it —
a new tab is a ``Panel``, and connection handling, the live mirror and bus
access come for free. It lives here rather than in soarm_sdk because it
depends on soarm_tamp; the dependency only points this way.

The awkward part this package exists to hide is that the two halves of
soarm_tamp never share a process. Planning needs ``pyhpp``, which only
exists inside the HPP container; the servos hang off the host's USB. So:

* **planning** is a ``docker exec`` through ``scripts/hpp_container.sh``,
  with its output streamed back into the panel;
* **execution** runs in this process — but ``execute.run()`` opens the
  serial port itself, and the dashboard's live mirror is already holding
  it. The mirror therefore *releases* the port for the duration and
  follows ``<run>/live.jsonl`` instead, which is the trace execute already
  writes for exactly this purpose. One owner of the bus at any moment.
"""

from __future__ import annotations

__all__ = ["build_app"]


def build_app(*args, **kwargs):
    """Lazy re-export so importing this package needs no viser."""
    from .app import build_app as _build

    return _build(*args, **kwargs)

"""Dashboard tabs specific to soarm_tamp."""

from __future__ import annotations

__all__ = ["build_tcp_panel", "build_pickplace_panel"]


def build_tcp_panel(*args, **kwargs):
    from .tcp import build_tcp_panel as _b

    return _b(*args, **kwargs)


def build_pickplace_panel(*args, **kwargs):
    from .pickplace import build_pickplace_panel as _b

    return _b(*args, **kwargs)
from .watchdog import build_watchdog_panel

__all__ = ["build_watchdog_panel"]

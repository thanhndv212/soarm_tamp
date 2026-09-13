"""``python -m soarm_tamp.dashboard`` — the plan-and-run dashboard."""

from __future__ import annotations

import argparse
from pathlib import Path

from .app import DEFAULT_URDF, build_app


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", default="", help="servo bus, e.g. /dev/cu.usbmodem...")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--port", type=int, default=8080, help="Viser HTTP port")
    ap.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    ap.add_argument(
        "--no-stream",
        action="store_true",
        help="use the legacy per-iteration poll instead of holding the port",
    )
    args = ap.parse_args()

    device = args.device
    if not device:
        from soarm_sdk import get_available_ports

        ports = get_available_ports()
        if ports:
            device = ports[0][0]
            print(f"[soarm_tamp.dashboard] auto-selected {device}")

    build_app(
        device=device,
        baud=args.baud,
        port=args.port,
        urdf_path=args.urdf,
        use_stream=not args.no_stream,
    ).run()


if __name__ == "__main__":
    main()

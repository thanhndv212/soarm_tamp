"""soarm_tamp.dashboard.__main__ is now a thin wrapper around the shared
soarm_sdk.cli.dashboard.launch() — the actual argument parsing and
device/stream resolution are tested there (soarm_sdk's test suite), since
this module no longer has any of its own. What's worth covering here is
only that this module forwards the right profile, label, and defaults.

No viser, no hardware: launch() itself is faked.
"""

from __future__ import annotations

from pathlib import Path

from soarm_tamp.dashboard.__main__ import main


def test_main_forwards_the_plan_and_run_profile_and_soarm_tamp_defaults(monkeypatch):
    calls = []

    def _fake_launch(profile, argv, log_label, *, default_urdf, default_use_stream):
        calls.append(
            dict(
                profile=profile,
                argv=argv,
                log_label=log_label,
                default_urdf=default_urdf,
                default_use_stream=default_use_stream,
            )
        )

    monkeypatch.setattr("soarm_sdk.cli.dashboard.launch", _fake_launch)

    main(["--device", "/dev/ttyX"])

    assert len(calls) == 1
    call = calls[0]
    assert call["profile"].name == "plan_and_run"
    assert call["argv"] == ["--device", "/dev/ttyX"]
    assert call["log_label"] == "soarm_tamp.dashboard"
    assert isinstance(call["default_urdf"], Path)
    # Streaming defaults on: the plan-and-run panels need the persistent
    # interface for TCP goals and pick-and-place execution regardless.
    assert call["default_use_stream"] is True


def test_main_passes_argv_none_through_unchanged(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "soarm_sdk.cli.dashboard.launch",
        lambda profile, argv, log_label, **kwargs: calls.append(argv),
    )

    main(None)

    assert calls == [None]

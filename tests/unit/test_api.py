from __future__ import annotations

import subprocess
from pathlib import Path

from ros2docker import api


def _write_api_config(path: Path) -> Path:
    path.write_text(
        '{"container_name": "api_container", "image_name": "api_image", "run_type": "command", "command": "true"}',
        encoding="utf-8",
    )
    return path


def test_run_uses_config_dir_as_subprocess_cwd(tmp_path: Path, monkeypatch) -> None:
    config_dir = tmp_path / "configs"
    config_dir.mkdir()
    config_path = _write_api_config(config_dir / "ros2docker.json")
    calls: list[dict[str, object]] = []

    def fake_subprocess_run(command, *, check, cwd=None):
        calls.append({"command": command, "check": check, "cwd": cwd})
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(api.subprocess, "run", fake_subprocess_run)

    api.run(config_path)

    assert calls
    assert Path(str(calls[0]["cwd"])) == config_dir


def test_stop_runs_subprocess_with_check_false(tmp_path: Path, monkeypatch) -> None:
    config_path = _write_api_config(tmp_path / "ros2docker.json")
    calls: list[dict[str, object]] = []

    def fake_subprocess_run(command, *, check, cwd=None):
        calls.append({"command": command, "check": check, "cwd": cwd})
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(api.subprocess, "run", fake_subprocess_run)

    api.stop(config_path)

    assert calls[0]["command"] == ["docker", "stop", "api_container"]
    assert calls[0]["check"] is False


def test_build_run_calls_build_before_run(monkeypatch) -> None:
    calls: list[tuple[object, ...]] = []

    def fake_build(config_file, override, *, dry_run):
        calls.append(("build", config_file, override, dry_run))
        return ["build-result"]

    def fake_run(config_file, override, *, mount, extra_run_args, dry_run):
        calls.append(("run", config_file, override, mount, extra_run_args, dry_run))
        return ["run-result"]

    monkeypatch.setattr(api, "build", fake_build)
    monkeypatch.setattr(api, "run", fake_run)

    result = api.build_run(
        "ros2docker.json",
        {"image_name": "demo"},
        mount=".",
        extra_run_args=["--network", "host"],
        dry_run=True,
    )

    assert result == (["build-result"], ["run-result"])
    assert calls == [
        ("build", "ros2docker.json", {"image_name": "demo"}, True),
        ("run", "ros2docker.json", {"image_name": "demo"}, ".", ["--network", "host"], True),
    ]


def test_run_dry_run_returns_command_without_executing_subprocess(tmp_path: Path, monkeypatch) -> None:
    config_path = _write_api_config(tmp_path / "ros2docker.json")

    def fail_subprocess_run(*args, **kwargs):
        raise AssertionError("dry-run should not execute subprocess.run")

    monkeypatch.setattr(api.subprocess, "run", fail_subprocess_run)

    command = api.run(config_path, dry_run=True)

    assert command[:2] == ["docker", "run"]
    assert "api_image" in command


def test_build_temporary_context_is_removed_after_build(tmp_path: Path, monkeypatch) -> None:
    config_path = _write_api_config(tmp_path / "ros2docker.json")
    context_paths: list[Path] = []

    def fake_run(command, *, dry_run, cwd=None, check=True):
        context_path = Path(command[-1])
        assert context_path.is_dir()
        assert (context_path / "Dockerfile").is_file()
        assert (context_path / "entrypoint.sh").is_file()
        assert (context_path / "bake_packages").is_dir()
        context_paths.append(context_path)
        return list(command)

    monkeypatch.setattr(api, "_run", fake_run)

    api.build(config_path)

    assert context_paths
    assert not context_paths[0].exists()

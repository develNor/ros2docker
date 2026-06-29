from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

import ros2docker.cli as cli
from ros2docker import __version__
from ros2docker.cli import main

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _write_cli_config(tmp_path: Path) -> Path:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        json.dumps(
            {
                "container_name": "cli_container",
                "image_name": "cli_image",
                "run_type": "command",
                "command": ["bash", "-lc", "echo cli-ok"],
            }
        ),
        encoding="utf-8",
    )
    return config_path


def test_run_cli_uses_default_config_without_config_file(tmp_path: Path, capsys) -> None:
    mounted_dir = tmp_path / "mounted"
    mounted_dir.mkdir()

    result = main(["run", "--no-build", "--dry-run", "-m", str(mounted_dir)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert f"{mounted_dir.resolve()}:/ws" in captured.out
    assert captured.out.rstrip().endswith("ros2docker bash")


def test_run_cli_dry_run_with_config_prints_build_and_run_commands(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["run", "--dry-run", "-f", str(config_path)])

    captured = capsys.readouterr()
    lines = captured.out.splitlines()
    assert result == 0
    assert captured.err == ""
    assert len(lines) == 2
    assert lines[0].startswith("docker build ")
    assert "-t cli_image" in lines[0]
    assert lines[1].startswith("docker run ")
    assert "--name cli_container" in lines[1]
    assert lines[1].endswith("cli_image bash -lc 'echo cli-ok'")


def test_build_cli_dry_run_prints_docker_build_command(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["build", "--dry-run", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out.startswith("docker build ")
    assert "-t cli_image" in captured.out


def test_run_cli_dry_run_forwards_extra_docker_args_after_separator(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)
    data_dir = tmp_path / "data"
    data_dir.mkdir()

    result = main(
        [
            "run",
            "--no-build",
            "--dry-run",
            "-f",
            str(config_path),
            "--",
            "-v",
            f"{data_dir}:/data:ro",
        ]
    )

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert f"{data_dir.resolve()}:/data:ro" in captured.out
    assert captured.out.rstrip().endswith("cli_image bash -lc 'echo cli-ok'")


def test_run_no_build_dispatches_to_run(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[object, ...]] = []

    def fake_run(config, override, *, mount, extra_run_args, dry_run):
        calls.append((config, override, mount, extra_run_args, dry_run))

    def fake_build_run(*args, **kwargs):
        pytest.fail("run --no-build should not call build_run")

    monkeypatch.setattr(cli, "run", fake_run)
    monkeypatch.setattr(cli, "build_run", fake_build_run)

    result = main(["run", "--no-build", "--dry-run", "-m", ".", "--", "--network", "host"])

    assert result == 0
    assert calls == [(None, None, ".", ["--network", "host"], True)]


def test_run_without_no_build_dispatches_to_build_run(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[object, ...]] = []

    def fake_run(*args, **kwargs):
        pytest.fail("run without --no-build should not call run")

    def fake_build_run(config, override, *, mount, extra_run_args, dry_run):
        calls.append((config, override, mount, extra_run_args, dry_run))

    monkeypatch.setattr(cli, "run", fake_run)
    monkeypatch.setattr(cli, "build_run", fake_build_run)

    result = main(["run", "--dry-run", "-m", ".", "--", "--network", "host"])

    assert result == 0
    assert calls == [(None, None, ".", ["--network", "host"], True)]


def test_invalid_config_returns_exit_code_1_and_formats_error(tmp_path: Path, capsys) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"run_type": "compose"}', encoding="utf-8")

    result = main(["run", "--no-build", "--dry-run", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 1
    assert captured.out == ""
    assert captured.err.startswith("ros2docker: error: Invalid ros2docker config")


def test_validate_cli_reports_valid_config(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["validate", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out == "Config OK\n"


def test_validate_cli_prints_resolved_config(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["validate", "-f", str(config_path), "--print-resolved"])

    captured = capsys.readouterr()
    resolved = json.loads(captured.out)
    assert result == 0
    assert captured.err == ""
    assert resolved["container_name"] == "cli_container"
    assert resolved["command"] == ["bash", "-lc", "echo cli-ok"]


def test_doctor_cli_reports_readiness_without_docker_side_effects(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys,
) -> None:
    config_path = _write_cli_config(tmp_path)
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)

    result = main(["doctor", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert "OK: config:" in captured.out
    assert "WARN: docker:" in captured.out
    assert "INFO: gpu/device:" in captured.out


def test_doctor_cli_returns_nonzero_for_requested_missing_workspace(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys,
) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"mount_ws": true}', encoding="utf-8")
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)

    result = main(["doctor", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 1
    assert captured.err == ""
    assert "ERROR: workspace:" in captured.out


def test_stop_cli_dry_run_prints_docker_stop_command(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["stop", "--dry-run", "-f", str(config_path)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out == "docker stop cli_container\n"


def test_exec_cli_dry_run_prints_docker_exec_command(tmp_path: Path, capsys) -> None:
    config_path = _write_cli_config(tmp_path)

    result = main(["exec", "--dry-run", "-f", str(config_path), "--", "bash", "-lc", "ros2 --help"])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out == "docker exec cli_container bash -lc 'ros2 --help'\n"


def test_exec_without_command_dispatches_interactive_default_shell(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[object, ...]] = []

    def fake_exec_shell(config, override, *, command, interactive, dry_run):
        calls.append((config, override, command, interactive, dry_run))

    monkeypatch.setattr(cli, "exec_shell", fake_exec_shell)

    result = main(["exec", "--dry-run"])

    assert result == 0
    assert calls == [(None, None, None, True, True)]


def test_exec_with_command_dispatches_without_interactive_shell(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[object, ...]] = []

    def fake_exec_shell(config, override, *, command, interactive, dry_run):
        calls.append((config, override, command, interactive, dry_run))

    monkeypatch.setattr(cli, "exec_shell", fake_exec_shell)

    result = main(["exec", "--dry-run", "--", "bash", "-lc", "true"])

    assert result == 0
    assert calls == [(None, None, ["bash", "-lc", "true"], False, True)]


def test_dry_run_does_not_execute_subprocess(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config_path = _write_cli_config(tmp_path)

    def fail_subprocess_run(*args, **kwargs):
        pytest.fail("dry-run should not execute subprocess.run")

    monkeypatch.setattr("ros2docker.api.subprocess.run", fail_subprocess_run)

    result = main(["run", "--no-build", "--dry-run", "-f", str(config_path)])

    assert result == 0


def test_completion_command_prints_bash_script(capsys) -> None:
    result = main(["completion", "bash"])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert "complete -F _ros2docker_completion ros2docker" in captured.out


def test_completion_command_prints_zsh_script(capsys) -> None:
    result = main(["completion", "zsh"])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert "bashcompinit" in captured.out
    assert "complete -F _ros2docker_completion ros2docker" in captured.out


def test_completion_command_rejects_unknown_shell(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["completion", "fish"])

    assert exc_info.value.code == 2
    captured = capsys.readouterr()
    assert "invalid choice" in captured.err


def test_hidden_complete_command_lists_subcommands(capsys) -> None:
    result = main(["__complete", "--", ""])

    captured = capsys.readouterr()
    lines = captured.out.splitlines()
    assert result == 0
    assert "init" in lines
    assert lines[-1] == ":list"


def test_hidden_complete_command_lists_profiles_for_init(capsys) -> None:
    result = main(["__complete", "--", "init", "--profile", ""])

    captured = capsys.readouterr()
    lines = captured.out.splitlines()
    assert result == 0
    assert "minimal" in lines
    assert lines[-1] == ":list"


def test_hidden_complete_command_requests_file_completion_for_config(capsys) -> None:
    result = main(["__complete", "--", "run", "--config", ""])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.out.strip() == ":files"


def test_cli_version_reports_package_metadata(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["--version"])

    captured = capsys.readouterr()
    assert exc_info.value.code == 0
    assert captured.err == ""
    assert captured.out.strip() == f"ros2docker {__version__}"


def test_python_module_entrypoint_reports_version() -> None:
    env = os.environ.copy()
    pythonpath = str(PACKAGE_ROOT / "src")
    if env.get("PYTHONPATH"):
        pythonpath = f"{pythonpath}{os.pathsep}{env['PYTHONPATH']}"
    env["PYTHONPATH"] = pythonpath

    result = subprocess.run(
        [sys.executable, "-m", "ros2docker", "--version"],
        cwd=PACKAGE_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert result.stderr == ""
    assert result.stdout.strip() == f"ros2docker {__version__}"

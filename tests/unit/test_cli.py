from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

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

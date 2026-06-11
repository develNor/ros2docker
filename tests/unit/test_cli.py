from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from ros2docker import __version__
from ros2docker.cli import main

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def test_run_cli_uses_default_config_without_config_file(tmp_path: Path, capsys) -> None:
    mounted_dir = tmp_path / "mounted"
    mounted_dir.mkdir()

    result = main(["run", "--no-build", "--dry-run", "-m", str(mounted_dir)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert f"{mounted_dir.resolve()}:/ws" in captured.out
    assert captured.out.rstrip().endswith("ros2docker bash")


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

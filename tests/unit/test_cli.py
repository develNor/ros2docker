from __future__ import annotations

from pathlib import Path

from ros2docker.cli import main


def test_run_cli_uses_default_config_without_config_file(tmp_path: Path, capsys) -> None:
    mounted_dir = tmp_path / "mounted"
    mounted_dir.mkdir()

    result = main(["run", "--no-build", "--dry-run", "-m", str(mounted_dir)])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert f"{mounted_dir.resolve()}:/ws" in captured.out
    assert captured.out.rstrip().endswith("ros2docker bash")

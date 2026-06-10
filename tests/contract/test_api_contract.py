from __future__ import annotations

import json
from importlib import resources
from pathlib import Path

import pytest

from ros2docker.api import build_context
from ros2docker.cli import main


def test_run_dry_run_mounts_dot_as_ws(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    monkeypatch.chdir(tmp_path)

    result = main(["run", "--no-build", "--dry-run", "-m", "."])

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert f"{tmp_path.resolve()}:/ws" in captured.out
    assert captured.out.rstrip().endswith("ros2docker bash")


def test_build_context_stages_bake_packages_without_mutating_package_resources(tmp_path: Path) -> None:
    package_dir = tmp_path / "example_msgs"
    package_dir.mkdir()
    (package_dir / "package.xml").write_text("<package/>", encoding="utf-8")
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(json.dumps({"bake_ros_packages": ["./example_msgs"]}), encoding="utf-8")
    build_resources = resources.files("ros2docker").joinpath("resources").joinpath("build")
    before = sorted(item.name for item in build_resources.iterdir())

    with build_context(config_path) as context_dir:
        assert (context_dir / "bake_packages" / "example_msgs").is_dir()
        assert sorted(item.name for item in build_resources.iterdir()) == before

    assert sorted(item.name for item in build_resources.iterdir()) == before

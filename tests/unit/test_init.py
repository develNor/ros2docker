from __future__ import annotations

import json
from pathlib import Path

import pytest

from ros2docker.api import init
from ros2docker.cli import main


def test_init_creates_default_starter_files(tmp_path: Path) -> None:
    init(cwd=tmp_path)

    assert (tmp_path / "ros2docker.json").is_file()
    assert (tmp_path / "ws" / "src").is_dir()
    assert (tmp_path / "ws" / ".gitignore").is_file()
    assert (tmp_path / "catmux.yaml").is_file()
    assert not (tmp_path / ".devcontainer" / "devcontainer.json").exists()

    config = json.loads((tmp_path / "ros2docker.json").read_text(encoding="utf-8"))
    assert config["profile"] == "minimal"
    assert config["image_name"] == "ros2docker-minimal"
    assert config["container_name"] == "ros2container_lyrical_minimal"
    assert config["mount_ws"] is True
    # Default lyrical minimal profile has no custom build_args (since lyrical is default)
    assert "build_args" not in config

    gitignore = (tmp_path / "ws" / ".gitignore").read_text(encoding="utf-8")
    assert "build/" in gitignore
    assert "install/" in gitignore
    assert "log/" in gitignore

    catmux = (tmp_path / "catmux.yaml").read_text(encoding="utf-8")
    assert "ros2 topic list" in catmux


def test_init_with_desktop_profile(tmp_path: Path) -> None:
    init(profile="desktop", cwd=tmp_path)

    config = json.loads((tmp_path / "ros2docker.json").read_text(encoding="utf-8"))
    assert config["profile"] == "desktop"
    assert config["image_name"] == "ros2docker-desktop"
    assert config["container_name"] == "ros2container_lyrical_desktop"


def test_init_with_custom_distro(tmp_path: Path) -> None:
    init(profile="minimal", ros_distro="jazzy", cwd=tmp_path)

    config = json.loads((tmp_path / "ros2docker.json").read_text(encoding="utf-8"))
    assert config["profile"] == "minimal"
    assert config["image_name"] == "ros2docker-minimal"
    assert config["container_name"] == "ros2container_jazzy_minimal"
    assert config["build_args"]["BASE_IMAGE"] == "ros:jazzy-ros-base"
    assert config["build_args"]["DIGEST"] == ""


def test_init_with_devcontainer(tmp_path: Path) -> None:
    init(devcontainer=True, cwd=tmp_path)

    assert (tmp_path / ".devcontainer" / "devcontainer.json").is_file()
    devcontainer = json.loads((tmp_path / ".devcontainer" / "devcontainer.json").read_text(encoding="utf-8"))
    assert devcontainer["name"] == "ros2docker workspace"
    assert devcontainer["image"] == "ros2docker-minimal"
    assert devcontainer["remoteUser"] == "containeruser"


def test_init_overwrite_guards(tmp_path: Path) -> None:
    # First init works
    init(cwd=tmp_path)

    # Second init without overwrite fails
    with pytest.raises(FileExistsError, match="Cannot initialize workspace: the following files already exist"):
        init(cwd=tmp_path)

    # Second init with overwrite succeeds
    init(overwrite=True, cwd=tmp_path)


def test_init_invalid_profile(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="Profile 'invalid-profile' not found"):
        init(profile="invalid-profile", cwd=tmp_path)


def test_cli_init_subcommand_dispatch(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    calls = []

    def fake_init(profile, ros_distro, devcontainer, overwrite, cwd=None):
        calls.append((profile, ros_distro, devcontainer, overwrite))

    import ros2docker.cli as cli

    monkeypatch.setattr(cli, "init", fake_init)

    # Test basic CLI call
    result = main(["init"])
    assert result == 0
    assert calls == [("minimal", "lyrical", False, False)]

    # Test CLI call with options
    calls.clear()
    result = main(["init", "--profile", "desktop", "--ros-distro", "jazzy", "--devcontainer", "--overwrite"])
    assert result == 0
    assert calls == [("desktop", "jazzy", True, True)]

from __future__ import annotations

from pathlib import Path

import pytest

from ros2docker.config import ConfigError, load_config


def write_config(path: Path, data: str) -> Path:
    path.write_text(data, encoding="utf-8")
    return path


def test_json_comments_and_comment_markers_inside_strings(tmp_path: Path) -> None:
    config_path = write_config(
        tmp_path / "ros2docker.json",
        """
        {
          // line comment
          "container_name": "demo//not-a-comment",
          "run_type": "bash",
          "run_args": [
            "-e", "URL=https://example.test/path/*not-comment*/"
          ]
        }
        """,
    )

    config = load_config(config_path)

    assert config["container_name"] == "demo//not-a-comment"
    assert config["run_args"] == ["-e", "URL=https://example.test/path/*not-comment*/"]


def test_override_replaces_top_level_config_values(tmp_path: Path) -> None:
    config_path = write_config(tmp_path / "ros2docker.json", '{"container_name": "old"}')

    config = load_config(config_path, '{"container_name": "new", "image_name": "image"}')

    assert config["container_name"] == "new"
    assert config["image_name"] == "image"


def test_volume_and_bake_paths_resolve_from_config_dir(tmp_path: Path) -> None:
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    package_dir = tmp_path / "packages" / "msgs"
    package_dir.mkdir(parents=True)
    config_path = tmp_path / "configs" / "ros2docker.json"
    config_path.parent.mkdir(exist_ok=True)
    config_path.write_text(
        """
        {
          "run_args": ["-v", "../data:/data:ro"],
          "bake_ros_packages": ["../packages/msgs"]
        }
        """,
        encoding="utf-8",
    )

    config = load_config(config_path)

    assert config["run_args"] == ["-v", f"{data_dir.resolve()}:/data:ro"]
    assert config["bake_ros_packages"] == [str(package_dir.resolve())]


def test_volume_paths_expand_home_and_env_vars(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    mounted = tmp_path / "mounted"
    mounted.mkdir()
    monkeypatch.setenv("ROS2DOCKER_TEST_MOUNT", str(mounted))
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"run_args": ["--volume=${ROS2DOCKER_TEST_MOUNT}:/data"]}',
    )

    config = load_config(config_path)

    assert config["run_args"] == [f"--volume={mounted.resolve()}:/data"]


def test_bare_dot_volume_path_resolves_from_config_dir(tmp_path: Path) -> None:
    config_path = write_config(tmp_path / "ros2docker.json", '{"run_args": ["-v", ".:/ws"]}')

    config = load_config(config_path)

    assert config["run_args"] == ["-v", f"{tmp_path.resolve()}:/ws"]


def test_missing_host_volume_path_fails_clearly(tmp_path: Path) -> None:
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"run_args": ["-v", "./missing:/missing"]}',
    )

    with pytest.raises(FileNotFoundError, match="Host path does not exist"):
        load_config(config_path)


def test_run_type_validation(tmp_path: Path) -> None:
    config_path = write_config(tmp_path / "ros2docker.json", '{"run_type": "compose"}')

    with pytest.raises(ConfigError, match="Unsupported run_type"):
        load_config(config_path)

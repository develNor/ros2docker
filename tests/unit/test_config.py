from __future__ import annotations

import json
from pathlib import Path

import pytest

from ros2docker.config import ConfigError, load_config, strip_json_comments


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


def test_bind_mount_src_path_resolves_from_config_dir(tmp_path: Path) -> None:
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    config_dir = tmp_path / "configs"
    config_dir.mkdir()
    config_path = write_config(
        config_dir / "ros2docker.json",
        '{"run_args": ["--mount", "type=bind,src=../data,dst=/data"]}',
    )

    config = load_config(config_path)

    assert config["run_args"] == ["--mount", f"type=bind,src={data_dir.resolve()},dst=/data"]


def test_bind_mount_source_path_resolves_from_config_dir_for_equals_form(tmp_path: Path) -> None:
    data_dir = tmp_path / "data"
    data_dir.mkdir()
    config_dir = tmp_path / "configs"
    config_dir.mkdir()
    config_path = write_config(
        config_dir / "ros2docker.json",
        '{"run_args": ["--mount=type=bind,source=../data,target=/data"]}',
    )

    config = load_config(config_path)

    assert config["run_args"] == [f"--mount=type=bind,source={data_dir.resolve()},target=/data"]


@pytest.mark.parametrize("arg", ["-v", "--volume", "--mount"])
def test_mount_or_volume_args_require_a_following_value(tmp_path: Path, arg: str) -> None:
    config_path = write_config(tmp_path / "ros2docker.json", json.dumps({"run_args": [arg]}))

    with pytest.raises(ConfigError, match="requires a following"):
        load_config(config_path)


def test_unresolved_environment_variable_in_mount_path_fails(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROS2DOCKER_MISSING_MOUNT", raising=False)
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"run_args": ["-v", "$ROS2DOCKER_MISSING_MOUNT:/data"]}',
    )

    with pytest.raises(ConfigError, match="Could not expand environment variables"):
        load_config(config_path)


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


def test_override_must_decode_to_object() -> None:
    with pytest.raises(ConfigError, match="Override must decode to a JSON object"):
        load_config(override='["container_name", "demo"]')


def test_command_list_allows_json_scalar_values(tmp_path: Path) -> None:
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"run_type": "command", "command": ["echo", 3, true]}',
    )

    config = load_config(config_path)

    assert config["command"] == ["echo", 3, True]


def test_command_list_rejects_non_scalar_values(tmp_path: Path) -> None:
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"run_type": "command", "command": ["echo", {"bad": "value"}]}',
    )

    with pytest.raises(ConfigError, match="'command' list items"):
        load_config(config_path)


def test_unterminated_jsonc_block_comment_fails(tmp_path: Path) -> None:
    config_path = write_config(
        tmp_path / "ros2docker.json",
        '{"container_name": "demo"} /* missing terminator',
    )

    with pytest.raises(ConfigError, match="Unterminated block comment"):
        load_config(config_path)


def test_json_comment_stripper_preserves_escaped_quotes_and_backslashes() -> None:
    parsed = json.loads(
        strip_json_comments(
            r"""
            {
              "container_name": "demo \" // still a string",
              "run_args": ["-e", "WIN_PATH=C:\\tmp\\/*not-comment*/"]
            }
            """
        )
    )

    assert parsed["container_name"] == 'demo " // still a string'
    assert parsed["run_args"] == ["-e", r"WIN_PATH=C:\tmp\/*not-comment*/"]

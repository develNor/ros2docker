from __future__ import annotations

from pathlib import Path

from ros2docker.commands import make_build_command, make_exec_shell_command, make_run_command, make_stop_command


def test_command_rendering_for_command_run_type(tmp_path: Path) -> None:
    ws = tmp_path / "ws"
    ws.mkdir()
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        """
        {
          "container_name": "demo_container",
          "image_name": "demo_image",
          "mount_ws": true,
          "run_type": "command",
          "command": ["bash", "-lc", "echo ok"],
          "run_args": ["--network", "host", "-e", "ROS_DOMAIN_ID=47"]
        }
        """,
        encoding="utf-8",
    )

    command = make_run_command(config_path)

    assert command[:3] == ["docker", "run", "--name"]
    assert "demo_container" in command
    assert "demo_image" in command
    assert ["bash", "-lc", "echo ok"] == command[-3:]
    assert f"{ws.resolve()}:/ws" in command
    assert "ROS_DOMAIN_ID=47" in command


def test_cli_mount_and_extra_args_are_rendered(tmp_path: Path) -> None:
    bag_dir = tmp_path / "bags"
    bag_dir.mkdir()
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"container_name": "player", "run_type": "bash"}', encoding="utf-8")

    command = make_run_command(config_path, mount=bag_dir, extra_run_args=["-e", "A=B"])

    assert f"{bag_dir.resolve()}:/ws" in command
    assert "-e" in command
    assert "A=B" in command


def test_build_stop_and_exec_commands(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        '{"container_name": "demo", "image_name": "image", "build_args": {"BASE_IMAGE": "base"}}',
        encoding="utf-8",
    )

    build = make_build_command(config_path, context_dir=tmp_path / "context")
    stop = make_stop_command(config_path)
    exec_command = make_exec_shell_command(config_path, command=["bash", "-lc", "true"], interactive=False)

    assert build[0:3] == ["docker", "build", "-t"]
    assert "image" in build
    assert "--build-arg" in build
    assert "BASE_IMAGE=base" in build
    assert build[-1] == str(tmp_path / "context")
    assert stop == ["docker", "stop", "demo"]
    assert exec_command == ["docker", "exec", "demo", "bash", "-lc", "true"]


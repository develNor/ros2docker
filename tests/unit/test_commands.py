from __future__ import annotations

import os
from pathlib import Path

import pytest

from ros2docker.commands import make_build_command, make_exec_shell_command, make_run_command, make_stop_command
from ros2docker.config import ConfigError


def _patch_x11_socket(monkeypatch: pytest.MonkeyPatch, *, present: bool) -> None:
    """Force the /tmp/.X11-unix existence check without touching the real host."""
    real_exists = Path.exists

    def fake_exists(self: Path) -> bool:
        if str(self) == "/tmp/.X11-unix":
            return present
        return real_exists(self)

    monkeypatch.setattr(Path, "exists", fake_exists)


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
    assert "-i" not in command
    assert "-t" not in command


def test_cli_mount_and_extra_args_are_rendered(tmp_path: Path) -> None:
    bag_dir = tmp_path / "bags"
    bag_dir.mkdir()
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"container_name": "player", "run_type": "bash"}', encoding="utf-8")

    command = make_run_command(config_path, mount=bag_dir, extra_run_args=["-e", "A=B"])

    assert f"{bag_dir.resolve()}:/ws" in command
    assert "-e" in command
    assert "A=B" in command


def test_default_config_can_open_mounted_bash_shell(tmp_path: Path) -> None:
    mounted_dir = tmp_path / "mounted"
    mounted_dir.mkdir()

    command = make_run_command(mount=mounted_dir)

    assert "--name" in command
    assert "ros2docker" in command
    assert f"{mounted_dir.resolve()}:/ws" in command
    assert "-i" in command
    assert "-t" in command
    assert command[-2:] == ["ros2docker", "bash"]


def test_catmux_run_type_defaults_to_interactive_docker_run(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"run_type": "catmux", "catmux_file": "/ws/catmux.yaml"}', encoding="utf-8")

    command = make_run_command(config_path)

    assert "-i" in command
    assert "-t" in command
    assert command[-4:] == ["catmux_create_session", "/ws/catmux.yaml", "--session_name", "ros2docker"]


def test_up_run_type_defaults_to_detached_without_interactive_flags(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"run_type": "up"}', encoding="utf-8")

    command = make_run_command(config_path)

    assert "-d" in command
    assert "-i" not in command
    assert "-t" not in command
    assert command[-4:] == ["ros2docker", "tail", "-f", "/dev/null"]


def test_command_run_type_can_be_made_explicitly_interactive(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        """
        {
          "run_type": "command",
          "command": ["bash", "-lc", "read -r line && echo $line"],
          "tty": true,
          "stdin_open": true
        }
        """,
        encoding="utf-8",
    )

    command = make_run_command(config_path)

    assert "-i" in command
    assert "-t" in command
    assert command[-3:] == ["bash", "-lc", "read -r line && echo $line"]


def test_build_stop_and_exec_commands(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        """
        {
          "container_name": "demo",
          "image_name": "image",
          "build_args": {
            "BASE_IMAGE": "base",
            "DIGEST": ""
          }
        }
        """,
        encoding="utf-8",
    )

    build = make_build_command(config_path, context_dir=tmp_path / "context")
    stop = make_stop_command(config_path)
    exec_command = make_exec_shell_command(config_path, command=["bash", "-lc", "true"], interactive=False)

    assert build[0:3] == ["docker", "build", "-t"]
    assert "image" in build
    assert "--build-arg" in build
    assert f"USER_UID={os.getuid()}" in build
    assert f"USER_GID={os.getgid()}" in build
    assert "BASE_IMAGE=base" in build
    assert "DIGEST=" in build
    assert build[-1] == str(tmp_path / "context")
    assert stop == ["docker", "stop", "demo"]
    assert exec_command == ["docker", "exec", "demo", "bash", "-lc", "true"]


def test_exec_shell_command_is_interactive_by_default() -> None:
    exec_command = make_exec_shell_command(override={"container_name": "demo"})

    assert exec_command == ["docker", "exec", "-it", "demo", "bash"]


def test_gui_forwarding_renders_x11_mount_when_socket_present(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _patch_x11_socket(monkeypatch, present=True)

    command = make_run_command(override={"enable_gui_forwarding": True}, mount=tmp_path)

    assert "/tmp/.X11-unix:/tmp/.X11-unix" in command
    assert "DISPLAY" in command


def test_gui_forwarding_without_x11_socket_raises(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _patch_x11_socket(monkeypatch, present=False)

    with pytest.raises(FileNotFoundError, match="/tmp/.X11-unix"):
        make_run_command(override={"enable_gui_forwarding": True}, mount=tmp_path)


def test_ssh_agent_forwarding_renders_socket_mount(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    sock = tmp_path / "agent.sock"
    sock.write_text("", encoding="utf-8")
    monkeypatch.setenv("SSH_AUTH_SOCK", str(sock))

    command = make_run_command(override={"forward_ssh_agent": True}, mount=tmp_path)

    assert "SSH_AUTH_SOCK" in command
    assert f"{sock}:{sock}" in command


def test_ssh_agent_forwarding_without_sock_raises(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv("SSH_AUTH_SOCK", raising=False)

    with pytest.raises(ConfigError, match="SSH_AUTH_SOCK is not set"):
        make_run_command(override={"forward_ssh_agent": True}, mount=tmp_path)


def test_ssh_agent_forwarding_with_missing_sock_raises(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("SSH_AUTH_SOCK", str(tmp_path / "missing.sock"))

    with pytest.raises(FileNotFoundError, match="does not exist"):
        make_run_command(override={"forward_ssh_agent": True}, mount=tmp_path)


def test_mount_ws_without_workspace_directory_raises(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"mount_ws": true}', encoding="utf-8")

    with pytest.raises(FileNotFoundError, match="workspace directory does not exist"):
        make_run_command(config_path)


def test_catmux_params_render_overwrite_argument(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text(
        """
        {
          "run_type": "catmux",
          "catmux_file": "/ws/catmux.yaml",
          "catmux_params": {"robot": "r1", "use_sim": true}
        }
        """,
        encoding="utf-8",
    )

    command = make_run_command(config_path)

    assert "--overwrite" in command
    overwrite_value = command[command.index("--overwrite") + 1]
    assert "robot=r1" in overwrite_value
    assert "use_sim=True" in overwrite_value

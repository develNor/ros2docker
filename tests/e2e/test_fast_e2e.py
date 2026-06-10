from __future__ import annotations

import time
from pathlib import Path

import pytest
from conftest import copy_fixture_tree, wait_for_file, write_config

pytestmark = pytest.mark.e2e


def _base_config(image: str, container: str) -> dict[str, object]:
    return {
        "container_name": container,
        "image_name": image,
        "run_type": "command",
        "command": ["bash", "-lc", "echo E2E_BASE_OK"],
    }


def test_minimal_cli_command_list_runs_ros_tools(docker_harness, shared_image: str, tmp_path: Path) -> None:
    config = _base_config(shared_image, docker_harness.container_name("minimal_cli"))
    config["command"] = [
        "bash",
        "-lc",
        "ros2 --help >/tmp/ros2_help && jq --version >/tmp/jq_version && "
        "python -c 'import rich; print(\"E2E_MINIMAL_OK\")'",
    ]
    config_path = write_config(tmp_path / "ros2docker.json", config)

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=180)

    assert "E2E_MINIMAL_OK" in result.stdout


def test_bash_run_type_starts_interactive_shell_under_pty(
    docker_harness,
    shared_image: str,
    tmp_path: Path,
) -> None:
    container = docker_harness.container_name("bash_shell")
    config_path = write_config(
        tmp_path / "bash.ros2docker.json",
        {
            "container_name": container,
            "image_name": shared_image,
            "run_type": "bash",
        },
    )

    command = docker_harness.start_cli_pty("run", "--no-build", "-f", str(config_path))
    try:
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            command.read_available()
            inspect = docker_harness.docker(
                "inspect",
                "-f",
                "{{.State.Running}}",
                container,
                timeout=10,
                check=False,
            )
            if inspect.returncode == 0 and inspect.stdout.strip() == "true":
                break
            if command.process.poll() is not None:
                raise AssertionError(f"bash container exited early:\n{command.output}")
            time.sleep(0.5)
        else:
            raise AssertionError(f"Timed out waiting for bash container:\n{command.output}")

        exec_result = docker_harness.cli(
            "exec",
            "-f",
            str(config_path),
            "--",
            "bash",
            "-lc",
            "echo E2E_BASH_OK",
            timeout=120,
        )
        assert "E2E_BASH_OK" in exec_result.stdout
    finally:
        docker_harness.docker("stop", container, timeout=30, check=False)
        command.wait(timeout=60, check=False)


def test_command_string_mount_and_extra_args(docker_harness, shared_image: str, tmp_path: Path) -> None:
    project = copy_fixture_tree("projects/command_mount", tmp_path)
    config = _base_config(shared_image, docker_harness.container_name("command_mount"))
    config.update(
        {
            "mount_ws": True,
            "command": (
                "bash -lc 'test -f /ws/probe.txt && test -f /mounted/in.txt && "
                'test "$CONFIG_EXTRA" = config && test "$CLI_EXTRA" = cli && '
                "echo E2E_MOUNT_OK'"
            ),
            "run_args": ["-v", "./mounted:/mounted:ro"],
            "extra_run_args": ["-e", "CONFIG_EXTRA=config"],
        }
    )
    config_path = write_config(project / "ros2docker.json", config)

    result = docker_harness.cli(
        "run",
        "--no-build",
        "-f",
        str(config_path),
        "--",
        "-e",
        "CLI_EXTRA=cli",
        timeout=180,
    )

    assert "E2E_MOUNT_OK" in result.stdout


def test_workspace_std_msgs_builds_and_is_sourced(docker_harness, shared_image: str, tmp_path: Path) -> None:
    project = copy_fixture_tree("workspaces/std", tmp_path)
    config = _base_config(shared_image, docker_harness.container_name("workspace_std"))
    config.update(
        {
            "mount_ws": True,
            "command": ["bash", "-lc", "ros2 run e2e_std_pkg std_probe"],
            "run_args": ["-e", "BUILD_ROS2WS=1"],
        }
    )
    config_path = write_config(project / "ros2docker.json", config)

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=420)

    assert "E2E_STD:std-ok" in result.stdout


def test_workspace_custom_msgs_uses_baked_message_package(
    docker_harness,
    shared_image: str,
    tmp_path: Path,
) -> None:
    project = copy_fixture_tree("workspaces/custom", tmp_path)
    config = _base_config(shared_image, docker_harness.container_name("workspace_custom"))
    config.update(
        {
            "mount_ws": True,
            "command": [
                "bash",
                "-lc",
                "ros2 interface show e2e_msgs/msg/Ping >/tmp/ping.interface && ros2 run e2e_custom_pkg custom_probe",
            ],
            "run_args": ["-e", "BUILD_ROS2WS=1"],
        }
    )
    config_path = write_config(project / "ros2docker.json", config)

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=420)

    assert "E2E_CUSTOM:7:custom-ok" in result.stdout


def test_catmux_center_like_params_write_probe_file(docker_harness, shared_image: str, tmp_path: Path) -> None:
    project = copy_fixture_tree("projects/catmux_center_like", tmp_path)
    probe_file = project / "ws" / "out" / "catmux_probe.txt"
    container = docker_harness.container_name("catmux_center")
    config = {
        "container_name": container,
        "image_name": shared_image,
        "mount_ws": True,
        "run_type": "catmux",
        "catmux_file": "/ws/catmux_center_like.yaml",
        "catmux_params": {"role": "center", "mission": "fixture"},
    }
    config_path = write_config(project / "ros2docker.json", config)

    command = docker_harness.start_cli_pty("run", "--no-build", "-f", str(config_path))
    try:
        wait_for_file(probe_file, timeout=60, pty_command=command)
        contents = probe_file.read_text(encoding="utf-8")
        assert "role=center" in contents
        assert "mission=fixture" in contents
    finally:
        docker_harness.docker("stop", container, timeout=30, check=False)
        command.wait(timeout=60, check=False)


def test_dependency_check_passes_for_valid_workspace(docker_harness, shared_image: str, tmp_path: Path) -> None:
    project = copy_fixture_tree("workspaces/std", tmp_path)
    config = _base_config(shared_image, docker_harness.container_name("deps_pass"))
    config.update(
        {
            "mount_ws": True,
            "command": ["bash", "-lc", "echo E2E_DEP_PASS"],
            "run_args": ["-e", "CHECK_ROS2WS_DEPENDENCIES=1"],
        }
    )
    config_path = write_config(project / "ros2docker.json", config)

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=600)

    assert "All dependencies are satisfied." in result.stdout
    assert "E2E_DEP_PASS" in result.stdout


def test_dependency_check_fails_before_command_for_missing_rosdep(
    docker_harness,
    shared_image: str,
    tmp_path: Path,
) -> None:
    project = copy_fixture_tree("workspaces/missing_dep", tmp_path)
    probe_file = project / "ws" / "out" / "should_not_run"
    config = _base_config(shared_image, docker_harness.container_name("deps_fail"))
    config.update(
        {
            "mount_ws": True,
            "command": ["bash", "-lc", "mkdir -p /ws/out && touch /ws/out/should_not_run"],
            "run_args": ["-e", "CHECK_ROS2WS_DEPENDENCIES=1"],
        }
    )
    config_path = write_config(project / "ros2docker.json", config)

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=600, check=False)
    output = result.stdout + result.stderr

    assert result.returncode != 0
    assert not probe_file.exists()
    assert "Dependencies are missing." in output or "Cannot locate rosdep definition" in output


def test_up_lifecycle_exec_and_stop(docker_harness, shared_image: str, tmp_path: Path) -> None:
    container = docker_harness.container_name("up_lifecycle")
    config = {
        "container_name": container,
        "image_name": shared_image,
        "run_type": "up",
    }
    config_path = write_config(tmp_path / "ros2docker.json", config)

    docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=180)
    exec_result = docker_harness.cli(
        "exec",
        "-f",
        str(config_path),
        "--",
        "bash",
        "-lc",
        "echo E2E_UP_OK",
        timeout=120,
    )
    stop_result = docker_harness.cli("stop", "-f", str(config_path), timeout=120)

    assert "E2E_UP_OK" in exec_result.stdout
    assert container in stop_result.stdout


def test_gui_and_ssh_forwarding_contracts(docker_harness, shared_image: str, tmp_path: Path) -> None:
    no_gui_config = _base_config(shared_image, docker_harness.container_name("no_gui"))
    no_gui_config["command"] = ["bash", "-lc", "echo E2E_NO_GUI_OK"]
    no_gui_path = write_config(tmp_path / "no_gui.ros2docker.json", no_gui_config)
    no_gui_result = docker_harness.cli("run", "--no-build", "-f", str(no_gui_path), timeout=180)
    assert "E2E_NO_GUI_OK" in no_gui_result.stdout

    ssh_sock = Path("/tmp") / f"ros2docker_e2e_ssh_{docker_harness.run_id}.sock"
    ssh_sock.write_text("fake ssh agent socket for ros2docker e2e\n", encoding="utf-8")
    try:
        ssh_config = _base_config(shared_image, docker_harness.container_name("ssh_agent"))
        ssh_config.update(
            {
                "forward_ssh_agent": True,
                "command": ["bash", "-lc", 'test -e "$SSH_AUTH_SOCK" && echo E2E_SSH_OK'],
            }
        )
        ssh_path = write_config(tmp_path / "ssh.ros2docker.json", ssh_config)
        ssh_result = docker_harness.cli(
            "run",
            "--no-build",
            "-f",
            str(ssh_path),
            timeout=180,
            env={"SSH_AUTH_SOCK": str(ssh_sock)},
        )
        assert "E2E_SSH_OK" in ssh_result.stdout
    finally:
        ssh_sock.unlink(missing_ok=True)

    gui_config = _base_config(shared_image, docker_harness.container_name("gui"))
    gui_config.update(
        {
            "enable_gui_forwarding": True,
            "command": ["bash", "-lc", "test -d /tmp/.X11-unix && echo E2E_GUI_OK"],
        }
    )
    gui_path = write_config(tmp_path / "gui.ros2docker.json", gui_config)
    if Path("/tmp/.X11-unix").exists():
        gui_result = docker_harness.cli("run", "--no-build", "-f", str(gui_path), timeout=180)
        assert "E2E_GUI_OK" in gui_result.stdout
    else:
        gui_result = docker_harness.cli(
            "run",
            "--no-build",
            "--dry-run",
            "-f",
            str(gui_path),
            timeout=60,
            check=False,
        )
        assert gui_result.returncode == 1
        assert "GUI forwarding requested but /tmp/.X11-unix does not exist." in gui_result.stderr


def test_multi_container_native_chatter(docker_harness, shared_image: str, tmp_path: Path) -> None:
    domain_id = 120 + int(docker_harness.run_id[:2], 16) % 80
    topic = f"/ros2docker_e2e_chatter_{docker_harness.run_id}"
    publisher_container = docker_harness.container_name("chatter_pub")
    publisher_config = {
        "container_name": publisher_container,
        "image_name": shared_image,
        "run_type": "command",
        "command": [
            "bash",
            "-lc",
            f"ros2 topic pub -r 5 {topic} std_msgs/msg/String '{{data: hello_e2e}}' >/tmp/e2e_pub.log 2>&1",
        ],
        "run_args": ["--network", "host", "-e", f"ROS_DOMAIN_ID={domain_id}"],
    }
    publisher_path = write_config(tmp_path / "publisher.ros2docker.json", publisher_config)
    subscriber_config = {
        "container_name": docker_harness.container_name("chatter_sub"),
        "image_name": shared_image,
        "run_type": "command",
        "command": [
            "bash",
            "-lc",
            f"timeout 25 ros2 topic echo --once {topic} std_msgs/msg/String",
        ],
        "run_args": ["--network", "host", "-e", f"ROS_DOMAIN_ID={domain_id}"],
    }
    subscriber_path = write_config(tmp_path / "subscriber.ros2docker.json", subscriber_config)

    publisher = docker_harness.start_cli("run", "--no-build", "-f", str(publisher_path))
    try:
        time.sleep(6)
        result = docker_harness.cli("run", "--no-build", "-f", str(subscriber_path), timeout=60)
        assert "hello_e2e" in result.stdout
    finally:
        docker_harness.docker("stop", publisher_container, timeout=30, check=False)
        publisher.wait(timeout=60, check=False)

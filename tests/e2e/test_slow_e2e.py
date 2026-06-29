from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from conftest import FIXTURES_ROOT, wait_for_file, write_config

pytestmark = [pytest.mark.e2e, pytest.mark.slow]


def _parse_agent_var(agent_output: str, name: str) -> str:
    match = re.search(rf"{name}=([^;]+);", agent_output)
    assert match, f"{name} not found in ssh-agent output:\n{agent_output}"
    return match.group(1)


def test_rosbag_record_and_play_round_trip(docker_harness, shared_image: str, tmp_path: Path) -> None:
    container = docker_harness.container_name("rosbag_round_trip")
    command = r"""
set -euo pipefail
bag=/tmp/ros2docker_e2e_bag
topic=/ros2docker_e2e_bag_topic
rm -rf "$bag"
ros2 topic pub -r 10 "$topic" std_msgs/msg/String "{data: bag-ok}" >/tmp/e2e_pub.log 2>&1 &
pub_pid=$!
sleep 2
set +e
timeout -s INT 8 ros2 bag record -o "$bag" --topics "$topic" >/tmp/e2e_record.log 2>&1
record_status=$?
set -e
kill "$pub_pid" >/dev/null 2>&1 || true
wait "$pub_pid" >/dev/null 2>&1 || true
if [ "$record_status" != "0" ] && [ "$record_status" != "124" ]; then
  cat /tmp/e2e_record.log
  exit "$record_status"
fi
test -f "$bag/metadata.yaml"
ros2 bag info "$bag" >/tmp/e2e_bag_info.log
grep "/ros2docker_e2e_bag_topic" /tmp/e2e_bag_info.log >/dev/null
timeout 30 ros2 bag play "$bag" >/tmp/e2e_bag_play.log 2>&1
echo E2E_BAG_OK
"""
    config_path = write_config(
        tmp_path / "rosbag.ros2docker.json",
        {
            "container_name": container,
            "image_name": shared_image,
            "run_type": "command",
            "command": ["bash", "-lc", command],
        },
    )

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=180)

    assert "E2E_BAG_OK" in result.stdout


def test_foxglove_bridge_launch_smoke(docker_harness, shared_image: str, tmp_path: Path) -> None:
    container = docker_harness.container_name("foxglove_smoke")
    command = r"""
set -euo pipefail
status=0
timeout 12 ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 >/tmp/e2e_foxglove.log 2>&1 || status=$?
test "$status" = 124
grep -E "foxglove|Foxglove|bridge" /tmp/e2e_foxglove.log >/dev/null
echo E2E_FOXGLOVE_OK
"""
    config_path = write_config(
        tmp_path / "foxglove.ros2docker.json",
        {
            "container_name": container,
            "image_name": shared_image,
            "run_type": "command",
            "command": ["bash", "-lc", command],
        },
    )

    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=120)

    assert "E2E_FOXGLOVE_OK" in result.stdout


def test_minimal_profile_ros_base_builds_and_runs(docker_harness, tmp_path: Path) -> None:
    # The shared E2E image is desktop-full; the minimal profile's ros-base path
    # (ros:*-ros-base, no desktop) is otherwise never built or run in E2E, so a
    # regression that only affects ros-base would go undetected. Build it and run
    # a basic ROS tool command to confirm the path builds and runs.
    image = docker_harness.image_tag("minimal-profile")
    container = docker_harness.container_name("minimal_profile")
    config_path = write_config(
        tmp_path / "minimal.ros2docker.json",
        {
            "profile": "minimal",
            "container_name": container,
            "image_name": image,
            "run_type": "command",
            "command": [
                "bash",
                "-lc",
                "ros2 pkg list >/tmp/ros2_pkg_list && ros2 --help >/tmp/ros2_help && echo E2E_MINIMAL_OK",
            ],
        },
    )

    docker_harness.cli("build", "-f", str(config_path), timeout=2400)
    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=180)

    assert "E2E_MINIMAL_OK" in result.stdout


@pytest.mark.parametrize(
    ("ros_distro", "base_image"),
    [
        ("lyrical", "osrf/ros:lyrical-desktop-resolute"),
        ("kilted", "osrf/ros:kilted-desktop-noble"),
        ("rolling", "osrf/ros:rolling-desktop-noble"),
    ],
)
def test_latest_base_image_matrix_builds_and_runs(
    docker_harness,
    tmp_path: Path,
    ros_distro: str,
    base_image: str,
) -> None:
    image = docker_harness.image_tag(f"alt-base-{ros_distro}")
    container = docker_harness.container_name(f"alt_base_{ros_distro}")
    config_path = write_config(
        tmp_path / f"alt-base-{ros_distro}.ros2docker.json",
        {
            "container_name": container,
            "image_name": image,
            "run_type": "command",
            "command": [
                "bash",
                "-lc",
                f'test "$ROS_DISTRO" = {ros_distro} && ros2 --help >/tmp/ros2_help && echo E2E_ALT_BASE_OK',
            ],
            "bake_ros_packages": [str(FIXTURES_ROOT / "bake" / "e2e_msgs")],
            "build_args": {
                "BASE_IMAGE": base_image,
                "DIGEST": "",
            },
        },
    )

    docker_harness.cli("build", "-f", str(config_path), timeout=2400)
    result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=180)

    assert "E2E_ALT_BASE_OK" in result.stdout


@pytest.mark.skipif(shutil.which("Xvfb") is None, reason="Xvfb is not installed on the host")
def test_gui_forwarding_forwards_x_socket(docker_harness, shared_image: str, tmp_path: Path) -> None:
    # Prove GUI (X11) forwarding works end to end: a real X server (Xvfb) on the
    # host, then a GUI-forwarding container whose `xdpyinfo` succeeds against the
    # forwarded /tmp/.X11-unix socket. `-ac` disables X access control so the
    # container (running as the same UID) can connect without an Xauthority.
    display = ":99"
    x11_socket = Path("/tmp/.X11-unix") / f"X{display.lstrip(':')}"
    xvfb = subprocess.Popen(
        ["Xvfb", display, "-screen", "0", "1024x768x24", "-ac", "-nolisten", "tcp"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        wait_for_file(x11_socket, timeout=15)
        container = docker_harness.container_name("gui_forward")
        command = "xdpyinfo >/tmp/xdpyinfo.log 2>&1 && echo E2E_GUI_OK"
        config_path = write_config(
            tmp_path / "gui.ros2docker.json",
            {
                "container_name": container,
                "image_name": shared_image,
                "enable_gui_forwarding": True,
                "run_type": "command",
                "command": ["bash", "-lc", command],
            },
        )

        result = docker_harness.cli("run", "--no-build", "-f", str(config_path), timeout=120, env={"DISPLAY": display})

        assert "E2E_GUI_OK" in result.stdout
    finally:
        xvfb.terminate()
        try:
            xvfb.wait(timeout=10)
        except subprocess.TimeoutExpired:
            xvfb.kill()


@pytest.mark.skipif(
    any(shutil.which(tool) is None for tool in ("ssh-agent", "ssh-add", "ssh-keygen")),
    reason="openssh-client is not installed on the host",
)
def test_ssh_agent_forwarding_socket_is_usable(docker_harness, shared_image: str, tmp_path: Path) -> None:
    # Prove SSH-agent forwarding works end to end: an ephemeral host agent holding
    # a throwaway key, then a `forward_ssh_agent` container whose `ssh-add -l`
    # lists that key over the forwarded agent socket.
    key = tmp_path / "id_ed25519"
    subprocess.run(
        ["ssh-keygen", "-t", "ed25519", "-N", "", "-f", str(key)],
        check=True,
        capture_output=True,
    )
    agent = subprocess.run(["ssh-agent", "-s"], check=True, capture_output=True, text=True)
    auth_sock = _parse_agent_var(agent.stdout, "SSH_AUTH_SOCK")
    agent_pid = _parse_agent_var(agent.stdout, "SSH_AGENT_PID")
    try:
        subprocess.run(
            ["ssh-add", str(key)],
            check=True,
            capture_output=True,
            env={**os.environ, "SSH_AUTH_SOCK": auth_sock},
        )
        container = docker_harness.container_name("ssh_forward")
        command = "ssh-add -l >/tmp/ssh_add.log 2>&1 && echo E2E_SSH_OK"
        config_path = write_config(
            tmp_path / "ssh.ros2docker.json",
            {
                "container_name": container,
                "image_name": shared_image,
                "forward_ssh_agent": True,
                "run_type": "command",
                "command": ["bash", "-lc", command],
            },
        )

        result = docker_harness.cli(
            "run", "--no-build", "-f", str(config_path), timeout=120, env={"SSH_AUTH_SOCK": auth_sock}
        )

        assert "E2E_SSH_OK" in result.stdout
    finally:
        subprocess.run(
            ["ssh-agent", "-k"],
            check=False,
            capture_output=True,
            env={**os.environ, "SSH_AGENT_PID": agent_pid, "SSH_AUTH_SOCK": auth_sock},
        )

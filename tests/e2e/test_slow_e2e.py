from __future__ import annotations

from pathlib import Path

import pytest
from conftest import FIXTURES_ROOT, write_config

pytestmark = [pytest.mark.e2e, pytest.mark.slow]


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

from __future__ import annotations

import json
import re
from pathlib import Path

from ros2docker.config import strip_json_comments

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

SUPPORTED_CONFIG_KEYS = {
    "container_name",
    "image_name",
    "run_type",
    "mount_ws",
    "enable_gui_forwarding",
    "forward_ssh_agent",
    "run_args",
    "extra_run_args",
    "build_args",
    "bake_ros_packages",
    "catmux_file",
    "catmux_params",
    "command",
}


def _json_from_comments(text: str) -> dict[str, object]:
    return json.loads(strip_json_comments(text))


def _readme_config_block() -> dict[str, object]:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    match = re.search(r"## Config.*?```json\n(?P<json>.*?)\n```", readme, flags=re.DOTALL)
    assert match, "README.md must contain a JSON config block in the Config section."
    return _json_from_comments(match.group("json"))


def test_readme_config_surface_matches_supported_keys() -> None:
    config = _readme_config_block()

    assert set(config) <= SUPPORTED_CONFIG_KEYS
    assert "session_configs_dir" not in config


def test_example_config_surface_matches_supported_keys() -> None:
    example = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "examples" / "ros2docker.json"
    config = _json_from_comments(example.read_text(encoding="utf-8"))

    assert set(config) <= SUPPORTED_CONFIG_KEYS
    assert "session_configs_dir" not in config

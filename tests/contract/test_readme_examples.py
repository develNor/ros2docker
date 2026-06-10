from __future__ import annotations

import json
import re
from pathlib import Path

from ros2docker.config import load_config, strip_json_comments

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_CONFIG_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "examples" / "ros2docker.json"


def _json_from_comments(text: str) -> dict[str, object]:
    parsed = json.loads(strip_json_comments(text))
    assert isinstance(parsed, dict)
    return parsed


def _readme_config_block() -> dict[str, object]:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    match = re.search(r"## Config.*?```json\n(?P<json>.*?)\n```", readme, flags=re.DOTALL)
    assert match, "README.md must contain a JSON config block in the Config section."
    return _json_from_comments(match.group("json"))


def test_documented_config_block_loads() -> None:
    config = load_config(override=_readme_config_block())

    assert config["container_name"] == "example_ros2container"
    assert config["run_type"] == "bash"


def test_packaged_example_config_loads() -> None:
    config = load_config(EXAMPLE_CONFIG_PATH)

    assert config["container_name"] == "example_ros2container"
    assert config["run_type"] == "catmux"

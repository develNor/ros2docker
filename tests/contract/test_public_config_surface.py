from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from ros2docker.config import DEFAULT_CONFIG, ConfigError, load_config, public_config_keys, strip_json_comments

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "schema" / "ros2docker.schema.json"
EXAMPLE_CONFIG_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "examples" / "ros2docker.json"
PUBLIC_DOC_PATHS = (
    PACKAGE_ROOT / "README.md",
    PACKAGE_ROOT / "docs" / "configuration.md",
)
REMOVED_CONFIG_KEYS = {"session_configs_dir"}


def _json_from_comments(text: str) -> dict[str, object]:
    return json.loads(strip_json_comments(text))


def _schema() -> dict[str, object]:
    schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
    assert isinstance(schema, dict)
    return schema


def _schema_keys() -> set[str]:
    properties = _schema()["properties"]
    assert isinstance(properties, dict)
    return set(properties)


def _readme_config_block() -> dict[str, object]:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")
    match = re.search(r"## Config.*?```json\n(?P<json>.*?)\n```", readme, flags=re.DOTALL)
    assert match, "README.md must contain a JSON config block in the Config section."
    return _json_from_comments(match.group("json"))


def test_schema_closes_the_public_config_surface() -> None:
    schema = _schema()

    assert schema["type"] == "object"
    assert schema["additionalProperties"] is False


def test_runtime_public_config_keys_match_schema() -> None:
    assert public_config_keys() == _schema_keys()


def test_default_config_keys_are_in_schema() -> None:
    assert set(DEFAULT_CONFIG) <= _schema_keys()


def test_readme_config_block_keys_are_in_schema() -> None:
    config = _readme_config_block()

    assert set(config) <= _schema_keys()


def test_example_config_keys_are_in_schema() -> None:
    config = _json_from_comments(EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8"))

    assert set(config) <= _schema_keys()


def test_removed_config_keys_are_absent_from_schema_examples_and_public_docs() -> None:
    schema_keys = _schema_keys()
    public_paths = (*PUBLIC_DOC_PATHS, EXAMPLE_CONFIG_PATH, SCHEMA_PATH)

    for key in REMOVED_CONFIG_KEYS:
        assert key not in schema_keys
        for path in public_paths:
            assert key not in path.read_text(encoding="utf-8")


def test_load_config_rejects_unknown_config_file_keys(tmp_path: Path) -> None:
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"session_configs_dir": "./sessions"}', encoding="utf-8")

    with pytest.raises(
        ConfigError,
        match=r"Unknown config key 'session_configs_dir'\. This key is not part of ros2docker core\.",
    ):
        load_config(config_path)


def test_load_config_rejects_unknown_override_keys() -> None:
    with pytest.raises(
        ConfigError,
        match=r"Unknown config key 'session_configs_dir'\. This key is not part of ros2docker core\.",
    ):
        load_config(override={"session_configs_dir": "./sessions"})

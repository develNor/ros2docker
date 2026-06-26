from __future__ import annotations

import json
import re
from collections.abc import Mapping
from pathlib import Path

import pytest

from ros2docker.config import (
    DEFAULT_CONFIG,
    VALID_RUN_TYPES,
    ConfigError,
    load_config,
    public_config_keys,
    strip_json_comments,
)

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


def _json_type_matches(value: object, type_name: str) -> bool:
    if type_name == "array":
        return isinstance(value, list)
    if type_name == "boolean":
        return isinstance(value, bool)
    if type_name == "null":
        return value is None
    if type_name == "number":
        return isinstance(value, int | float) and not isinstance(value, bool)
    if type_name == "object":
        return isinstance(value, dict)
    if type_name == "string":
        return isinstance(value, str)
    raise AssertionError(f"Unhandled schema type in test helper: {type_name}")


def _schema_fragment_accepts(value: object, fragment: Mapping[str, object]) -> bool:
    if "oneOf" in fragment:
        options = fragment["oneOf"]
        assert isinstance(options, list)
        return sum(_schema_fragment_accepts(value, option) for option in options if isinstance(option, dict)) == 1

    type_spec = fragment.get("type")
    if isinstance(type_spec, str):
        type_names = [type_spec]
    elif isinstance(type_spec, list):
        type_names = [item for item in type_spec if isinstance(item, str)]
    else:
        type_names = []

    if type_names and not any(_json_type_matches(value, type_name) for type_name in type_names):
        return False

    if isinstance(value, list) and isinstance(fragment.get("items"), dict):
        return all(_schema_fragment_accepts(item, fragment["items"]) for item in value)

    if isinstance(value, dict) and isinstance(fragment.get("additionalProperties"), dict):
        additional = fragment["additionalProperties"]
        return all(_schema_fragment_accepts(item, additional) for item in value.values())

    return True


def _assert_config_matches_schema(config: dict[str, object], schema: dict[str, object]) -> None:
    properties = schema["properties"]
    assert isinstance(properties, dict)
    assert set(config) <= set(properties)

    for key, value in config.items():
        fragment = properties[key]
        assert isinstance(fragment, dict)
        assert _schema_fragment_accepts(value, fragment), f"{key!r} does not match its schema fragment"

    all_of = schema.get("allOf", [])
    assert isinstance(all_of, list)
    for rule in all_of:
        assert isinstance(rule, dict)
        condition = rule.get("if", {})
        requirement = rule.get("then", {})
        assert isinstance(condition, dict)
        assert isinstance(requirement, dict)
        condition_properties = condition.get("properties", {})
        assert isinstance(condition_properties, dict)
        run_type = condition_properties.get("run_type", {})
        if isinstance(run_type, dict) and config.get("run_type") == run_type.get("const"):
            required = requirement.get("required", [])
            assert isinstance(required, list)
            assert set(required) <= set(config)


def test_schema_closes_the_public_config_surface() -> None:
    schema = _schema()

    assert schema["type"] == "object"
    assert schema["additionalProperties"] is False


def test_runtime_public_config_keys_match_schema() -> None:
    assert public_config_keys() == _schema_keys()


def test_default_config_keys_are_in_schema() -> None:
    assert set(DEFAULT_CONFIG) <= _schema_keys()


def test_schema_run_type_enum_matches_runtime_values() -> None:
    run_type = _schema()["properties"]["run_type"]
    assert isinstance(run_type, dict)

    assert set(run_type["enum"]) == VALID_RUN_TYPES


def test_schema_types_match_runtime_validation_expectations() -> None:
    properties = _schema()["properties"]
    assert isinstance(properties, dict)

    assert properties["mount_ws"]["type"] == "boolean"
    assert properties["tty"]["type"] == "boolean"
    assert properties["stdin_open"]["type"] == "boolean"
    profile_options = properties["profile"]["oneOf"]
    assert {option.get("type") for option in profile_options} == {"null", "string", "array"}
    profile_array = next(option for option in profile_options if option.get("type") == "array")
    assert profile_array["items"]["type"] == "string"
    assert properties["dockerfile"]["type"] == "string"
    assert properties["build_args"]["type"] == "object"
    assert properties["build_args"]["additionalProperties"]["type"] == ["string", "number", "boolean", "null"]
    assert properties["catmux_params"]["type"] == "object"
    assert properties["catmux_params"]["additionalProperties"]["type"] == ["string", "number", "boolean", "null"]
    assert properties["extra_run_args"]["type"] == "array"
    assert properties["extra_run_args"]["items"]["type"] == "string"
    assert properties["bake_ros_packages"]["type"] == "array"
    assert properties["bake_ros_packages"]["items"]["type"] == "string"

    command = properties["command"]
    assert isinstance(command, dict)
    command_options = command["oneOf"]
    assert isinstance(command_options, list)
    assert {option["type"] for option in command_options if isinstance(option, dict)} == {"string", "array"}
    command_array = next(option for option in command_options if isinstance(option, dict) and option["type"] == "array")
    assert command_array["items"]["type"] == ["string", "number", "boolean"]


def test_readme_config_block_keys_are_in_schema() -> None:
    config = _readme_config_block()

    assert set(config) <= _schema_keys()


def test_example_config_keys_are_in_schema() -> None:
    config = _json_from_comments(EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8"))

    assert set(config) <= _schema_keys()


def test_packaged_example_validates_against_schema_and_loads() -> None:
    schema = _schema()
    example = _json_from_comments(EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8"))

    _assert_config_matches_schema(example, schema)
    loaded = load_config(EXAMPLE_CONFIG_PATH)

    assert loaded["container_name"] == example["container_name"]
    assert loaded["run_type"] == example["run_type"]


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
        match=r"Invalid ros2docker config: root: Additional properties are not allowed",
    ):
        load_config(config_path)


def test_load_config_rejects_unknown_override_keys() -> None:
    with pytest.raises(
        ConfigError,
        match=r"Invalid ros2docker config: root: Additional properties are not allowed",
    ):
        load_config(override={"session_configs_dir": "./sessions"})

"""Configuration loading and path resolution for ros2docker."""

from __future__ import annotations

import copy
import json
import os
from collections.abc import Mapping, Sequence
from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


class ConfigError(ValueError):
    """Raised when a ros2docker config is invalid."""


DEFAULT_CONFIG: dict[str, Any] = {
    "container_name": "ros2docker",
    "image_name": "ros2docker",
    "profile": None,
    "dockerfile": "Dockerfile.generic",
    "run_type": "bash",
    "mount_ws": False,
    "enable_gui_forwarding": False,
    "forward_ssh_agent": False,
    "run_args": [],
    "extra_run_args": [],
    "build_args": {},
    "bake_ros_packages": [],
}

VALID_RUN_TYPES = {"bash", "catmux", "up", "command"}


@lru_cache
def config_schema() -> dict[str, Any]:
    """Return the packaged ros2docker JSON Schema."""

    schema_text = (
        resources.files("ros2docker")
        .joinpath("resources")
        .joinpath("schema")
        .joinpath("ros2docker.schema.json")
        .read_text(encoding="utf-8")
    )
    schema = json.loads(schema_text)
    if not isinstance(schema, dict):
        raise ConfigError("ros2docker config schema must be a JSON object.")
    return schema


@lru_cache
def public_config_keys() -> frozenset[str]:
    """Return the supported top-level ros2docker config keys."""

    schema = config_schema()
    properties = schema.get("properties", {})
    if not isinstance(properties, dict):
        raise ConfigError("ros2docker config schema must define object properties.")
    return frozenset(str(key) for key in properties)


def strip_json_comments(text: str) -> str:
    """Remove C/C++ style comments while preserving string literals."""

    output: list[str] = []
    in_string = False
    escape = False
    i = 0

    while i < len(text):
        char = text[i]
        next_char = text[i + 1] if i + 1 < len(text) else ""

        if in_string:
            output.append(char)
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            in_string = True
            output.append(char)
            i += 1
            continue

        if char == "/" and next_char == "/":
            i += 2
            while i < len(text) and text[i] not in "\r\n":
                i += 1
            continue

        if char == "/" and next_char == "*":
            i += 2
            while i + 1 < len(text) and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            if i + 1 >= len(text):
                raise ConfigError("Unterminated block comment in JSON input.")
            i += 2
            continue

        output.append(char)
        i += 1

    return "".join(output)


def parse_override(override: str | Mapping[str, Any] | None) -> dict[str, Any]:
    if override is None:
        return {}
    if isinstance(override, str):
        try:
            parsed = json.loads(strip_json_comments(override))
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid JSON override: {exc}") from exc
    elif isinstance(override, Mapping):
        parsed = dict(override)
    else:
        raise ConfigError(f"Override must be JSON text or a mapping, got {type(override).__name__}.")

    if not isinstance(parsed, dict):
        raise ConfigError("Override must decode to a JSON object.")
    return parsed


def get_config_dir(config_file: str | os.PathLike[str] | None = None) -> str:
    if config_file is None:
        return os.getcwd()
    return str(_resolve_config_file(config_file).parent)


def load_profile(profile_name: str) -> dict[str, Any]:
    """Load a predefined ros2docker profile from resources."""
    try:
        profile_text = (
            resources.files("ros2docker")
            .joinpath("resources")
            .joinpath("profiles")
            .joinpath(f"{profile_name}.json")
            .read_text(encoding="utf-8")
        )
    except FileNotFoundError as exc:
        raise ConfigError(f"Profile {profile_name!r} not found.") from exc

    try:
        profile_json = json.loads(strip_json_comments(profile_text))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"Invalid JSON in profile {profile_name!r}: {exc}") from exc

    if not isinstance(profile_json, dict):
        raise ConfigError(f"Profile {profile_name!r} must be a JSON object.")

    return profile_json


# Build args whose values are space-separated package lists. When several
# profiles (or a profile and a config) contribute these, the lists are unioned
# instead of replaced so add-on profiles compose.
ADDITIVE_BUILD_ARGS = ("APT_PACKAGES", "PIP_PACKAGES")


def merge_configs(base: dict[str, Any], overlay: dict[str, Any]) -> None:
    """Deep merge dictionary values from overlay into base while replacing lists/scalars."""
    for key, val in overlay.items():
        if key in base and isinstance(base[key], dict) and isinstance(val, dict):
            base[key] = _merge_mapping(base[key], val)
        else:
            base[key] = copy.deepcopy(val)


def _merge_mapping(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, val in overlay.items():
        if key in ADDITIVE_BUILD_ARGS and isinstance(merged.get(key), str) and isinstance(val, str):
            merged[key] = _merge_package_list(merged[key], val)
        else:
            merged[key] = copy.deepcopy(val)
    return merged


def _merge_package_list(existing: str, addition: str) -> str:
    """Union two space-separated package lists, preserving first-seen order."""
    tokens: list[str] = []
    for token in (*existing.split(), *addition.split()):
        if token not in tokens:
            tokens.append(token)
    return " ".join(tokens)


def _normalize_profile_names(profile_value: Any) -> list[str]:
    """Normalize a profile config value to an ordered list of profile names."""
    if profile_value is None:
        return []
    if isinstance(profile_value, str):
        return [profile_value]
    if isinstance(profile_value, list):
        names: list[str] = []
        for item in profile_value:
            if not isinstance(item, str):
                raise ConfigError("Profile names must be strings.")
            names.append(item)
        return names
    raise ConfigError("Profile must be a string, a list of strings, or null.")


def load_config(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, Any] | None = None,
    *,
    resolve_run_args: bool = True,
) -> dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    config_dir = Path.cwd()

    override_config = parse_override(override)

    file_config: dict[str, Any] = {}
    if config_file is not None:
        config_path = _resolve_config_file(config_file)
        if not config_path.is_file():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        config_dir = config_path.parent
        try:
            file_config = json.loads(strip_json_comments(config_path.read_text(encoding="utf-8")))
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid JSON in {config_path}: {exc}") from exc
        if not isinstance(file_config, dict):
            raise ConfigError(f"Config file must contain a JSON object: {config_path}")

    # Determine which profile(s) to load (override takes precedence over file config).
    # A profile may be a single name or an ordered list of add-on profiles.
    profile_value = override_config.get("profile")
    if profile_value is None:
        profile_value = file_config.get("profile")

    for profile_name in _normalize_profile_names(profile_value):
        profile_config = load_profile(profile_name)
        merge_configs(config, profile_config)

    # Merge file config and overrides
    merge_configs(config, file_config)
    merge_configs(config, override_config)

    _apply_interactivity_defaults(config)
    _validate_config(config)
    _normalize_config_paths(config, config_dir, resolve_run_args=resolve_run_args)
    return config


def _apply_interactivity_defaults(config: dict[str, Any]) -> None:
    interactive = config.get("run_type") in {"bash", "catmux"}
    config.setdefault("tty", interactive)
    config.setdefault("stdin_open", interactive)


def _resolve_config_file(config_file: str | os.PathLike[str]) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(os.fspath(config_file))))
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def _validate_public_config_keys(config: Mapping[str, Any]) -> None:
    unknown_keys = sorted(set(config) - public_config_keys())
    if unknown_keys:
        key = unknown_keys[0]
        raise ConfigError(f"Unknown config key {key!r}. This key is not part of ros2docker core.")


@lru_cache
def _schema_validator() -> Draft202012Validator:
    schema = config_schema()
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


def _validate_config(config: Mapping[str, Any]) -> None:
    errors = sorted(_schema_validator().iter_errors(config), key=_validation_error_sort_key)
    if errors:
        shown = "; ".join(_format_validation_error(error) for error in errors[:3])
        if len(errors) > 3:
            shown = f"{shown}; and {len(errors) - 3} more error(s)"
        raise ConfigError(f"Invalid ros2docker config: {shown}")


def _validation_error_sort_key(error: ValidationError) -> tuple[list[str], str]:
    return ([str(part) for part in error.absolute_path], error.message)


def _format_validation_error(error: ValidationError) -> str:
    path = ".".join(str(part) for part in error.absolute_path)
    location = path or "root"
    return f"{location}: {error.message}"


def _normalize_config_paths(config: dict[str, Any], config_dir: Path, *, resolve_run_args: bool) -> None:
    if resolve_run_args:
        config["run_args"] = normalize_docker_host_paths(config.get("run_args", []), config_dir)
        config["extra_run_args"] = normalize_docker_host_paths(config.get("extra_run_args", []), config_dir)

    bake_packages = []
    for raw_path in config.get("bake_ros_packages", []):
        resolved = resolve_host_path(raw_path, config_dir)
        if not resolved.exists():
            raise FileNotFoundError(f"bake_ros_packages path does not exist: {raw_path!r} (resolved to {resolved})")
        if not resolved.is_dir():
            raise NotADirectoryError(
                f"bake_ros_packages path must be a directory: {raw_path!r} (resolved to {resolved})"
            )
        bake_packages.append(str(resolved))
    config["bake_ros_packages"] = bake_packages


def normalize_docker_host_paths(
    args: Sequence[str],
    base_dir: str | os.PathLike[str],
) -> list[str]:
    normalized: list[str] = []
    i = 0
    base_path = Path(base_dir)
    args_list = list(args)

    while i < len(args_list):
        arg = args_list[i]

        if arg in {"-v", "--volume"}:
            if i + 1 >= len(args_list):
                raise ConfigError(f"{arg} requires a following volume spec.")
            normalized.extend([arg, normalize_volume_spec(args_list[i + 1], base_path)])
            i += 2
            continue

        if arg.startswith("--volume="):
            volume = arg.split("=", 1)[1]
            normalized.append(f"--volume={normalize_volume_spec(volume, base_path)}")
            i += 1
            continue

        if arg == "--mount":
            if i + 1 >= len(args_list):
                raise ConfigError("--mount requires a following mount spec.")
            normalized.extend([arg, normalize_mount_spec(args_list[i + 1], base_path)])
            i += 2
            continue

        if arg.startswith("--mount="):
            mount = arg.split("=", 1)[1]
            normalized.append(f"--mount={normalize_mount_spec(mount, base_path)}")
            i += 1
            continue

        normalized.append(arg)
        i += 1

    return normalized


def normalize_volume_spec(spec: str, base_dir: Path) -> str:
    parts = spec.split(":")
    if len(parts) < 2:
        return spec

    host_path = parts[0]
    if not _looks_like_host_path(host_path):
        return spec

    resolved = resolve_host_path(host_path, base_dir)
    parts[0] = str(resolved)
    return ":".join(parts)


def normalize_mount_spec(spec: str, base_dir: Path) -> str:
    fields = spec.split(",")
    parsed: list[str] = []
    is_bind = any(field == "type=bind" for field in fields)

    for field in fields:
        key, sep, value = field.partition("=")
        if is_bind and sep and key in {"source", "src"}:
            parsed.append(f"{key}={resolve_host_path(value, base_dir)}")
        else:
            parsed.append(field)

    return ",".join(parsed)


def resolve_host_path(
    raw_path: str | os.PathLike[str],
    base_dir: str | os.PathLike[str],
) -> Path:
    text = os.fspath(raw_path)
    expanded = os.path.expanduser(os.path.expandvars(text))
    if "$" in expanded:
        raise ConfigError(f"Could not expand environment variables in host path: {text!r}")

    path = Path(expanded)
    if _is_relative_path(text):
        path = Path(base_dir) / expanded
    elif not path.is_absolute() and _looks_like_host_path(text):
        path = Path.cwd() / expanded

    resolved = path.resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Host path does not exist: {text!r} (resolved to {resolved})")
    return resolved


def _is_relative_path(path: str) -> bool:
    return path in {".", ".."} or path.startswith("./") or path.startswith("../")


def _looks_like_host_path(path: str) -> bool:
    expanded = os.path.expanduser(os.path.expandvars(path))
    return (
        path in {".", ".."}
        or path.startswith("./")
        or path.startswith("../")
        or path.startswith("~")
        or path.startswith("$")
        or os.path.isabs(expanded)
    )

"""Configuration loading and path resolution for ros2docker."""

from __future__ import annotations

import copy
import json
import os
from pathlib import Path
from typing import Any, Mapping, Sequence


class ConfigError(ValueError):
    """Raised when a ros2docker config is invalid."""


DEFAULT_CONFIG: dict[str, Any] = {
    "container_name": "ros2docker",
    "image_name": "ros2docker",
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


def load_config(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, Any] | None = None,
    *,
    resolve_run_args: bool = True,
) -> dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    config_dir = Path.cwd()

    if config_file is not None:
        config_path = _resolve_config_file(config_file)
        if not config_path.is_file():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        config_dir = config_path.parent
        try:
            loaded = json.loads(strip_json_comments(config_path.read_text(encoding="utf-8")))
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid JSON in {config_path}: {exc}") from exc
        if not isinstance(loaded, dict):
            raise ConfigError(f"Config file must contain a JSON object: {config_path}")
        config.update(loaded)

    config.update(parse_override(override))
    _normalize_config_paths(config, config_dir, resolve_run_args=resolve_run_args)
    _validate_config(config)
    return config


def _resolve_config_file(config_file: str | os.PathLike[str]) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(os.fspath(config_file))))
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def _validate_config(config: Mapping[str, Any]) -> None:
    run_type = config.get("run_type", "bash")
    if run_type not in VALID_RUN_TYPES:
        raise ConfigError(
            f"Unsupported run_type {run_type!r}. Expected one of: {', '.join(sorted(VALID_RUN_TYPES))}."
        )

    for list_key in ("run_args", "extra_run_args", "bake_ros_packages"):
        if not isinstance(config.get(list_key, []), list):
            raise ConfigError(f"{list_key!r} must be a list.")

    if not isinstance(config.get("build_args", {}), dict):
        raise ConfigError("'build_args' must be a JSON object.")

    if run_type == "catmux" and not config.get("catmux_file"):
        raise ConfigError("'catmux_file' is required when run_type is 'catmux'.")

    if run_type == "command":
        command = config.get("command")
        if not isinstance(command, (str, list)):
            raise ConfigError("'command' must be a string or list when run_type is 'command'.")


def _normalize_config_paths(config: dict[str, Any], config_dir: Path, *, resolve_run_args: bool) -> None:
    if resolve_run_args:
        config["run_args"] = normalize_docker_host_paths(config.get("run_args", []), config_dir)
        config["extra_run_args"] = normalize_docker_host_paths(config.get("extra_run_args", []), config_dir)

    bake_packages = []
    for raw_path in config.get("bake_ros_packages", []):
        resolved = resolve_host_path(raw_path, config_dir)
        if not resolved.exists():
            raise FileNotFoundError(
                f"bake_ros_packages path does not exist: {raw_path!r} (resolved to {resolved})"
            )
        if not resolved.is_dir():
            raise NotADirectoryError(
                f"bake_ros_packages path must be a directory: {raw_path!r} (resolved to {resolved})"
            )
        bake_packages.append(str(resolved))
    config["bake_ros_packages"] = bake_packages

    session_configs_dir = config.get("session_configs_dir")
    if isinstance(session_configs_dir, str) and _is_relative_path(session_configs_dir):
        config["session_configs_dir"] = str(resolve_host_path(session_configs_dir, config_dir, must_exist=False))


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
    *,
    must_exist: bool = True,
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
    if must_exist and not resolved.exists():
        raise FileNotFoundError(f"Host path does not exist: {text!r} (resolved to {resolved})")
    return resolved


def _is_relative_path(path: str) -> bool:
    return path in {".", ".."} or path.startswith("./") or path.startswith("../")


def _looks_like_host_path(path: str) -> bool:
    expanded = os.path.expanduser(os.path.expandvars(path))
    return (
        path in {".", ".."}
        or
        path.startswith("./")
        or path.startswith("../")
        or path.startswith("~")
        or path.startswith("$")
        or os.path.isabs(expanded)
    )

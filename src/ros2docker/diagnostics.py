"""Read-only host diagnostics for ros2docker configs."""

from __future__ import annotations

import os
import shutil
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

from .config import get_config_dir, load_config


@dataclass(frozen=True)
class Diagnostic:
    status: str
    name: str
    detail: str


def collect_diagnostics(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
) -> list[Diagnostic]:
    diagnostics: list[Diagnostic] = []

    try:
        config = load_config(config_file, override)
        config_dir = Path(get_config_dir(config_file))
    except Exception as exc:  # noqa: BLE001 - doctor reports user-facing config failures.
        return [Diagnostic("ERROR", "config", str(exc))]

    diagnostics.append(Diagnostic("OK", "config", "Config syntax, schema, and local path resolution passed."))
    diagnostics.extend(_docker_diagnostics())
    diagnostics.extend(_build_context_diagnostics(config))
    diagnostics.extend(_workspace_diagnostics(config, config_dir))
    diagnostics.extend(_gui_diagnostics(config))
    diagnostics.extend(_ssh_agent_diagnostics(config))
    diagnostics.append(_gpu_device_diagnostic(config))
    return diagnostics


def diagnostics_exit_code(diagnostics: list[Diagnostic]) -> int:
    return 1 if any(diagnostic.status == "ERROR" for diagnostic in diagnostics) else 0


def format_diagnostics(diagnostics: list[Diagnostic]) -> str:
    return "\n".join(f"{diagnostic.status}: {diagnostic.name}: {diagnostic.detail}" for diagnostic in diagnostics)


def _docker_diagnostics() -> list[Diagnostic]:
    docker = shutil.which("docker")
    if docker is None:
        return [Diagnostic("WARN", "docker", "Docker CLI was not found on PATH.")]

    try:
        result = subprocess.run(
            [docker, "info"],
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return [Diagnostic("WARN", "docker", f"Docker CLI was found, but `docker info` failed: {exc}.")]

    if result.returncode == 0:
        return [Diagnostic("OK", "docker", "Docker CLI and daemon are available.")]

    detail = result.stderr.strip() or result.stdout.strip() or f"`docker info` exited {result.returncode}."
    return [Diagnostic("WARN", "docker", detail)]


def _build_context_diagnostics(config: Mapping[str, object]) -> list[Diagnostic]:
    build_resources = resources.files("ros2docker").joinpath("resources").joinpath("build")
    dockerfile = build_resources.joinpath("Dockerfile")
    entrypoint = build_resources.joinpath("entrypoint.sh")
    diagnostics = [Diagnostic("OK", "image", f"Image name is {config.get('image_name', 'ros2docker')!r}.")]

    if dockerfile.is_file() and entrypoint.is_file():
        diagnostics.append(Diagnostic("OK", "build context", "Packaged Dockerfile and entrypoint are available."))
    else:
        diagnostics.append(Diagnostic("ERROR", "build context", "Packaged Dockerfile or entrypoint is missing."))

    return diagnostics


def _workspace_diagnostics(config: Mapping[str, object], config_dir: Path) -> list[Diagnostic]:
    if not config.get("mount_ws"):
        return [Diagnostic("INFO", "workspace", "`mount_ws` is false; no config-adjacent workspace is required.")]

    workspace = config_dir / "ws"
    if not workspace.exists():
        return [Diagnostic("ERROR", "workspace", f"`mount_ws` is true, but {workspace} does not exist.")]
    if not workspace.is_dir():
        return [Diagnostic("ERROR", "workspace", f"`mount_ws` is true, but {workspace} is not a directory.")]

    ros2src = workspace / "ros2src"
    if ros2src.is_dir():
        return [Diagnostic("OK", "workspace", f"Workspace and ROS 2 source layout found at {ros2src}.")]
    return [Diagnostic("WARN", "workspace", f"{workspace} exists, but {ros2src} is missing.")]


def _gui_diagnostics(config: Mapping[str, object]) -> list[Diagnostic]:
    if not config.get("enable_gui_forwarding"):
        return [Diagnostic("INFO", "x11", "GUI forwarding is disabled.")]

    diagnostics: list[Diagnostic] = []
    if os.environ.get("DISPLAY"):
        diagnostics.append(Diagnostic("OK", "x11 DISPLAY", "DISPLAY is set."))
    else:
        diagnostics.append(Diagnostic("ERROR", "x11 DISPLAY", "GUI forwarding is enabled, but DISPLAY is not set."))

    x11_socket = Path("/tmp/.X11-unix")
    if x11_socket.exists():
        diagnostics.append(Diagnostic("OK", "x11 socket", "/tmp/.X11-unix exists."))
    else:
        diagnostics.append(
            Diagnostic("ERROR", "x11 socket", "GUI forwarding is enabled, but /tmp/.X11-unix is missing.")
        )
    return diagnostics


def _ssh_agent_diagnostics(config: Mapping[str, object]) -> list[Diagnostic]:
    if not config.get("forward_ssh_agent"):
        return [Diagnostic("INFO", "ssh agent", "SSH agent forwarding is disabled.")]

    ssh_auth_sock = os.environ.get("SSH_AUTH_SOCK")
    if not ssh_auth_sock:
        return [Diagnostic("ERROR", "ssh agent", "SSH agent forwarding is enabled, but SSH_AUTH_SOCK is not set.")]

    ssh_auth_sock_path = Path(ssh_auth_sock)
    if ssh_auth_sock_path.exists():
        return [Diagnostic("OK", "ssh agent", f"SSH_AUTH_SOCK exists at {ssh_auth_sock_path}.")]
    return [Diagnostic("ERROR", "ssh agent", f"SSH_AUTH_SOCK does not exist: {ssh_auth_sock_path}.")]


def _gpu_device_diagnostic(config: Mapping[str, object]) -> Diagnostic:
    args = [str(item) for key in ("run_args", "extra_run_args") for item in _list_config_value(config, key)]
    joined = " ".join(args)
    if "--gpus" in args or "/dev/dri" in joined or "nvidia" in joined.lower():
        return Diagnostic("OK", "gpu/device", "GPU or device-related Docker run args are configured.")
    return Diagnostic("INFO", "gpu/device", "No GPU or device-related Docker run args are configured.")


def _list_config_value(config: Mapping[str, object], key: str) -> list[object]:
    value = config.get(key, [])
    if isinstance(value, list):
        return value
    return []

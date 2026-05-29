"""Docker command rendering for ros2docker."""

from __future__ import annotations

import os
import shlex
from pathlib import Path
from typing import Mapping, Sequence

from .config import ConfigError, get_config_dir, load_config, normalize_docker_host_paths, resolve_host_path


def make_build_command(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    context_dir: str | os.PathLike[str],
) -> list[str]:
    config = load_config(config_file, override, resolve_run_args=False)
    build_args = [
        "-t",
        _image_name(config),
        "--build-arg",
        f"USER_UID={os.getuid()}",
        "--build-arg",
        f"USER_GID={os.getgid()}",
    ]

    for key, value in config.get("build_args", {}).items():
        build_args.extend(["--build-arg", f"{key}={value}"])

    return ["docker", "build", *build_args, str(Path(context_dir))]


def make_run_command(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    mount: str | os.PathLike[str] | None = None,
    extra_run_args: Sequence[str] | None = None,
) -> list[str]:
    config = load_config(config_file, override)
    run_args = [
        "docker",
        "run",
        *_core_run_args(config),
        *_local_run_args(config),
        *_workspace_mount_args(config_file, config, mount),
        *normalize_docker_host_paths(extra_run_args or [], Path.cwd()),
        *_run_type_args(config),
        _image_name(config),
        *_run_command(config),
    ]
    return run_args


def make_stop_command(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
) -> list[str]:
    config = load_config(config_file, override, resolve_run_args=False)
    return ["docker", "stop", _container_name(config)]


def make_exec_shell_command(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    command: Sequence[str] | None = None,
    interactive: bool = True,
) -> list[str]:
    config = load_config(config_file, override, resolve_run_args=False)
    exec_args = ["docker", "exec"]
    if interactive:
        exec_args.append("-it")
    exec_args.append(_container_name(config))
    exec_args.extend(command or ["bash"])
    return exec_args


def _image_name(config: Mapping[str, object]) -> str:
    return str(config.get("image_name") or "ros2docker")


def _container_name(config: Mapping[str, object]) -> str:
    return str(config.get("container_name") or config.get("image_name") or "ros2docker")


def _core_run_args(config: Mapping[str, object]) -> list[str]:
    return [
        "--name",
        _container_name(config),
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--rm",
        "-e",
        "LIBGL_ALWAYS_SOFTWARE=1",
    ]


def _local_run_args(config: Mapping[str, object]) -> list[str]:
    args: list[str] = []

    if config.get("enable_gui_forwarding"):
        x11_socket = Path("/tmp/.X11-unix")
        if not x11_socket.exists():
            raise FileNotFoundError("GUI forwarding requested but /tmp/.X11-unix does not exist.")
        args.extend(["-v", "/tmp/.X11-unix:/tmp/.X11-unix", "-e", "DISPLAY"])

    if config.get("forward_ssh_agent"):
        ssh_auth_sock = os.environ.get("SSH_AUTH_SOCK")
        if not ssh_auth_sock:
            raise ConfigError("forward_ssh_agent is true but SSH_AUTH_SOCK is not set.")
        sock_path = Path(ssh_auth_sock)
        if not sock_path.exists():
            raise FileNotFoundError(
                f"forward_ssh_agent is true but SSH_AUTH_SOCK does not exist: {ssh_auth_sock}"
            )
        args.extend(["-e", "SSH_AUTH_SOCK", "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"])

    args.extend(config.get("run_args", []))
    args.extend(config.get("extra_run_args", []))
    return args


def _workspace_mount_args(
    config_file: str | os.PathLike[str] | None,
    config: Mapping[str, object],
    mount: str | os.PathLike[str] | None,
) -> list[str]:
    if mount is not None:
        mount_path = resolve_host_path(os.fspath(mount), Path.cwd())
        return ["-v", f"{mount_path}:/ws", "-w", "/ws"]

    if config.get("mount_ws"):
        ws_host = Path(get_config_dir(config_file)) / "ws"
        if not ws_host.exists():
            raise FileNotFoundError(
                f"mount_ws is true but workspace directory does not exist: {ws_host}"
            )
        return ["-v", f"{ws_host.resolve()}:/ws", "-w", "/ws"]

    return []


def _run_type_args(config: Mapping[str, object]) -> list[str]:
    run_type = str(config.get("run_type") or "bash")
    if run_type in {"bash", "catmux"}:
        return ["-it"]
    if run_type == "command":
        return []
    if run_type == "up":
        return ["-d"]
    raise ConfigError(f"Unsupported run_type: {run_type!r}")


def _run_command(config: Mapping[str, object]) -> list[str]:
    run_type = str(config.get("run_type") or "bash")

    if run_type == "catmux":
        catmux_file = str(config["catmux_file"])
        command = [
            "catmux_create_session",
            catmux_file,
            "--session_name",
            _container_name(config),
        ]
        catmux_params = config.get("catmux_params")
        if isinstance(catmux_params, Mapping) and catmux_params:
            params = ",".join(f"{key}={value}" for key, value in catmux_params.items())
            command.extend(["--overwrite", params])
        return command

    if run_type == "bash":
        return ["bash"]

    if run_type == "up":
        return ["tail", "-f", "/dev/null"]

    if run_type == "command":
        command = config["command"]
        if isinstance(command, str):
            return shlex.split(command)
        if isinstance(command, list):
            return [str(part) for part in command]
        raise ConfigError("'command' must be a string or list.")

    raise ConfigError(f"Unsupported run_type: {run_type!r}")

"""Public Python API for ros2docker."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import tempfile
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from importlib import resources
from pathlib import Path

from .commands import (
    make_build_command,
    make_exec_shell_command,
    make_run_command,
    make_stop_command,
)
from .config import get_config_dir, load_config

RunResult = list[str] | subprocess.CompletedProcess[bytes]


def build(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    dry_run: bool = False,
) -> RunResult:
    with build_context(config_file, override) as context_dir:
        command = make_build_command(config_file, override, context_dir=context_dir)
        return _run(command, dry_run=dry_run)


def run(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    mount: str | os.PathLike[str] | None = None,
    extra_run_args: Sequence[str] | None = None,
    dry_run: bool = False,
) -> RunResult:
    command = make_run_command(
        config_file,
        override,
        mount=mount,
        extra_run_args=extra_run_args,
    )
    return _run(command, dry_run=dry_run, cwd=get_config_dir(config_file))


def build_run(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    mount: str | os.PathLike[str] | None = None,
    extra_run_args: Sequence[str] | None = None,
    dry_run: bool = False,
) -> tuple[RunResult, RunResult]:
    build_result = build(config_file, override, dry_run=dry_run)
    run_result = run(
        config_file,
        override,
        mount=mount,
        extra_run_args=extra_run_args,
        dry_run=dry_run,
    )
    return build_result, run_result


def stop(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    dry_run: bool = False,
) -> RunResult:
    command = make_stop_command(config_file, override)
    return _run(command, dry_run=dry_run, check=False)


def exec_shell(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    command: Sequence[str] | None = None,
    interactive: bool = True,
    dry_run: bool = False,
) -> RunResult:
    docker_command = make_exec_shell_command(
        config_file,
        override,
        command=command,
        interactive=interactive,
    )
    return _run(docker_command, dry_run=dry_run)


@contextmanager
def build_context(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
) -> Iterator[Path]:
    config = load_config(config_file, override, resolve_run_args=False)
    dockerfile_name = config.get("dockerfile", "Dockerfile.generic")
    with tempfile.TemporaryDirectory(prefix="ros2docker-build-") as temp_dir:
        context_dir = Path(temp_dir)
        _copy_build_resources(context_dir, dockerfile_name)
        _stage_bake_packages(config, context_dir / "bake_packages")
        yield context_dir


def _copy_build_resources(context_dir: Path, dockerfile_name: str = "Dockerfile.generic") -> None:
    build_resource = resources.files("ros2docker").joinpath("resources").joinpath("build")
    for item in build_resource.iterdir():
        if item.name.startswith("Dockerfile"):
            continue
        destination = context_dir / item.name
        with resources.as_file(item) as source:
            if source.is_dir():
                shutil.copytree(source, destination)
            else:
                shutil.copy2(source, destination)

    # Copy the selected Dockerfile as 'Dockerfile'
    dockerfile_resource = build_resource.joinpath(dockerfile_name)
    destination = context_dir / "Dockerfile"
    with resources.as_file(dockerfile_resource) as source:
        shutil.copy2(source, destination)

    (context_dir / "bake_packages").mkdir(exist_ok=True)


def _stage_bake_packages(config: Mapping[str, object], bake_dir: Path) -> None:
    seen_names: set[str] = set()
    package_paths = config.get("bake_ros_packages", [])
    if not isinstance(package_paths, list):
        raise ValueError("'bake_ros_packages' must be a list.")

    for package_path in package_paths:
        if not isinstance(package_path, str | os.PathLike):
            raise TypeError(f"bake package path must be path-like, got {type(package_path).__name__}.")
        source = Path(package_path)
        package_name = source.name
        if package_name in seen_names:
            raise ValueError(f"Duplicate bake package directory name: {package_name}")
        seen_names.add(package_name)
        shutil.copytree(source, bake_dir / package_name, symlinks=True)


def _run(
    command: Sequence[str],
    *,
    dry_run: bool,
    cwd: str | os.PathLike[str] | None = None,
    check: bool = True,
) -> RunResult:
    print(shlex.join(list(command)), flush=True)
    if dry_run:
        return list(command)
    return subprocess.run(command, check=check, cwd=cwd)


def init(
    profile: str = "minimal",
    ros_distro: str = "lyrical",
    devcontainer: bool = False,
    overwrite: bool = False,
    *,
    cwd: str | os.PathLike[str] | None = None,
) -> None:
    """Initialize a ros2docker workspace with profile, distro, and devcontainer options."""
    from .config import ConfigError, load_profile

    try:
        # Enforce that profile name exists in resources
        load_profile(profile)
    except ConfigError as exc:
        raise ValueError(str(exc)) from exc

    base_dir = Path(cwd or os.getcwd()).resolve()

    files_to_check = [
        base_dir / "ros2docker.json",
        base_dir / "ws" / ".gitignore",
        base_dir / "catmux.yaml",
    ]
    if devcontainer:
        files_to_check.append(base_dir / ".devcontainer" / "devcontainer.json")

    existing = [str(p.relative_to(base_dir)) for p in files_to_check if p.exists()]
    if existing and not overwrite:
        raise FileExistsError(
            f"Cannot initialize workspace: the following files already exist: {', '.join(existing)}. "
            "Use --overwrite to overwrite them."
        )

    # Base image lookup/generation based on distro and profile
    distro_images = {
        "lyrical": {
            "minimal": "ros:lyrical-ros-base-resolute",
            "desktop": "osrf/ros:lyrical-desktop-full-resolute",
            "foxglove": "osrf/ros:lyrical-desktop-full-resolute",
            "zenoh": "osrf/ros:lyrical-desktop-full-resolute",
            "project-develnor": "osrf/ros:lyrical-desktop-full-resolute",
        },
        "jazzy": {
            "minimal": "ros:jazzy-ros-base",
            "desktop": "osrf/ros:jazzy-desktop",
            "foxglove": "osrf/ros:jazzy-desktop",
            "zenoh": "osrf/ros:jazzy-desktop",
            "project-develnor": "osrf/ros:jazzy-desktop",
        },
        "iron": {
            "minimal": "ros:iron-ros-base",
            "desktop": "osrf/ros:iron-desktop",
            "foxglove": "osrf/ros:iron-desktop",
            "zenoh": "osrf/ros:iron-desktop",
            "project-develnor": "osrf/ros:iron-desktop",
        },
        "humble": {
            "minimal": "ros:humble-ros-base",
            "desktop": "osrf/ros:humble-desktop",
            "foxglove": "osrf/ros:humble-desktop",
            "zenoh": "osrf/ros:humble-desktop",
            "project-develnor": "osrf/ros:humble-desktop",
        },
        "rolling": {
            "minimal": "ros:rolling-ros-base",
            "desktop": "osrf/ros:rolling-desktop",
            "foxglove": "osrf/ros:rolling-desktop",
            "zenoh": "osrf/ros:rolling-desktop",
            "project-develnor": "osrf/ros:rolling-desktop",
        },
    }

    base_image = None
    if ros_distro in distro_images:
        base_image = distro_images[ros_distro].get(profile)

    if base_image is None:
        if profile == "minimal":
            base_image = f"ros:{ros_distro}-ros-base"
        else:
            base_image = f"osrf/ros:{ros_distro}-desktop"

    config_data = {
        "container_name": f"ros2container_{ros_distro}_{profile}",
        "image_name": f"ros2docker-{profile}",
        "profile": profile,
        "run_type": "bash",
        "mount_ws": True,
    }

    build_args = {}
    if ros_distro != "lyrical":
        build_args["BASE_IMAGE"] = base_image
        build_args["DIGEST"] = ""

    if build_args:
        config_data["build_args"] = build_args

    # Generate workspace directories and files
    (base_dir / "ws" / "src").mkdir(parents=True, exist_ok=True)

    gitignore_path = base_dir / "ws" / ".gitignore"
    gitignore_path.write_text("build/\ninstall/\nlog/\n", encoding="utf-8")

    catmux_path = base_dir / "catmux.yaml"
    catmux_content = (
        "---\n"
        "common:\n"
        "  before_commands:\n"
        "    - source ~/.bashrc\n"
        "\n"
        "windows:\n"
        "  - name: ros2\n"
        "    layout: tiled\n"
        "    splits:\n"
        "      - commands:\n"
        "          - ros2 topic list\n"
    )
    catmux_path.write_text(catmux_content, encoding="utf-8")

    if devcontainer:
        devcontainer_dir = base_dir / ".devcontainer"
        devcontainer_dir.mkdir(parents=True, exist_ok=True)
        devcontainer_path = devcontainer_dir / "devcontainer.json"

        devcontainer_content = {
            "name": "ros2docker workspace",
            "image": config_data["image_name"],
            "workspaceFolder": "/ws",
            "mounts": ["source=${localWorkspaceFolder}/ws,target=/ws,type=bind"],
            "customizations": {"vscode": {"settings": {}, "extensions": ["ms-iot.vscode-ros"]}},
            "remoteUser": "containeruser",
        }
        devcontainer_path.write_text(json.dumps(devcontainer_content, indent=2), encoding="utf-8")

    config_path = base_dir / "ros2docker.json"
    config_path.write_text(json.dumps(config_data, indent=2), encoding="utf-8")

    print(f"Initialized ros2docker workspace in {base_dir}")

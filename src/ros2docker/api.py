"""Public Python API for ros2docker."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import tempfile
from contextlib import contextmanager
from importlib import resources
from pathlib import Path
from typing import Iterator, Mapping, Sequence

from .commands import (
    make_build_command,
    make_exec_shell_command,
    make_run_command,
    make_stop_command,
)
from .config import get_config_dir, load_config


def build(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    dry_run: bool = False,
):
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
):
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
):
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
):
    command = make_stop_command(config_file, override)
    return _run(command, dry_run=dry_run, check=False)


def exec_shell(
    config_file: str | os.PathLike[str] | None = None,
    override: str | Mapping[str, object] | None = None,
    *,
    command: Sequence[str] | None = None,
    interactive: bool = True,
    dry_run: bool = False,
):
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
    with tempfile.TemporaryDirectory(prefix="ros2docker-build-") as temp_dir:
        context_dir = Path(temp_dir)
        _copy_build_resources(context_dir)
        _stage_bake_packages(config, context_dir / "bake_packages")
        yield context_dir


def _copy_build_resources(context_dir: Path) -> None:
    build_resource = resources.files("ros2docker").joinpath("resources", "build")
    for item in build_resource.iterdir():
        destination = context_dir / item.name
        if item.is_dir():
            shutil.copytree(item, destination)
        else:
            with resources.as_file(item) as source:
                shutil.copy2(source, destination)
    (context_dir / "bake_packages").mkdir(exist_ok=True)


def _stage_bake_packages(config: Mapping[str, object], bake_dir: Path) -> None:
    seen_names: set[str] = set()
    for package_path in config.get("bake_ros_packages", []):
        source = Path(os.fspath(package_path))
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
):
    print(shlex.join(list(command)), flush=True)
    if dry_run:
        return list(command)
    return subprocess.run(command, check=check, cwd=cwd)

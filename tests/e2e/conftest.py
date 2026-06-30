from __future__ import annotations

import json
import os
import pty
import select
import shutil
import subprocess
import sys
import time
import uuid
from collections.abc import Mapping, Sequence
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
FIXTURES_ROOT = Path(__file__).resolve().parent / "fixtures"
CONTAINER_PREFIX = "ros2docker_e2e_"
IMAGE_PREFIX = "ros2docker-e2e"


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    skip_e2e = pytest.mark.skip(reason="set ROS2DOCKER_RUN_E2E=1 to run Docker E2E tests")
    skip_slow = pytest.mark.skip(reason="set ROS2DOCKER_RUN_SLOW_E2E=1 to run slow Docker E2E tests")

    run_e2e = os.environ.get("ROS2DOCKER_RUN_E2E") == "1"
    run_slow = os.environ.get("ROS2DOCKER_RUN_SLOW_E2E") == "1"

    for item in items:
        if "e2e" in item.keywords and not run_e2e:
            item.add_marker(skip_e2e)
        elif "slow" in item.keywords and not run_slow:
            item.add_marker(skip_slow)


def write_config(path: Path, config: Mapping[str, object]) -> Path:
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return path


def copy_fixture_tree(name: str, tmp_path: Path) -> Path:
    source = FIXTURES_ROOT / name
    target = tmp_path / source.name
    shutil.copytree(source, target)
    return target


def wait_for_file(path: Path, *, timeout: float, pty_command: PtyCommand | None = None) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pty_command is not None:
            pty_command.read_available()
        if path.exists():
            return
        time.sleep(0.2)

    output = f"\nPTY output:\n{pty_command.output}" if pty_command is not None else ""
    raise AssertionError(f"Timed out waiting for {path}{output}")


class RunningCommand:
    def __init__(self, command: Sequence[str], process: subprocess.Popen[str]):
        self.command = list(command)
        self.process = process
        self.stdout = ""
        self.stderr = ""

    def wait(self, *, timeout: float, check: bool = True) -> subprocess.CompletedProcess[str]:
        try:
            stdout, stderr = self.process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            self.process.kill()
            stdout, stderr = self.process.communicate()
            raise AssertionError(
                f"Command timed out: {' '.join(self.command)}\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}"
            ) from exc

        self.stdout = stdout or ""
        self.stderr = stderr or ""
        result = subprocess.CompletedProcess(self.command, self.process.returncode, self.stdout, self.stderr)
        if check and result.returncode != 0:
            raise AssertionError(
                f"Command failed with {result.returncode}: {' '.join(self.command)}\n"
                f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
            )
        return result


class PtyCommand:
    def __init__(self, command: Sequence[str], process: subprocess.Popen[bytes], master_fd: int):
        self.command = list(command)
        self.process = process
        self.master_fd = master_fd
        self.output = ""
        self._closed = False

    def read_available(self) -> None:
        if self._closed:
            return
        while True:
            readable, _, _ = select.select([self.master_fd], [], [], 0)
            if not readable:
                return
            try:
                chunk = os.read(self.master_fd, 4096)
            except OSError:
                self._closed = True
                return
            if not chunk:
                self._closed = True
                return
            self.output += chunk.decode(errors="replace")

    def wait(self, *, timeout: float, check: bool = True) -> int:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read_available()
            returncode = self.process.poll()
            if returncode is not None:
                self.read_available()
                self.close()
                if check and returncode != 0:
                    raise AssertionError(
                        f"PTY command failed with {returncode}: {' '.join(self.command)}\n{self.output}"
                    )
                return returncode
            time.sleep(0.1)

        self.process.kill()
        self.read_available()
        self.close()
        raise AssertionError(f"PTY command timed out: {' '.join(self.command)}\n{self.output}")

    def close(self) -> None:
        if not self._closed:
            try:
                os.close(self.master_fd)
            except OSError:
                pass
            self._closed = True


class DockerHarness:
    def __init__(self, run_id: str):
        self.run_id = run_id
        self.containers: set[str] = set()
        self.images: set[str] = set()

    def container_name(self, label: str) -> str:
        name = f"{CONTAINER_PREFIX}{label}_{self.run_id}"
        self.containers.add(name)
        return name

    def image_tag(self, label: str) -> str:
        tag = f"{IMAGE_PREFIX}:{label}-{self.run_id}"
        self.images.add(tag)
        return tag

    def cli(
        self,
        *args: str,
        timeout: float = 120,
        check: bool = True,
        env: Mapping[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        return self.run(
            [sys.executable, "-m", "ros2docker.cli", *args],
            timeout=timeout,
            check=check,
            env=env,
        )

    def docker(
        self,
        *args: str,
        timeout: float = 60,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        return self.run(["docker", *args], timeout=timeout, check=check)

    def run(
        self,
        command: Sequence[str],
        *,
        timeout: float,
        check: bool,
        env: Mapping[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        completed = subprocess.run(
            list(command),
            cwd=PACKAGE_ROOT,
            env=self._env(env),
            text=True,
            capture_output=True,
            timeout=timeout,
        )
        if check and completed.returncode != 0:
            raise AssertionError(
                f"Command failed with {completed.returncode}: {' '.join(command)}\n"
                f"STDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
            )
        return completed

    def start_cli(
        self,
        *args: str,
        env: Mapping[str, str] | None = None,
    ) -> RunningCommand:
        command = [sys.executable, "-m", "ros2docker.cli", *args]
        process = subprocess.Popen(
            command,
            cwd=PACKAGE_ROOT,
            env=self._env(env),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return RunningCommand(command, process)

    def start_cli_pty(
        self,
        *args: str,
        env: Mapping[str, str] | None = None,
    ) -> PtyCommand:
        command = [sys.executable, "-m", "ros2docker.cli", *args]
        master_fd, slave_fd = pty.openpty()
        process = subprocess.Popen(
            command,
            cwd=PACKAGE_ROOT,
            env=self._env({"TERM": "xterm", **(env or {})}),
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
        )
        os.close(slave_fd)
        return PtyCommand(command, process, master_fd)

    def cleanup(self) -> None:
        for container in sorted(self.containers):
            if container.startswith(CONTAINER_PREFIX):
                self.docker("rm", "-f", container, timeout=30, check=False)
        for image in sorted(self.images):
            if image.startswith(f"{IMAGE_PREFIX}:"):
                self.docker("rmi", "-f", image, timeout=120, check=False)

    @staticmethod
    def _env(extra: Mapping[str, str] | None = None) -> dict[str, str]:
        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")
        src_path = str(PACKAGE_ROOT / "src")
        existing_pythonpath = env.get("PYTHONPATH")
        env["PYTHONPATH"] = f"{src_path}{os.pathsep}{existing_pythonpath}" if existing_pythonpath else src_path
        if extra:
            env.update(extra)
        return env


@pytest.fixture(scope="session")
def e2e_run_id() -> str:
    return uuid.uuid4().hex[:10]


@pytest.fixture(scope="session")
def docker_harness(e2e_run_id: str) -> DockerHarness:
    harness = DockerHarness(e2e_run_id)
    try:
        yield harness
    finally:
        harness.cleanup()


@pytest.fixture(scope="session")
def shared_image(docker_harness: DockerHarness, tmp_path_factory: pytest.TempPathFactory) -> str:
    image = docker_harness.image_tag("shared")
    project = tmp_path_factory.mktemp("shared-image")
    config_path = write_config(
        project / "ros2docker.json",
        {
            "profile": "project-develnor",
            "container_name": docker_harness.container_name("shared_image_build"),
            "image_name": image,
            "run_type": "command",
            "command": ["bash", "-lc", "true"],
            "bake_ros_packages": [str(FIXTURES_ROOT / "bake" / "e2e_msgs")],
            "build_args": {
                # openssh-client provides ssh-add and x11-utils provides xdpyinfo,
                # used by the GUI/SSH-agent forwarding E2E tests.
                "APT_PACKAGES": "jq openssh-client x11-utils",
                "PIP_PACKAGES": "rich",
            },
        },
    )
    docker_harness.cli("build", "-f", str(config_path), timeout=2400)
    return image

"""Behavior tests for the `doctor` host diagnostics.

These exercise the WARN/ERROR branches that are the whole point of `doctor`:
Docker availability, GUI (X11) forwarding preconditions, SSH-agent forwarding
preconditions, and workspace layout. Each test asserts an observable
``Diagnostic`` (status + name + detail), never a private helper.
"""

from __future__ import annotations

import subprocess
import types
from pathlib import Path

import pytest

from ros2docker.diagnostics import Diagnostic, collect_diagnostics, diagnostics_exit_code


def _by_name(diagnostics: list[Diagnostic], name: str) -> Diagnostic:
    matches = [d for d in diagnostics if d.name == name]
    assert matches, f"no diagnostic named {name!r} in {[d.name for d in diagnostics]}"
    return matches[0]


def _patch_x11_socket(monkeypatch: pytest.MonkeyPatch, *, present: bool) -> None:
    """Force the /tmp/.X11-unix existence check without touching the real host."""
    real_exists = Path.exists

    def fake_exists(self: Path) -> bool:
        if str(self) == "/tmp/.X11-unix":
            return present
        return real_exists(self)

    monkeypatch.setattr(Path, "exists", fake_exists)


def test_config_failure_is_reported_as_single_error() -> None:
    # An invalid run_type fails schema validation inside load_config.
    diagnostics = collect_diagnostics(override={"run_type": "bogus"})

    assert len(diagnostics) == 1
    assert diagnostics[0].status == "ERROR"
    assert diagnostics[0].name == "config"
    assert diagnostics_exit_code(diagnostics) == 1


def test_docker_available_when_info_succeeds(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: "/usr/bin/docker")
    monkeypatch.setattr(
        "ros2docker.diagnostics.subprocess.run",
        lambda *a, **k: types.SimpleNamespace(returncode=0, stdout="", stderr=""),
    )

    docker = _by_name(collect_diagnostics(override={}), "docker")
    assert docker.status == "OK"


def test_docker_warns_when_info_exits_nonzero(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: "/usr/bin/docker")
    monkeypatch.setattr(
        "ros2docker.diagnostics.subprocess.run",
        lambda *a, **k: types.SimpleNamespace(returncode=1, stdout="", stderr="daemon down"),
    )

    docker = _by_name(collect_diagnostics(override={}), "docker")
    assert docker.status == "WARN"
    assert "daemon down" in docker.detail


def test_docker_warns_when_info_raises(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: "/usr/bin/docker")

    def boom(*_a: object, **_k: object) -> object:
        raise subprocess.TimeoutExpired(cmd="docker info", timeout=5)

    monkeypatch.setattr("ros2docker.diagnostics.subprocess.run", boom)

    docker = _by_name(collect_diagnostics(override={}), "docker")
    assert docker.status == "WARN"
    assert "docker info" in docker.detail


def test_gui_ok_when_display_and_socket_present(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    monkeypatch.setenv("DISPLAY", ":0")
    _patch_x11_socket(monkeypatch, present=True)

    diagnostics = collect_diagnostics(override={"enable_gui_forwarding": True})
    assert _by_name(diagnostics, "x11 DISPLAY").status == "OK"
    assert _by_name(diagnostics, "x11 socket").status == "OK"


def test_gui_errors_when_display_unset_and_socket_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    monkeypatch.delenv("DISPLAY", raising=False)
    _patch_x11_socket(monkeypatch, present=False)

    diagnostics = collect_diagnostics(override={"enable_gui_forwarding": True})
    assert _by_name(diagnostics, "x11 DISPLAY").status == "ERROR"
    assert _by_name(diagnostics, "x11 socket").status == "ERROR"
    assert diagnostics_exit_code(diagnostics) == 1


def test_ssh_agent_errors_when_sock_unset(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    monkeypatch.delenv("SSH_AUTH_SOCK", raising=False)

    ssh = _by_name(collect_diagnostics(override={"forward_ssh_agent": True}), "ssh agent")
    assert ssh.status == "ERROR"
    assert "SSH_AUTH_SOCK is not set" in ssh.detail


def test_ssh_agent_errors_when_sock_missing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    monkeypatch.setenv("SSH_AUTH_SOCK", str(tmp_path / "missing.sock"))

    ssh = _by_name(collect_diagnostics(override={"forward_ssh_agent": True}), "ssh agent")
    assert ssh.status == "ERROR"
    assert "does not exist" in ssh.detail


def test_ssh_agent_ok_when_sock_exists(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    sock = tmp_path / "agent.sock"
    sock.write_text("", encoding="utf-8")
    monkeypatch.setenv("SSH_AUTH_SOCK", str(sock))

    ssh = _by_name(collect_diagnostics(override={"forward_ssh_agent": True}), "ssh agent")
    assert ssh.status == "OK"


def test_gpu_args_are_recognised(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)

    gpu = _by_name(collect_diagnostics(override={"run_args": ["--gpus", "all"]}), "gpu/device")
    assert gpu.status == "OK"


def test_workspace_warns_when_ros2src_missing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    (tmp_path / "ws").mkdir()
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"mount_ws": true}', encoding="utf-8")

    workspace = _by_name(collect_diagnostics(config_path), "workspace")
    assert workspace.status == "WARN"


def test_workspace_errors_when_ws_is_not_a_directory(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    (tmp_path / "ws").write_text("not a dir", encoding="utf-8")
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"mount_ws": true}', encoding="utf-8")

    workspace = _by_name(collect_diagnostics(config_path), "workspace")
    assert workspace.status == "ERROR"


def test_workspace_ok_when_ros2src_present(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setattr("ros2docker.diagnostics.shutil.which", lambda name: None)
    (tmp_path / "ws" / "ros2src").mkdir(parents=True)
    config_path = tmp_path / "ros2docker.json"
    config_path.write_text('{"mount_ws": true}', encoding="utf-8")

    workspace = _by_name(collect_diagnostics(config_path), "workspace")
    assert workspace.status == "OK"

from __future__ import annotations

import pytest

from ros2docker import completion
from ros2docker.completion import (
    DIRECTIVE_FILES,
    DIRECTIVE_LIST,
    available_profiles,
    complete,
    complete_lines,
    completion_script,
)


def test_completing_first_token_offers_subcommands_and_global_options() -> None:
    candidates, directive = complete([""])

    assert directive == DIRECTIVE_LIST
    assert {"build", "run", "stop", "exec", "validate", "doctor", "init", "completion"} <= set(candidates)
    assert "--version" in candidates
    assert "--help" in candidates


def test_completing_subcommand_options() -> None:
    candidates, directive = complete(["run", "--"])

    assert directive == DIRECTIVE_LIST
    assert "--no-build" in candidates
    assert "--mount" in candidates
    assert "--dry-run" in candidates


def test_profile_value_completion_lists_packaged_profiles() -> None:
    candidates, directive = complete(["init", "--profile", ""])

    assert directive == DIRECTIVE_LIST
    assert candidates == available_profiles()
    assert "minimal" in candidates
    assert "desktop" in candidates


def test_ros_distro_value_completion() -> None:
    candidates, directive = complete(["init", "--ros-distro", ""])

    assert directive == DIRECTIVE_LIST
    assert "lyrical" in candidates
    assert "humble" in candidates


def test_config_path_option_falls_back_to_file_completion() -> None:
    candidates, directive = complete(["run", "--config", ""])

    assert directive == DIRECTIVE_FILES
    assert candidates == []


def test_mount_option_falls_back_to_file_completion() -> None:
    _, directive = complete(["run", "-m", ""])

    assert directive == DIRECTIVE_FILES


def test_completion_subcommand_completes_shells() -> None:
    candidates, directive = complete(["completion", ""])

    assert directive == DIRECTIVE_LIST
    assert candidates == ["bash", "zsh"]


def test_value_already_supplied_returns_to_options() -> None:
    candidates, directive = complete(["init", "--profile", "minimal", ""])

    assert directive == DIRECTIVE_LIST
    assert "--ros-distro" in candidates


def test_complete_lines_appends_directive_as_last_line() -> None:
    lines = complete_lines(["init", "--ros-distro", ""])

    assert lines[-1] == DIRECTIVE_LIST
    assert "lyrical" in lines[:-1]


def test_available_profiles_handles_missing_resources(monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(_package: str):
        raise FileNotFoundError

    monkeypatch.setattr(completion.resources, "files", boom)

    assert available_profiles() == []


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_completion_script_renders_known_shells(shell: str) -> None:
    script = completion_script(shell)

    assert "_ros2docker_completion" in script
    assert "complete -F _ros2docker_completion ros2docker" in script
    assert "ros2docker __complete --" in script
    if shell == "zsh":
        assert "bashcompinit" in script


def test_completion_script_rejects_unknown_shell() -> None:
    with pytest.raises(ValueError):
        completion_script("fish")

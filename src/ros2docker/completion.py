"""Self-contained shell completion engine for the ros2docker CLI.

This module renders completion candidates for a partially typed command line and
emits the small shell wrapper scripts that wire `<TAB>` to those candidates. It
deliberately avoids a third-party completion dependency: the wrapper shells out
to a hidden ``ros2docker __complete`` command and lets the shell filter the
returned candidates by prefix (or fall back to file completion).

The subcommand/option spec below is the single source of truth for completion and
is verified against the real argparse parser by a contract test, so the two
cannot drift.
"""

from __future__ import annotations

from importlib import resources

# Directive markers printed as the final line of `__complete` output. The shell
# wrapper pops this line and decides whether to offer the listed words or fall
# back to filesystem completion.
DIRECTIVE_LIST = ":list"
DIRECTIVE_FILES = ":files"

GLOBAL_OPTIONS: tuple[str, ...] = ("-h", "--help", "--version")

# Subcommand -> its option flags. Positional/value arguments are handled by the
# value-specific branches in `complete`, not listed here.
COMMAND_OPTIONS: dict[str, tuple[str, ...]] = {
    "build": ("-h", "--help", "-f", "--config", "-o", "--override", "--dry-run"),
    "run": ("-h", "--help", "-f", "--config", "-o", "--override", "--dry-run", "-m", "--mount", "--no-build"),
    "stop": ("-h", "--help", "-f", "--config", "-o", "--override", "--dry-run"),
    "exec": ("-h", "--help", "-f", "--config", "-o", "--override", "--dry-run"),
    "validate": ("-h", "--help", "-f", "--config", "-o", "--override", "--print-resolved"),
    "doctor": ("-h", "--help", "-f", "--config", "-o", "--override"),
    "init": ("-h", "--help", "--profile", "--ros-distro", "--devcontainer", "--overwrite"),
    "completion": ("-h", "--help"),
}

# Options that take a file path. When one of these is the preceding word the
# wrapper is told to use filesystem completion instead of a word list.
FILE_VALUE_OPTIONS: frozenset[str] = frozenset({"-f", "--config", "-m", "--mount"})

# ROS 2 distributions offered for `init --ros-distro`. Mirrors the documented set
# in `cli.py` init help.
ROS_DISTROS: tuple[str, ...] = ("lyrical", "jazzy", "humble", "iron", "rolling")


def available_profiles() -> list[str]:
    """Return packaged profile names (without the ``.json`` suffix), sorted."""
    try:
        profiles_dir = resources.files("ros2docker").joinpath("resources").joinpath("profiles")
        names = [entry.name[: -len(".json")] for entry in profiles_dir.iterdir() if entry.name.endswith(".json")]
    except (FileNotFoundError, OSError):
        return []
    return sorted(names)


def _active_subcommand(preceding: list[str]) -> str | None:
    """Return the first token in `preceding` that names a known subcommand."""
    for word in preceding:
        if word in COMMAND_OPTIONS:
            return word
    return None


def complete(words: list[str]) -> tuple[list[str], str]:
    """Return ``(candidates, directive)`` for completing the final token in `words`.

    `words` is the command line after the program name, where the last element is
    the (possibly empty) token currently being completed. Candidates are returned
    unfiltered; the shell wrapper filters them by the typed prefix.
    """
    preceding = words[:-1]
    subcommand = _active_subcommand(preceding)

    if subcommand is None:
        # Completing the subcommand itself, or a global option.
        return [*sorted(COMMAND_OPTIONS), *GLOBAL_OPTIONS], DIRECTIVE_LIST

    previous = preceding[-1] if preceding else ""

    if previous in FILE_VALUE_OPTIONS:
        return [], DIRECTIVE_FILES
    if subcommand == "init" and previous == "--profile":
        return available_profiles(), DIRECTIVE_LIST
    if subcommand == "init" and previous == "--ros-distro":
        return list(ROS_DISTROS), DIRECTIVE_LIST
    if subcommand == "completion" and previous == "completion":
        return ["bash", "zsh"], DIRECTIVE_LIST

    return list(COMMAND_OPTIONS[subcommand]), DIRECTIVE_LIST


def complete_lines(words: list[str]) -> list[str]:
    """Render `complete` output as the lines printed by ``__complete``."""
    candidates, directive = complete(words)
    return [*candidates, directive]


def _bash_function() -> str:
    return r"""_ros2docker_completion() {
    local cur words response directive
    cur="${COMP_WORDS[COMP_CWORD]}"
    words=("${COMP_WORDS[@]:1:COMP_CWORD}")

    local IFS=$'\n'
    response=($(ros2docker __complete -- "${words[@]}" 2>/dev/null))
    directive="${response[${#response[@]}-1]}"
    unset 'response[${#response[@]}-1]'

    if [[ "$directive" == ":files" ]]; then
        compopt -o default 2>/dev/null
        COMPREPLY=()
        return 0
    fi

    COMPREPLY=($(compgen -W "${response[*]}" -- "$cur"))
    return 0
}
complete -F _ros2docker_completion ros2docker
"""


def completion_script(shell: str) -> str:
    """Return the shell completion script for `shell` (``bash`` or ``zsh``)."""
    if shell == "bash":
        return _bash_function()
    if shell == "zsh":
        # zsh reuses the bash completion function via bashcompinit.
        return "autoload -U +X bashcompinit && bashcompinit\n" + _bash_function()
    raise ValueError(f"Unsupported shell: {shell!r}")

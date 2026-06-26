"""Contract: the completion spec must match the real argparse parser.

Completion candidates are rendered from a declarative spec in
`ros2docker.completion` rather than by introspecting argparse at runtime. This
test introspects the parser here (in test code, where private argparse access is
acceptable) so the spec cannot silently drift from the actual CLI surface.
"""

from __future__ import annotations

import argparse

from ros2docker import cli
from ros2docker.completion import (
    COMMAND_OPTIONS,
    FILE_VALUE_OPTIONS,
    GLOBAL_OPTIONS,
)

# Subcommands intentionally hidden from users and excluded from the spec.
HIDDEN_SUBCOMMANDS = {"__complete"}


def _subparsers_action(parser: argparse.ArgumentParser) -> argparse._SubParsersAction:
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            return action
    raise AssertionError("CLI parser has no subparsers")


def _option_strings(parser: argparse.ArgumentParser) -> set[str]:
    options: set[str] = set()
    for action in parser._actions:
        options.update(action.option_strings)
    return options


def test_command_options_match_parser() -> None:
    subparsers = _subparsers_action(cli._make_parser())
    visible = {name for name in subparsers.choices if name not in HIDDEN_SUBCOMMANDS}

    assert set(COMMAND_OPTIONS) == visible, "completion COMMAND_OPTIONS is out of sync with CLI subcommands"

    for name, expected in COMMAND_OPTIONS.items():
        actual = _option_strings(subparsers.choices[name])
        assert set(expected) == actual, f"option flags for {name!r} drifted: {set(expected) ^ actual}"


def test_global_options_present_on_root_parser() -> None:
    options = _option_strings(cli._make_parser())

    assert set(GLOBAL_OPTIONS) <= options


def test_file_value_options_are_real_options() -> None:
    subparsers = _subparsers_action(cli._make_parser())
    all_options: set[str] = set()
    for name in COMMAND_OPTIONS:
        all_options |= _option_strings(subparsers.choices[name])

    assert FILE_VALUE_OPTIONS <= all_options

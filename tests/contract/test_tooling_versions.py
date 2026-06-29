"""Contract tests that lock tooling versions across pre-commit and CI.

Pre-commit and the project's checks must run the *same* tool versions, or a
contributor's local pre-commit can pass/format differently than CI ("passed
locally, failed CI"). These tests make that invariant enforced rather than
merely commented:

- the ruff pre-commit ``rev`` equals the ``ruff==`` pin in ``pyproject.toml``;
- the actionlint hook ``rev`` equals ``ACTIONLINT_VERSION`` in the merge gate.
"""

from __future__ import annotations

import re
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
PRE_COMMIT_PATH = PACKAGE_ROOT / ".pre-commit-config.yaml"
PYPROJECT_PATH = PACKAGE_ROOT / "pyproject.toml"
MERGE_GATE_PATH = PACKAGE_ROOT / ".github" / "workflows" / "pr-merge-gate.yml"


def _hook_rev(pre_commit: str, repo_url: str) -> str:
    """Return the ``rev:`` pinned for the pre-commit repo at ``repo_url``."""
    match = re.search(
        rf"^\s*-\s*repo:\s*{re.escape(repo_url)}\s*\n\s*rev:\s*(\S+)",
        pre_commit,
        re.MULTILINE,
    )
    assert match, f"no pre-commit repo entry found for {repo_url}"
    return match.group(1).strip()


def test_precommit_ruff_matches_pyproject_pin() -> None:
    pyproject = PYPROJECT_PATH.read_text(encoding="utf-8")
    pin = re.search(r'"ruff==([^"]+)"', pyproject)
    assert pin, "no `ruff==` pin found in pyproject.toml [dev] dependencies"
    pyproject_version = pin.group(1)

    rev = _hook_rev(
        PRE_COMMIT_PATH.read_text(encoding="utf-8"),
        "https://github.com/astral-sh/ruff-pre-commit",
    )
    # ruff-pre-commit tags are the ruff version prefixed with "v".
    assert rev == f"v{pyproject_version}", (
        f"ruff pre-commit rev {rev!r} does not match pyproject pin ruff=={pyproject_version}; bump one so they match"
    )


def test_precommit_actionlint_matches_ci_version() -> None:
    merge_gate = MERGE_GATE_PATH.read_text(encoding="utf-8")
    ci = re.search(r"ACTIONLINT_VERSION:\s*(\S+)", merge_gate)
    assert ci, "no ACTIONLINT_VERSION found in pr-merge-gate.yml"
    ci_version = ci.group(1).strip()

    rev = _hook_rev(
        PRE_COMMIT_PATH.read_text(encoding="utf-8"),
        "https://github.com/rhysd/actionlint",
    )
    # The actionlint hook tag is the CI version prefixed with "v".
    assert rev == f"v{ci_version}", (
        f"actionlint pre-commit rev {rev!r} does not match CI ACTIONLINT_VERSION {ci_version!r}; keep them in sync"
    )

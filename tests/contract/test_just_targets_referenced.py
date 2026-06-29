"""Contract test: every ``just`` recipe referenced in CI/docs exists.

Workflows and docs invoke recipes such as ``just test-nondocker-cov`` and
``just docs``. Renaming or removing a recipe would break CI or the documented
workflow with no local signal until a workflow fails. This test parses the
recipe names from the ``justfile`` and asserts that every ``just <name>``
referenced in ``.github/workflows/*.yml`` and in Markdown docs exists.

To avoid false positives from English prose ("…not just that a branch was
pushed"), references in Markdown are only collected from code contexts (fenced
code blocks and inline code spans), where real command invocations live.
"""

from __future__ import annotations

import re
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
JUSTFILE_PATH = PACKAGE_ROOT / "justfile"
WORKFLOWS_DIR = PACKAGE_ROOT / ".github" / "workflows"
DOCS_DIR = PACKAGE_ROOT / "docs"

# A recipe header starts a line with the recipe name, optional parameters, then
# a colon — but not ``:=`` (a variable/setting assignment).
RECIPE_RE = re.compile(r"^(?P<name>[A-Za-z_][A-Za-z0-9_-]*)[^:\n]*:(?!=)", re.MULTILINE)
# A ``just`` invocation: the command, then a recipe name on the SAME line
# (``[ \t]+``, not ``\s+``, so a step named "Install just" followed by a YAML
# ``run:`` key does not match across the newline). Flags like ``--list`` start
# with ``-`` and are not recipes, so they are excluded.
INVOCATION_RE = re.compile(r"\bjust[ \t]+(?P<name>[A-Za-z_][A-Za-z0-9_-]*)")
FENCED_CODE_RE = re.compile(r"```[^\n]*\n(.*?)```", re.DOTALL)
INLINE_CODE_RE = re.compile(r"`([^`\n]+)`")


def _recipe_names() -> set[str]:
    return set(RECIPE_RE.findall(JUSTFILE_PATH.read_text(encoding="utf-8")))


def _invocations_in_text(text: str) -> set[str]:
    return set(INVOCATION_RE.findall(text))


def _invocations_in_markdown(text: str) -> set[str]:
    code = FENCED_CODE_RE.findall(text) + INLINE_CODE_RE.findall(text)
    names: set[str] = set()
    for segment in code:
        names |= _invocations_in_text(segment)
    return names


def _markdown_files() -> list[Path]:
    return sorted(PACKAGE_ROOT.glob("*.md")) + sorted(DOCS_DIR.rglob("*.md"))


def test_recipe_names_are_parsed() -> None:
    # Guard: if parsing yields nothing, the references check would be vacuous.
    recipes = _recipe_names()
    assert {"check", "lint", "docs", "test-contract"} <= recipes, recipes


def test_every_referenced_just_recipe_exists() -> None:
    recipes = _recipe_names()
    failures: list[str] = []

    for workflow in sorted(WORKFLOWS_DIR.glob("*.yml")):
        for name in sorted(_invocations_in_text(workflow.read_text(encoding="utf-8"))):
            if name not in recipes:
                failures.append(f"{workflow.relative_to(PACKAGE_ROOT)}: just {name}")

    for markdown in _markdown_files():
        for name in sorted(_invocations_in_markdown(markdown.read_text(encoding="utf-8"))):
            if name not in recipes:
                failures.append(f"{markdown.relative_to(PACKAGE_ROOT)}: just {name}")

    assert not failures, (
        "These `just <recipe>` references point at recipes that do not exist in "
        "the justfile (rename the reference or restore the recipe):\n" + "\n".join(failures)
    )

"""Contract test: ``src`` has no unreferenced module-level private helpers.

Hardens the soft "no dead code" audit into a deterministic gate (issue #148).
A top-level ``def _name`` / ``async def _name`` in ``src/ros2docker`` that is
referenced nowhere in first-party code (``src`` + ``tests``) is dead — the exact
drift that let ``config._validate_public_config_keys`` sit unused until a manual
audit removed it (#141, #145). The quality model's own rule applies: if a hard
check can express a rule, it should.

The check is deliberately narrow (per #148): a static, name-based reference scan
over first-party Python only. It excludes a function's own body, so a
self-recursive-but-otherwise-unused helper is still flagged. It does not resolve
dynamic (string / ``getattr``) references, nor same-named helpers in different
modules (a live ``_run`` in one module counts as a reference for a dead ``_run``
in another) — untangling those would need a heavyweight dead-code framework,
which this check intentionally is not.
"""

from __future__ import annotations

import ast
from collections.abc import Iterator
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = PACKAGE_ROOT / "src" / "ros2docker"
FIRST_PARTY_ROOTS = (SRC_ROOT, PACKAGE_ROOT / "tests")


def _python_files(*roots: Path) -> list[Path]:
    files: list[Path] = []
    for root in roots:
        files.extend(p for p in root.rglob("*.py") if "__pycache__" not in p.parts)
    return sorted(files)


def _is_private(name: str) -> bool:
    # Private = leading underscore(s) but not a dunder such as ``__main__``.
    return name.startswith("_") and not (name.startswith("__") and name.endswith("__"))


def _module_level_private_functions(path: Path) -> Iterator[tuple[str, int, int]]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef) and _is_private(node.name):
            assert node.end_lineno is not None
            yield node.name, node.lineno, node.end_lineno


def _identifier_references(path: Path) -> list[tuple[str, int]]:
    """Return every ``(identifier, lineno)`` used as a name or attribute in ``path``."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    references: list[tuple[str, int]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            references.append((node.id, node.lineno))
        elif isinstance(node, ast.Attribute):
            references.append((node.attr, node.lineno))
    return references


def test_no_unreferenced_module_level_private_functions() -> None:
    references = {path: _identifier_references(path) for path in _python_files(*FIRST_PARTY_ROOTS)}

    dead: list[str] = []
    for path in _python_files(SRC_ROOT):
        for name, start, end in _module_level_private_functions(path):
            referenced = any(
                identifier == name and not (ref_path == path and start <= lineno <= end)
                for ref_path, occurrences in references.items()
                for identifier, lineno in occurrences
            )
            if not referenced:
                dead.append(f"{path.relative_to(PACKAGE_ROOT)}:{start} {name}")

    assert not dead, "Unreferenced module-level private functions (dead code):\n" + "\n".join(dead)

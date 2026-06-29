"""Contract test: every doc under docs/ is reachable from README.

``test_markdown_links.py`` already checks that internal links *resolve*. This
test checks the stronger property that every documentation file is *discoverable*
by following internal Markdown links starting at ``README.md`` — so a doc cannot
silently become orphaned. This turns the stated goal ("all documents traceable
from the base README") into a machine-checked invariant. Relates to #47.
"""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote, urlparse

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
README = (PACKAGE_ROOT / "README.md").resolve()
LINK_RE = re.compile(r"(?<!!)\[[^\]]+\]\((?P<target>[^)]+)\)")

# Docs intentionally left out of the README link graph. Each glob (matched
# against the repo-root-relative path) has a documented reason:
#   - docs/release-notes/*: per-release notes surfaced through GitHub Releases,
#     not the README documentation tree.
#   - *TEMPLATE*: copy-me templates (e.g. release-notes/TEMPLATE.md), not
#     standalone documentation.
ALLOWLIST_GLOBS = (
    "docs/release-notes/*",
    "*TEMPLATE*",
)


def _internal_md_links(markdown: Path) -> list[Path]:
    """Resolve the internal ``.md`` link targets in a Markdown file."""
    targets: list[Path] = []
    for match in LINK_RE.finditer(markdown.read_text(encoding="utf-8")):
        raw = match.group("target").strip()
        if not raw or raw.startswith("#"):
            continue
        parsed = urlparse(raw)
        if parsed.scheme or parsed.netloc:
            continue
        target = unquote(raw.split("#", 1)[0])
        if not target.endswith(".md"):
            continue
        resolved = (markdown.parent / target).resolve()
        if resolved.exists():
            targets.append(resolved)
    return targets


def _reachable_from_readme() -> set[Path]:
    """Transitive closure of internal ``.md`` links starting at README.md."""
    seen = {README}
    stack = [README]
    while stack:
        for nxt in _internal_md_links(stack.pop()):
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    return seen


def _is_allowlisted(rel_path: Path) -> bool:
    return any(rel_path.match(glob) for glob in ALLOWLIST_GLOBS)


def test_every_doc_is_reachable_from_readme() -> None:
    reachable = _reachable_from_readme()

    orphans = [
        str(doc.relative_to(PACKAGE_ROOT))
        for doc in sorted((PACKAGE_ROOT / "docs").rglob("*.md"))
        if not _is_allowlisted(doc.relative_to(PACKAGE_ROOT)) and doc.resolve() not in reachable
    ]

    assert not orphans, (
        "These docs are not reachable by following internal .md links from "
        "README.md. Link them from README/docs, or add them to ALLOWLIST_GLOBS "
        "with a reason:\n" + "\n".join(orphans)
    )

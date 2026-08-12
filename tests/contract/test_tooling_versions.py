"""Contract tests that lock tooling versions across pre-commit and CI.

Pre-commit and the project's checks must run the *same* tool versions, or a
contributor's local pre-commit can pass/format differently than CI ("passed
locally, failed CI"). These tests make that invariant enforced rather than
merely commented:

- the ruff pre-commit ``rev`` equals the ``ruff==`` pin in ``pyproject.toml``;
- the actionlint hook ``rev`` equals ``ACTIONLINT_VERSION`` in the merge gate;
- every Docker-based hook names its image by digest.

The third one is a different failure from the first two. ``rev:`` pins the
*hook definition*; a ``language: docker_image`` hook then runs whatever its
entry names, and upstream's ``hadolint-docker`` entry carried no tag at all —
so ``:latest``. On 2026-08-07 hadolint shipped ``DL3064`` and every open PR
went red with no commit in this repository, which blocked the v0.1.4 release
because the release workflow gates publishing on the release-notes PR.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

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


#: pre-commit hooks whose language is ``docker_image``. Their id is not enough
#: to tell (``shellcheck`` does not say "docker"), and the language lives
#: upstream, so the repository URLs are named here. A new Docker-based hook
#: must be added, which is the moment to notice it needs a digest.
DOCKER_IMAGE_HOOK_REPOS = {
    "https://github.com/rhysd/actionlint": {"actionlint-docker"},
    "https://github.com/hadolint/hadolint": {"hadolint-docker"},
    "https://github.com/koalaman/shellcheck-precommit": {"shellcheck"},
}

IMAGE_WITH_DIGEST = re.compile(r"^\S+@sha256:[0-9a-f]{64}(?:\s|$)")


def _configured_hooks() -> dict[tuple[str, str], dict]:
    config = yaml.safe_load(PRE_COMMIT_PATH.read_text(encoding="utf-8"))
    return {(repo["repo"], hook["id"]): hook for repo in config["repos"] for hook in repo.get("hooks", [])}


def test_docker_based_hooks_pin_their_image_by_digest() -> None:
    """A hook that runs a linter as an image must say which image, exactly.

    Without an ``entry`` override the image reference is upstream's, and this
    repository cannot see it — which is how a ``:latest`` sat behind a pinned
    ``rev:`` with a comment saying the rev *was* the version.
    """
    hooks = _configured_hooks()
    problems: list[str] = []

    for repo_url, hook_ids in DOCKER_IMAGE_HOOK_REPOS.items():
        for hook_id in hook_ids:
            hook = hooks.get((repo_url, hook_id))
            if hook is None:
                problems.append(f"{hook_id}: no longer configured from {repo_url}; update this test")
                continue
            entry = str(hook.get("entry", "")).strip()
            if not entry:
                problems.append(
                    f"{hook_id}: no `entry` override, so the image it runs is whatever "
                    f"{repo_url}@{_hook_rev(PRE_COMMIT_PATH.read_text(encoding='utf-8'), repo_url)} "
                    "names — which for hadolint was an untagged, i.e. `:latest`, image"
                )
            elif not IMAGE_WITH_DIGEST.match(entry):
                problems.append(
                    f"{hook_id}: entry {entry!r} does not pin the image by digest. "
                    "Use `name:tag@sha256:...` so the tag stays readable and the "
                    "digest is what runs."
                )

    assert not problems, "Docker-based pre-commit hooks that are not reproducible:\n" + "\n".join(problems)


def test_docker_hook_tags_agree_with_their_rev() -> None:
    """The readable half of the pin must not drift from the hook repo's rev.

    A digest alone is reproducible and unreadable; a tag beside it is only
    useful while it says the same thing as the ``rev:`` above it.
    """
    pre_commit = PRE_COMMIT_PATH.read_text(encoding="utf-8")
    hooks = _configured_hooks()
    problems: list[str] = []

    for repo_url, hook_ids in DOCKER_IMAGE_HOOK_REPOS.items():
        rev = _hook_rev(pre_commit, repo_url).lstrip("v")
        for hook_id in hook_ids:
            entry = str(hooks.get((repo_url, hook_id), {}).get("entry", ""))
            reference = entry.split()[0] if entry else ""
            tag = reference.split("@", 1)[0].rsplit(":", 1)[-1].lstrip("v")
            if tag != rev:
                problems.append(f"{hook_id}: image tag {tag!r} but hook rev {rev!r} ({repo_url})")

    assert not problems, "Docker hook image tags that disagree with their rev:\n" + "\n".join(problems)

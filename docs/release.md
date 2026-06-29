# Release

Releases are built by `.github/workflows/release.yml`.

## Owner runbook: cutting a release

Cutting a release is mostly autonomous. The human owner performs only the three
gated steps — Code Owner approvals, the `vX.Y.Z` tag, and the `pypi` deployment
approval (the three gates in [agentic-workflow.md](agentic-workflow.md)). Full
hands-off publish is intentionally impossible; "as autonomous as possible" means
autonomous through the release PR, with the owner approving the irreversible
publish. Follow these steps in order. Replace `vX.Y.Z` with the target version.

### 1. Quality pass — clear drift first (agent)

Skip for a trivial patch release. Otherwise prompt the agent:

> Run the repository quality workflow autonomously for a release: create and
> complete focused PRs for test-ci-audit, then documentation-audit, then
> implementation-cleanup — each started from a fresh `origin/main`, opened ready
> with auto-merge. Report each PR URL and CI status.

Depth comes from running these as separate, focused PRs, not one pass (see
[quality-model.md](quality-model.md)). **Your part:** approve the Code Owner
review on any PR that touches an owned path (`.github/`, `tests/contract/`,
`pyproject.toml`, `docs/release.md`, …) — the bot cannot self-approve those.

### 2. Prepare the release PR (agent)

Prompt the agent:

> Prepare release vX.Y.Z: open an issue from release-process.md and a PR that
> reviews the diff since the previous tag, writes `docs/release-notes/vX.Y.Z.md`
> from the template, updates the Dockerfile base image digest and the pinned
> `ZENOH_VERSION` / `ZENOH_ROS2DDS_VERSION` / `MCAP_CLI_VERSION`, and records the
> validation checks in the PR body. Open it ready with auto-merge.

**Your part:** review and approve the release PR (it touches `docs/release.md`,
an owned path, so it needs your Code Owner approval). It merges once green.

### 3. Tag the release (owner — manual)

`vX.Y.Z` tags are restricted to the owner, so push the tag yourself after the
release PR merges:

```bash
git switch main && git fetch --prune && git pull --ff-only
git tag vX.Y.Z
git push origin vX.Y.Z
```

### 4. Approve the publish (owner — manual)

The tag push starts `release.yml`, which validates and then waits on the `pypi`
deployment environment. Approve the `pypi` deployment in the Actions run. The
workflow then publishes to PyPI through Trusted Publishing and creates the GitHub
Release from `docs/release-notes/vX.Y.Z.md`.

### 5. Verify

Confirm the new version on PyPI and the GitHub Release page. For a rehearsal,
publish to TestPyPI first via the manual `release` workflow dispatch (see
[Publishing](#publishing)).

## Pre-Release Soft-Check

Before cutting a release, run the repository quality soft-check pass. Hard checks
(the contract tests and `ci-success`) catch mechanically-verifiable problems, but
issue-driven, organic growth also produces drift that only a review pass catches:
naming and framing that no longer fit, stale or deprecated docs, and interfaces
that eroded through least-invasive patches.

Run the multi-PR quality workflow
([../.github/ISSUE_TEMPLATE/quality-workflow.md](../.github/ISSUE_TEMPLATE/quality-workflow.md)),
which sequences `test-ci-audit`, `documentation-audit`, and
`implementation-cleanup`. This is a recommended checklist step, **not** a CI
gate — there is intentionally no automation that fails a release based on how
recently an audit ran.

## Version Source

Package versions come from Git tags through `setuptools-scm`. The project does
not keep a static version in `pyproject.toml` or `src/ros2docker/__init__.py`.

## Tags

Stable releases use tags in this form:

```text
vX.Y.Z
```

Push the tag from the release commit after the release PR has merged.

## Release Notes

Each stable release must include a release notes file matching the tag:

```text
docs/release-notes/vX.Y.Z.md
```

Start from [docs/release-notes/TEMPLATE.md](release-notes/TEMPLATE.md). The
release notes should summarize the diff from the previous release tag to the
new tag, including user-facing CLI, config, API, Docker, dependency, packaging,
compatibility, and migration changes.

A required `verify-release-notes` job fails the release *before* any publish
step if the tag-specific release notes file is missing, so a tag push with no
notes never reaches PyPI (publishing is irreversible). When present, that same
file becomes the GitHub Release description.

## Validation

The release workflow runs non-Docker checks on Python 3.10 and 3.12, validates
package artifacts, verifies tag-specific release notes, and runs fast Docker E2E
before publishing:

```bash
just lint
just typecheck
just test-unit
just test-contract
just docs
just package
just test-e2e-fast
```

## Publishing

Tag pushes publish to PyPI through Trusted Publishing and create a GitHub
Release with the release notes, wheel, source distribution, and `SHA256SUMS`.

Manual workflow dispatch publishes only to TestPyPI. Use it for release
rehearsals before cutting a PyPI tag.

Trusted Publishers must be configured for repository `develNor/ros2docker`,
workflow `.github/workflows/release.yml`, and environments `pypi` and
`testpypi`.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the canonical maintainer release
workflow.

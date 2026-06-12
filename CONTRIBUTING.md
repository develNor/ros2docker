# Contributing

This is the canonical development workflow for contributors.

## Development Setup

Prerequisites:

- Python 3.10 or 3.12 when mirroring CI.
- Docker for runtime and E2E checks.
- `just` for local workflow commands.
- `pipx` for user-style installs.
- GitHub CLI `gh` for PR and auto-merge workflows.

Set up the repository from the root:

```bash
just setup
```

This creates `.venv`, installs `.[dev]`, and installs pre-commit hooks.

## Local Checks

Run individual checks from the repository root:

```bash
just lint
just typecheck
just test-unit
just test-contract
just test-nondocker-cov
just docs
just package
```

For small review PRs, run lightweight checks before opening the draft PR:

```bash
just lint
just typecheck
just test-unit
```

Before opening or updating a ready PR, run:

```bash
just check
```

`just check` runs linting, type checking, unit and contract tests under
coverage, docs checks, and package validation. It does not run Docker E2E
checks.

Use Docker E2E checks when Docker/runtime behavior changed:

```bash
just test-e2e-fast
just test-e2e-slow
```

`just test-e2e-fast` runs the fast Docker smoke fixtures. `just test-e2e-slow`
runs the full Docker E2E suite, including slow image and ROS launch checks.

## Branch And Merge Policy

`main` is protected. Do not push directly to `main`.

All changes must go through a pull request. Local `main` should be treated as a
clean mirror of `origin/main`.

Start each change from an up-to-date `origin/main`:

```bash
git fetch --prune
git switch -c <type>/<short-topic> origin/main
```

Keep the change small and coherent. Update tests for changed behavior. Update
README, docs, examples, or schema files when CLI, config, API, Docker behavior,
or public behavior changes.

After the PR has merged, clean up from a synchronized `main`:

```bash
git switch main
git fetch --prune
git pull --ff-only
git branch -d <branch>
```

If local `main` diverges from `origin/main`, do not continue working on `main`.
Move any useful local work to a topic branch first, then restore `main` to match
`origin/main`.

## Pull Request Modes

This repository uses two pull request modes.

### Review PRs

Review PRs are the default. They are intended for small changes, documentation
updates, and changes that should be inspected before merge.

Review PRs should usually be opened as draft pull requests. Draft PRs provide
lightweight CI feedback but are not expected to be mergeable yet.

Use this flow:

```bash
gh pr create --draft --base main --title "<title>" --body "<summary>"
```

After review approval, mark the PR ready:

```bash
gh pr ready <number>
```

### Autonomous PRs

Autonomous PRs are used when the user explicitly asks for autonomous iteration,
auto-merge, or completion without further review.

Autonomous PRs are opened as ready-for-review pull requests, may have auto-merge
enabled, and must pass the full required merge gate before entering `main`.

Use this flow:

```bash
gh pr create --base main --title "<title>" --body "<summary>"
gh pr merge <number> --auto --squash --delete-branch
gh pr checks <number> --watch --required
```

If CI fails or auto-merge is blocked, inspect the failure, fix the same PR
branch, push the update, and repeat until the PR merges or is clearly blocked.

## CI Policy

The required merge gate for `main` is:

```text
ci-success
```

Draft PRs run `pr-lightweight` for quick feedback. The lightweight workflow runs:

```bash
just lint
just typecheck
just test-unit
```

Ready PRs and merge queue entries run `pr-merge-gate`. The `ci-success` job in
that workflow requires:

```bash
just check
just test-e2e-fast
```

The ready-PR non-Docker gate runs unit and contract tests through
`just test-nondocker-cov`, enforcing the configured coverage threshold and
uploading `coverage.xml` as a workflow artifact. Docker E2E remains behavioral
validation only.

A PR cannot merge into `main` unless `ci-success` passes. Prefer requiring only
this stable aggregate check in branch protection so individual job names can
change without rewriting repository policy.

Full Docker E2E runs in the scheduled `nightly-e2e` workflow and may also be
triggered manually. Nightly full E2E is not a replacement for the required PR
merge gate.

Avoid path filters or branch filters for required workflows. If a required
workflow is skipped by filtering, GitHub can leave the check pending and block
merging.

See [docs/ci.md](docs/ci.md) for the contributor-facing CI workflow summary.

## Release Workflow

Releases are built from version tags. After the release PR has merged, create
and push a tag in the form `vX.Y.Z` from the release commit:

Release PRs must include `docs/release-notes/vX.Y.Z.md`, copied from
`docs/release-notes/TEMPLATE.md` and filled with a summary of user-facing
changes since the previous release tag. Include compatibility, migration,
Docker/dependency, packaging, and validation notes. If there are no breaking
changes or migration steps, say that explicitly.

```bash
git switch main
git fetch --prune
git pull --ff-only
git tag vX.Y.Z
git push origin vX.Y.Z
```

The `release` workflow validates the tag with non-Docker checks, package
artifact validation, tag-specific release notes, and fast Docker E2E before
publishing. Successful tag releases publish the wheel and sdist to PyPI through
Trusted Publishing, then create a GitHub Release with the release notes, wheel,
sdist, and `SHA256SUMS`.

Maintainers must configure pending Trusted Publishers on PyPI and TestPyPI for
repository `develNor/ros2docker`, workflow `.github/workflows/release.yml`, and
environments `pypi` and `testpypi`. No PyPI API token should be stored in GitHub
secrets for the normal release path.

Use the manual `release` workflow dispatch for TestPyPI rehearsals. Manual runs
publish only to TestPyPI and do not create GitHub Releases.

See [docs/release.md](docs/release.md) for the focused release workflow summary.

## Pull Request Description

Include local command results in the PR summary. Call out whether the PR is a
review PR or an autonomous PR, which checks were run, and whether Docker/runtime
behavior changed. Release PRs should also include a release notes draft or point
to the completed `docs/release-notes/vX.Y.Z.md` file.

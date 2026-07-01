# Contributing

This is the canonical development workflow for contributors.

## Development Setup

Prerequisites:

- Python 3.10 through 3.14 when mirroring CI.
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

Actionable work should be tracked in GitHub Issues. Use the templates in
[.github/ISSUE_TEMPLATE/](.github/ISSUE_TEMPLATE/) for reusable task recipes,
link PRs to issues with `Fixes #<number>`, `Closes #<number>`, or
`Refs #<number>`, and keep temporary PR checklists in the PR body. See
[docs/work-items.md](docs/work-items.md) for the work-item policy.

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

The required merge gate for `main` is the single aggregate status check
`ci-success`; a PR cannot merge into `main` until it is green.

- Draft (review) PRs run `pr-lightweight` for quick feedback: `just lint`,
  `just typecheck`, `just test-unit`.
- Ready PRs and merge-queue entries run `pr-merge-gate`, whose `ci-success` job
  runs the full `just check` plus `just test-e2e-fast`.

Avoid path or branch filters on required workflows: a filtered-out required check
can stay pending and block merging.

See [docs/ci.md](docs/ci.md) for the workflow contract behind `ci-success` — the
Python 3.10–3.14 non-Docker matrix, the two-gate coverage model, and the
dependency, build, and workflow lint jobs. Repository settings that live outside
git are recorded in
[docs/github-repository-settings.md](docs/github-repository-settings.md).

## Release Workflow

Before cutting a non-trivial release, run the repository quality soft-check pass —
the multi-PR
[quality-workflow](.github/ISSUE_TEMPLATE/quality-workflow.md) — to clear organic
drift that hard checks miss. It is a recommended checklist step, not a CI gate.

Releases are cut by the owner from `vX.Y.Z` tags after the release PR merges, and
publishing to PyPI runs through Trusted Publishing behind a human-approved
deployment. The full runbook — tag protection, the required
`docs/release-notes/vX.Y.Z.md`, validation checks, and Trusted Publishing /
TestPyPI setup — lives in [docs/release.md](docs/release.md).

## Pull Request Description

Include local command results in the PR summary. Call out whether the PR is a
review PR or an autonomous PR, which checks were run, and whether Docker/runtime
behavior changed. Release PRs should also include a release notes draft or point
to the completed `docs/release-notes/vX.Y.Z.md` file.

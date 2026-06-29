# CI

This repository keeps contributor checks aligned with `justfile` targets so the
same commands can run locally and in GitHub Actions.

## Pull Requests

Draft review PRs run `.github/workflows/pr-lightweight.yml` for quick feedback:

```bash
just lint
just typecheck
just test-unit
```

Ready PRs, merge queue entries, and pushes to `main` run
`.github/workflows/pr-merge-gate.yml`. The required aggregate branch protection
check is:

```text
ci-success
```

`ci-success` depends on Python 3.10 through 3.14 non-Docker checks, package
validation, and fast Docker E2E:

```bash
just check
just test-e2e-fast
```

The ready-PR non-Docker checks run unit and contract tests through
`just test-nondocker-cov`, enforcing the configured coverage threshold and
uploading `coverage.xml` as an artifact. The Python 3.12 leg also publishes
that coverage report to Codecov with `CODECOV_TOKEN` for the README coverage
badge and Codecov PR coverage statuses. Docker E2E tests are not collected for
coverage.

Coverage uses two gates with different roles:

- The **hard gate** is `tool.coverage.report.fail_under` in `pyproject.toml`
  (currently `80`), enforced by `just test-nondocker-cov` inside `ci-success`.
  It is the only coverage number that can block a merge and is kept a few points
  below actual coverage to catch real regressions without forcing churn.
- The **advisory Codecov statuses** are configured in
  [`codecov.yml`](../codecov.yml) and are **not** required checks:
  - `codecov/patch` targets `90%` of changed lines — meaningful coverage of new
    code without "ghost hunting" a single uncovered defensive line.
  - `codecov/project` is a looser whole-project backstop: target `auto` against
    the PR base, threshold `1%`, to catch gradual overall erosion.

Raise `fail_under` only when actual coverage rises through real behavior tests;
never weaken, skip, or delete tests to hit a number.

Use `ci-success` as the required check instead of individual job names.

### Dependency Review

`pr-merge-gate.yml` includes a `dependency-review` job that runs on pull
requests and merge queue entries. It uses `actions/dependency-review-action` to
**block** any PR that introduces a dependency with a known **high or above**
severity vulnerability, across both `runtime` and `development` scopes. The job
is part of the required `ci-success` aggregate; it is tolerated as skipped on
direct pushes to `main`, where there is no PR diff to review.

This complements rather than duplicates the other security tooling:

- **Dependabot** alerts on dependencies that are already merged.
- **Dependency Review** blocks the PR that is introducing one.
- The advisory nightly **Trivy image scan** scans the built image's OS packages,
  not the declared Python dependencies in `pyproject.toml`.

### Workflow Lint

`pr-merge-gate.yml` includes a `workflow-lint` job that runs
[actionlint](https://github.com/rhysd/actionlint) (pinned by version and SHA256)
against the workflow files. `ubuntu-latest` ships `shellcheck`, which actionlint
invokes on `run:` blocks. The job is part of the required `ci-success` aggregate,
so a workflow that becomes invalid — wrong contexts, bad expressions or matrix
usage, shell issues — blocks the merge instead of silently changing what CI
means.

Run the same check locally:

```bash
just lint-workflows
```

This uses the pinned `actionlint-docker` pre-commit hook (the actionlint image
bundles `shellcheck`), so it matches the CI job without a local Go or binary
install. Keep the pre-commit `rev` and the CI `ACTIONLINT_VERSION` in sync.

The workflow-lint job is intentionally the only workflow-security automation
added here. GitHub Actions **security** scanning is already covered by CodeQL
default setup (languages `actions` + `python`, threat model `remote`), which is a
required check via the `Protect main` ruleset. The owner-vs-contributor boundary
for CI changes is enforced by branch protection plus `CODEOWNERS`, not by a
separate policy job — see
[docs/github-repository-settings.md](github-repository-settings.md).

## Docker E2E

`just test-e2e-fast` runs the Docker smoke fixtures used by the merge gate.
Run it locally when Dockerfiles, Docker command rendering, runtime mounts,
entrypoints, or ROS workspace behavior change.

`just test-e2e-slow` runs the complete Docker E2E suite. It is intended for
slow image and ROS launch coverage.

GUI (X11) forwarding and SSH-agent forwarding are validated end to end in the
nightly slow E2E suite, not just as command contracts: the nightly job installs
`xvfb` and `openssh-client`, then the slow tests start a real `Xvfb` display and
an ephemeral `ssh-agent` with a throwaway key and assert that `xdpyinfo` and
`ssh-add -l` succeed *inside* a forwarding container. Locally these two tests
self-skip when `Xvfb` / `openssh-client` are not installed, so a manual check is
only needed when changing forwarding behavior on a host without those tools.

## Nightly

`.github/workflows/nightly-e2e.yml` runs the slow E2E suite on a schedule and
by manual dispatch. Nightly E2E does not replace the required PR merge gate.

`.github/workflows/image-scan.yml` runs an advisory container image scan on a
nightly schedule and by manual dispatch. It builds the default ros2docker image
through the CLI, scans HIGH and CRITICAL fixed vulnerabilities with Trivy, and
uploads the report as an artifact. Vulnerability findings are advisory and do
not fail the workflow or participate in the required PR merge gate.

Dependabot is configured in `.github/dependabot.yml` for weekly grouped GitHub
Actions and Python dependency updates.

## Local Checks

Use [CONTRIBUTING.md](../CONTRIBUTING.md) as the canonical contributor workflow.

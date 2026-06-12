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

Codecov is configured in [`codecov.yml`](../codecov.yml) to report two PR
coverage statuses from the uploaded `nondocker` report:

- `codecov/patch` is strict for changed lines: target `100%`, threshold `0%`.
- `codecov/project` is a looser whole-project backstop: target `auto` against
  the PR base, threshold `1%`.

Patch coverage is the primary signal for new or changed code because it only
measures lines adjusted in the PR. Project coverage stays looser so it catches
gradual overall coverage erosion without turning existing uncovered code into
the focus of every small change.

Use `ci-success` as the required check instead of individual job names.

## Docker E2E

`just test-e2e-fast` runs the Docker smoke fixtures used by the merge gate.
Run it locally when Dockerfiles, Docker command rendering, runtime mounts,
entrypoints, or ROS workspace behavior change.

`just test-e2e-slow` runs the complete Docker E2E suite. It is intended for
slow image and ROS launch coverage.

When changing GUI forwarding or SSH agent forwarding behavior, also do one
manual runtime check on a host with a real X11 display or SSH agent. CI validates
the command contracts, but hosted runners may not expose those host integrations
as live services.

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

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

Ready PRs and merge queue entries run `.github/workflows/pr-merge-gate.yml`.
The required aggregate branch protection check is:

```text
ci-success
```

`ci-success` depends on Python 3.10 and 3.12 lightweight checks, package
validation, and fast Docker E2E:

```bash
just check
just test-e2e-fast
```

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

## Local Checks

Use [CONTRIBUTING.md](../CONTRIBUTING.md) as the canonical contributor workflow.

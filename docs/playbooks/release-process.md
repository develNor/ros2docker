# Release Process

Goal:
Ship a validated, well-described ros2docker release.

Use when:
A maintainer is preparing a new version tag and GitHub/PyPI release.

Rules:
- Follow AGENTS.md, CONTRIBUTING.md, and DEVELOPMENT_PRINCIPLES.md.
- Start from a fresh origin/main topic branch.
- Do not tag or publish until the release PR has merged.
- Open a draft review PR by default. Use an autonomous PR only when explicitly
  requested.
- Update docs/tests if public CLI, config, API, Docker, or runtime behavior
  changed.

Tasks:
- Choose the target tag in the form vX.Y.Z.
- Identify the previous release tag.
- Review the diff from the previous release tag to HEAD, including commits,
  changed files, and merged PRs where available.
- Summarize important user-facing CLI, config, API, Docker, dependency, and
  packaging changes.
- State breaking changes and migration steps, or explicitly say there are none.
- Copy docs/release-notes/TEMPLATE.md to docs/release-notes/vX.Y.Z.md and fill
  it with the release notes summary.
- Update Dockerfile base image digest.
- Update pinned external tool versions:
  - ZENOH_VERSION
  - ZENOH_ROS2DDS_VERSION
  - MCAP_CLI_VERSION
- Update checksum verification where already specified.
- Include a release notes draft and local command results in the PR description.
- After the PR merges, tag the release commit and push the tag.
- Confirm the release workflow publishes to PyPI and creates the GitHub Release
  from docs/release-notes/vX.Y.Z.md.

Checks:
- just lint
- just typecheck
- just test-unit
- just test-contract
- just docs
- just package
- just test-e2e-fast

Report:
- previous release tag and target release tag,
- release notes file path,
- user-facing changes,
- compatibility or migration notes,
- Docker/dependency updates,
- commands run and results,
- PR URL and CI status,
- final PyPI and GitHub Release status after tagging.

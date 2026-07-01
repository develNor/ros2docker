---
name: Release process
about: Prepare, validate, describe, and publish a ros2docker release.
title: "Prepare release vX.Y.Z"
labels: ready, documentation
assignees: ""
---

## Goal

Ship a validated, well-described ros2docker release.

## Use When

A maintainer is preparing a new version tag and GitHub/PyPI release. This is the
executable checklist for step 2 of the owner release runbook in
[docs/release.md](../../docs/release.md#owner-runbook-cutting-a-release); the
runbook covers the surrounding owner-gated steps (tag and publish approval).

## Target

- Target tag: `vX.Y.Z`

## Constraints

- Follow the shared [task contract](../../docs/quality-model.md#task-contract):
  workflow, quality rules, work tracking, and test integrity.
- Start from a fresh `origin/main` topic branch.
- Do not tag or publish until the release PR has merged.
- Open a draft review PR by default.
- Use an autonomous PR only when explicitly requested.
- Update docs/tests if public CLI, config, API, Docker, or runtime behavior
  changed.

## Tasks

- [ ] Choose the target tag in the form `vX.Y.Z`.
- [ ] Identify the previous release tag.
- [ ] Review the diff from the previous release tag to HEAD, including commits,
      changed files, and merged PRs where available.
- [ ] Summarize important user-facing CLI, config, API, Docker, dependency, and
      packaging changes.
- [ ] State breaking changes and migration steps, or explicitly say there are
      none.
- [ ] Copy `docs/release-notes/TEMPLATE.md` to
      `docs/release-notes/vX.Y.Z.md` and fill it with the release notes
      summary.
- [ ] Update Dockerfile base image digest.
- [ ] Update pinned external tool versions: `ZENOH_VERSION`,
      `ZENOH_ROS2DDS_VERSION`, and `MCAP_CLI_VERSION`.
- [ ] Update checksum verification where already specified.
- [ ] Include a release notes draft and local command results in the PR
      description.
- [ ] After the PR merges, have the owner tag the release commit on `main` and
      push the protected `vX.Y.Z` tag.
- [ ] Confirm the release workflow publishes to PyPI and creates the GitHub
      Release from `docs/release-notes/vX.Y.Z.md`.

## Acceptance Criteria

- [ ] Release notes exist for the target tag.
- [ ] Compatibility, migration, Docker/dependency, packaging, and validation
      notes are explicit.
- [ ] The release workflow validates and publishes from the tag.

## Checks

- [ ] `just lint`
- [ ] `just typecheck`
- [ ] `just test-unit`
- [ ] `just test-contract`
- [ ] `just docs`
- [ ] `just package`
- [ ] `just test-e2e-fast`
- [ ] `just test-e2e-slow`

## Final Report

- Previous release tag:
- Target release tag:
- Release notes file path:
- User-facing changes:
- Compatibility or migration notes:
- Docker/dependency updates:
- Commands run:
- PR URL and CI status:
- Final PyPI and GitHub Release status after tagging:

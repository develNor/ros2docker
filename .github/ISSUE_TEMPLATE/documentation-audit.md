---
name: Documentation audit
about: Align public-facing documentation with the repository and remove drift.
title: "Audit public-facing documentation"
labels: agent-ready, documentation
assignees: ""
---

## Goal

Make public-facing documentation consistent, non-redundant, and aligned with
the repository.

## Scope

- README.
- CONTRIBUTING.
- DEVELOPMENT / development principles.
- AGENTS.md.
- docs/.
- Package metadata if it contains public-facing descriptions.

## Document Ownership

- README: user-facing purpose, installation, quick start, common usage.
- CONTRIBUTING: human contribution workflow, PR expectations, local checks.
- DEVELOPMENT: technical setup, architecture, testing, release process.
- AGENTS.md: agent-specific operating rules only; link to human docs instead of
  duplicating them.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [AGENTS.md](../../AGENTS.md).
- Prefer one canonical location plus links instead of repeated prose.
- Do not change source code unless needed to correct a documented command.
- Do not skip, weaken, or delete tests/CI to make this pass.

## Tasks

- [ ] Find duplicated, stale, contradictory, or misplaced information.
- [ ] Ensure documented workflows match actual repo configuration.
- [ ] Ensure agent instructions do not diverge from human development
      instructions.
- [ ] Keep AGENTS.md short and focused on agent behavior.

## Acceptance Criteria

- [ ] Public docs have clear ownership and no accidental policy duplication.
- [ ] README, CONTRIBUTING, DEVELOPMENT_PRINCIPLES, AGENTS.md, and docs links
      resolve.
- [ ] Any remaining duplication is intentional and recorded in the PR.

## Checks

- [ ] `just docs`
- [ ] `just lint`

## Final Report

- Final responsibility of each public-facing document:
- Moved or removed duplicated content:
- Remaining intentional duplication:
- Documented behavior that still lacks implementation or tests:
- Commands run:

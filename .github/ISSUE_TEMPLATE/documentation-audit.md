---
name: Documentation audit
about: Align public-facing documentation with the repository and remove drift.
title: "Audit public-facing documentation"
labels: ready, documentation
assignees: ""
---

## Goal

Make public-facing documentation consistent, non-redundant, and aligned with
the repository.

## Scope

- README.
- CONTRIBUTING.
- DEVELOPMENT / development principles.
- `.github/ISSUE_TEMPLATE/`.
- docs/.
- Package metadata if it contains public-facing descriptions.

## Document Ownership

Align each document with its owned concern. The canonical ownership map lives in
[docs/quality-model.md](../../docs/quality-model.md#document-ownership) — audit
against it; do not restate it here.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [docs/work-items.md](../../docs/work-items.md).
- Prefer one canonical location plus links instead of repeated prose.
- Do not change source code unless needed to correct a documented command.
- Do not skip, weaken, or delete tests/CI to make this pass.

## Tasks

- [ ] Find duplicated, stale, contradictory, or misplaced information.
- [ ] Ensure documented workflows match actual repo configuration.
- [ ] Ensure contributor workflow instructions do not diverge between docs and
      templates.
- [ ] Keep public docs focused on their document ownership.

## Acceptance Criteria

- [ ] Public docs have clear ownership and no accidental policy duplication.
- [ ] README, CONTRIBUTING, DEVELOPMENT_PRINCIPLES, issue templates, and docs
      links resolve.
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

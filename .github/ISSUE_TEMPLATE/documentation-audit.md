---
name: Documentation audit
about: Align public docs with the repo; justify or delete what no longer earns its place.
title: "Audit public-facing documentation"
labels: ready, documentation
assignees: ""
---

## Goal

Make public-facing documentation consistent, non-redundant, aligned with the
repository, and **no larger than it needs to be** — every document and section
must earn its place or be removed or merged into its canonical home.

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

- Follow the shared [task contract](../../docs/quality-model.md#task-contract):
  workflow, quality rules, work tracking, and test integrity.
- Prefer one canonical location plus links instead of repeated prose.
- You may remove or merge whole documents or sections, not only align them, when
  they do not earn their place.
- Do not change source code unless needed to correct a documented command.

## Tasks

- [ ] Find duplicated, stale, contradictory, or misplaced information.
- [ ] Justify or delete: for each document and major section, confirm it earns
      its place, or remove it or merge it into its canonical home.
- [ ] Generality: in files meant to be reusable, replace repo-specific names and
      details with generic forms, except where specifics must live (README
      badges, CODEOWNERS, packaging metadata, local instance config).
- [ ] Ensure documented workflows match actual repo configuration.
- [ ] Ensure contributor workflow instructions do not diverge between docs and
      templates.
- [ ] Keep public docs focused on their document ownership.

## Acceptance Criteria

- [ ] Public docs have clear ownership and no accidental policy duplication.
- [ ] Documents or sections that did not earn their place were removed or merged,
      and the change is recorded in the PR.
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

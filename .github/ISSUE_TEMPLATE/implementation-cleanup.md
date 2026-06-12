---
name: Implementation cleanup
about: Simplify implementation while preserving the public contract.
title: "Clean up implementation"
labels: agent-ready
assignees: ""
---

## Goal

Improve implementation quality while preserving the current public contract.

## Scope

- Source code.
- Tests only where needed to preserve or clarify behavior.
- Documentation only for small corrections caused by code cleanup.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [AGENTS.md](../../AGENTS.md).
- Do not change documented CLI/API behavior unless tests and docs prove the old
  behavior was stale or broken.
- Do not remove tests or weaken assertions.
- Do not add compatibility layers unless strictly required.
- Prefer clean removal of obsolete or dead code.
- Prefer the simplest implementation that satisfies the tests.
- Avoid broad rewrites.

## Tasks

- [ ] Check for dead code.
- [ ] Check for unused imports, functions, classes, and constants.
- [ ] Check for redundant branches and duplicated logic.
- [ ] Check for over-abstracted helpers and unclear names.
- [ ] Check for unnecessary compatibility paths.
- [ ] Check for unnecessarily complicated control flow.
- [ ] Check for tests that no longer test meaningful behavior.

## Acceptance Criteria

- [ ] Public CLI/API/config/Docker behavior is preserved unless explicitly
      documented and tested.
- [ ] Simplifications are focused and reviewable.
- [ ] Existing tests remain meaningful.

## Checks

- [ ] `just lint`
- [ ] `just typecheck`
- [ ] `just test-unit`
- [ ] `just test-contract`
- [ ] `just docs`
- [ ] `just test-e2e-fast`, if Docker/runtime behavior changed.

## Final Report

- Simplifications made:
- Public behavior preserved:
- Tests changed:
- Commands run:
- Remaining risks or follow-up:

---
name: Redesign
about: Bold, cross-cutting redesign of a single diagnosed concern, backed by the tests.
title: "Redesign: "
labels: ready
assignees: ""
---

## Goal

Redesign the specific concern named by the diagnosis that routed here — boldly if
the concern needs it. This is the counterweight to
[implementation-cleanup](implementation-cleanup.md): cleanup is small and safe;
redesign is allowed to rename, restructure, delete, and reframe.

## Scope

- The single concern named by the routing
  [diagnosis](repository-diagnosis.md) finding.
- Whatever that one concern spans — code, tests, and docs together (a vertical
  slice). Unlike the audit leaves, this task **may cross artifact lanes** for one
  coherent concern.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [docs/work-items.md](../../docs/work-items.md).
- Be bold **within the diagnosed scope**; do not expand beyond it, even if the
  diff is large.
- Lean on the test suite as the safety net: keep public behavior covered. If the
  redesign changes a public contract, update docs and tests in the same PR and
  call it out.
- Do not weaken or delete tests to make the redesign pass. Where behavior changes
  on purpose, rewrite the tests to assert the new shape.
- Prefer clean removal and renaming over compatibility shims (see the
  compatibility policy in
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md)).

## Tasks

- [ ] Restate the diagnosed concern and the target shape.
- [ ] Make the change as one coherent vertical slice (code + tests + docs).
- [ ] Rename, refocus, or delete as the target shape requires.
- [ ] Update README, docs, examples, and tests for any changed behavior.

## Acceptance Criteria

- [ ] The diagnosed concern is resolved in its target shape.
- [ ] Tests cover the new shape; no behavior is silently lost.
- [ ] Public-contract changes are documented and tested in the same PR.
- [ ] The change stays within the diagnosed scope.

## Checks

- [ ] `just lint`
- [ ] `just typecheck`
- [ ] `just test-unit`
- [ ] `just test-contract`
- [ ] `just docs`
- [ ] `just check`
- [ ] `just test-e2e-fast`, if Docker/runtime behavior changed.

## Final Report

- Concern redesigned:
- Target shape achieved:
- Code / tests / docs changed:
- Public behavior changes, with migration notes:
- Commands run:
- Remaining risks or follow-up:

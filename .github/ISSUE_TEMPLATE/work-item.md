---
name: Work item
about: Track actionable project work that should be handled through an issue and PR.
title: ""
labels: agent-ready
assignees: ""
---

## Goal

Describe the outcome this issue should achieve.

## Scope

- In scope:
- Out of scope:

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md) for branch setup, checks, CI,
  PR workflow, merge policy, and releases.
- Follow [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md) for
  quality rules and definition of done.
- Follow [AGENTS.md](../../AGENTS.md) for agent-specific operating rules.
- Keep the change small and coherent.
- Do not skip, weaken, or delete tests/CI to make this pass.

## Acceptance Criteria

- [ ] The requested behavior or documentation exists.
- [ ] Public behavior changes update README, docs, examples, schemas, and tests
      where applicable.
- [ ] The PR links this issue with `Fixes #<number>`, `Closes #<number>`, or
      `Refs #<number>`.
- [ ] The PR body records local checks and any manual verification.

## Checks

- [ ] `just lint`
- [ ] `just typecheck`
- [ ] `just test-unit`
- [ ] `just test-contract`
- [ ] `just docs`
- [ ] `just check`
- [ ] Docker/E2E checks, if Docker/runtime behavior changed.

## Final Report

- Changed files:
- Behavior or documentation changed:
- Checks run:
- Remaining risks or follow-up:

---
name: Test and CI audit
about: Audit tests, CI wiring, and coverage of documented behavior.
title: "Audit test and CI coverage"
labels: agent-ready
assignees: ""
---

## Goal

Audit and improve test coverage and test execution wiring.

## Scope

- tests/.
- pyproject.toml or equivalent test config.
- .pre-commit-config.yaml.
- .github/workflows/*.
- README/CONTRIBUTING/DEVELOPMENT sections that describe testing.
- Minimal source changes only if required to make behavior testable.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [AGENTS.md](../../AGENTS.md).
- Do not refactor implementation except where needed to make tests possible.
- Do not skip, weaken, or delete tests/CI to make this pass.

## Tasks

- [ ] Map every public README-documented command, feature, and CLI behavior to
      existing tests where feasible.
- [ ] Add missing tests for feasible documented behavior.
- [ ] Identify behavior that cannot sensibly be tested automatically and
      document a manual verification check.
- [ ] Verify that tests are collected and actually executed.
- [ ] Check that tests are not accidentally skipped, deselected, ignored, or
      excluded by config.
- [ ] Ensure skipped/xfail tests have explicit reasons.
- [ ] Verify that local test instructions, pre-commit, CI, and release
      validation are consistent.

## Acceptance Criteria

- [ ] Documented public behavior has appropriate automated coverage or a
      recorded manual verification reason.
- [ ] Local test commands and CI workflows agree on the intended gates.
- [ ] Changed behavior includes tests before the PR is marked ready.

## Checks

- [ ] `just lint`
- [ ] `just typecheck`
- [ ] `just test-unit`
- [ ] `just test-contract`
- [ ] `just docs`
- [ ] `just check`
- [ ] `just test-e2e-fast`, if Docker/runtime behavior changed.

## Final Report

- Changed files:
- Added or fixed tests:
- Remaining untested behavior and why:
- Commands run:

---
name: Quality workflow
about: Run the multi-PR repository quality workflow.
title: "Run repository quality workflow"
labels: agent-ready
assignees: ""
---

## Goal

Execute a multi-goal repository quality workflow with focused PRs.

## Autonomous Mode

- [ ] This issue explicitly requests autonomous PR creation and auto-merge.

If autonomous mode is not checked, open draft review PRs by default and stop
after each PR is ready for human review.

## Constraints

- Follow [CONTRIBUTING.md](../../CONTRIBUTING.md),
  [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md), and
  [AGENTS.md](../../AGENTS.md).
- Create one focused PR per goal.
- Do not combine unrelated goals into one PR.
- Start every goal from latest `origin/main`.
- Do not start a dependent goal until prerequisite PRs are merged.
- If CI fails, fix the current PR instead of starting a new goal.
- If a PR cannot be merged automatically, stop and report the blocker.
- Do not weaken tests, remove checks, or hide failures with broad skips/xfails.
- Keep each PR small and reviewable.

## Execution Plan

- [ ] Create and complete an issue from
      [test-ci-audit.md](test-ci-audit.md).
- [ ] Create and complete an issue from
      [documentation-audit.md](documentation-audit.md).
- [ ] After both are merged into `main`, create and complete an issue from
      [implementation-cleanup.md](implementation-cleanup.md).
- [ ] After the cleanup PR is created, create and complete an issue from
      [maintainer-review.md](maintainer-review.md) as a review-only task.

## Acceptance Criteria

- [ ] Each goal has a separate issue and PR.
- [ ] Each PR links its issue and reports checks run.
- [ ] Dependent goals start only after prerequisite PRs merge.

## Checks

Use the checks listed in each issue template.

## Final Report

- Issues created:
- PRs created:
- PR URLs and CI status:
- Merged PRs:
- Blockers:

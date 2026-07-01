---
name: Quality workflow
about: Run the multi-PR repository quality workflow.
title: "Run repository quality workflow"
labels: ready
assignees: ""
---

## Goal

Run the repository quality pass: diagnose first, then create one focused PR for
each goal the diagnosis triages — in priority order. A healthy repo may yield no
PRs at all; that is a successful outcome.

## Autonomous Mode

- [ ] This issue explicitly requests autonomous PR creation and auto-merge.

If autonomous mode is not checked, open draft review PRs by default and stop
after each PR is ready for review.

## Constraints

- Follow the shared [task contract](../../docs/quality-model.md#task-contract):
  workflow, quality rules, work tracking, and test integrity.
- Create one focused PR per goal.
- Do not combine unrelated goals into one PR.
- Start every goal from latest `origin/main`.
- Do not start a dependent goal until prerequisite PRs are merged.
- If CI fails, fix the current PR instead of starting a new goal.
- If a PR cannot be merged automatically, stop and report the blocker.
- Keep each PR focused and reviewable.

## Execution Plan

- [ ] Create and complete an issue from
      [repository-diagnosis.md](repository-diagnosis.md). This read-only pass
      triages findings by altitude and disposition and proposes the issues to
      open. If it finds nothing actionable, report that and stop here.
- [ ] For each actionable finding, in the diagnosis's priority order, create and
      complete one focused issue using the leaf template it names
      ([test-ci-audit.md](test-ci-audit.md),
      [documentation-audit.md](documentation-audit.md),
      [implementation-cleanup.md](implementation-cleanup.md), or
      [redesign.md](redesign.md)).

## Acceptance Criteria

- [ ] The diagnosis ran first and its triage is recorded.
- [ ] Each triaged goal has a separate issue and PR (a healthy repo may have
      none — record the "no action" result instead).
- [ ] Each PR links its issue and reports checks run.
- [ ] Dependent goals start only after prerequisite PRs merge.

## Checks

Use the checks listed in each issue template.

## Final Report

- Diagnosis summary (or "no action needed"):
- Issues created:
- PRs created:
- PR URLs and CI status:
- Merged PRs:
- Net line delta (product vs governance), with any net increase justified by the
  capability it bought:
- Blockers:

---
name: Maintainer review
about: Review a pull request from a strict maintainer perspective.
title: "Review PR #"
labels: ready
assignees: ""
---

## Goal

Review the referenced PR as a strict maintainer.

## PR

- PR URL or number:

## Constraints

- Follow the shared [task contract](../../docs/quality-model.md#task-contract):
  workflow, quality rules, work tracking, and test integrity.
- Do not modify files unless explicitly asked.
- Focus on bugs, regressions, missing tests, and process risks before style.

## Review Focus

- [ ] Scope creep.
- [ ] Missing or weak tests.
- [ ] Documentation drift.
- [ ] Accidental public API or CLI changes.
- [ ] Dead code left behind.
- [ ] Unnecessary abstraction.
- [ ] CI/pre-commit mismatch.
- [ ] Release or packaging risks.

## Acceptance Criteria

- [ ] Blocking issues are listed first and include file/line references.
- [ ] Non-blocking suggestions are clearly separated.
- [ ] The final recommendation is `merge` or `request changes`.

## Checks

- [ ] Inspect PR diff.
- [ ] Inspect CI status.
- [ ] Run local checks only if needed to verify a finding.

## Final Report

- Blocking issues:
- Non-blocking suggestions:
- Files/lines for each issue:
- Final recommendation:

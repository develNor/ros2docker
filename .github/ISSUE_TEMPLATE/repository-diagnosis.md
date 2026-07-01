---
name: Repository diagnosis
about: Read-only diagnosis and triage that decides what, if anything, the repo needs.
title: "Diagnose repository and triage follow-up work"
labels: ready
assignees: ""
---

## Goal

Take in the whole repository and produce a prioritized, severity-classified
diagnosis that decides what work — if any — the repo needs. This is the planning
keystone of the quality pass: it **routes, it does not edit**.

## How This Fits

This runs first in the soft-check pass (see
[docs/quality-model.md](../../docs/quality-model.md)). The
[quality-workflow](quality-workflow.md) orchestrator then creates and completes
exactly the issues this triage names — which may be none.

## Constraints

- **Read-only**: do not change code, docs, tests, or config in this task.
- Follow the shared [task contract](../../docs/quality-model.md#task-contract);
  only its quality-rules and tracking parts apply, since this task opens no PR
  and changes no tests.
- Classify by what a finding *needs*, not by what is least invasive. Do not
  pre-bias toward small changes.
- "No action needed" is a valid, first-class outcome — report it as success.
- Judge the repo on its own terms. This recipe must also work on a large,
  complex repository, not only this one.

## Diagnose Along Two Axes

Record each finding with an **altitude** (where it lives) and a **disposition**
(what it needs).

Altitude:

- **Vision** — does the README / stated purpose still match what the repo is?
- **Architecture** — module boundaries, decomposition, naming in the large.
- **Module / interface** — one unit's shape, cohesion, and naming.
- **Line / local** — dead code, duplication, micro-issues.
- **Meta / governance** — do the docs and process earn their keep, and are they
  proportionate to the product?

Disposition:

- **None** — healthy; leave it.
- **Inline** — trivial; fold into a routine leaf pass.
- **Scoped** — a normal focused issue ([test-ci-audit](test-ci-audit.md),
  [documentation-audit](documentation-audit.md), or
  [implementation-cleanup](implementation-cleanup.md)).
- **Redesign** — bold, possibly large, cross-cutting change; route to
  [redesign](redesign.md).

## Lenses (apply across altitudes)

- **Value / proportionality**: does each file or section earn its place, or does
  it belong in a more canonical home? Is the governance layer proportionate to
  the product it governs?
- **Generality**: do files meant to be reusable avoid repo-specific names and
  details? Allow specifics only where they must live (README badges, CODEOWNERS,
  packaging metadata, local instance config).
- **Drift**: do documented workflows, commands, and settings match the actual
  repository?

## Tasks

- [ ] Read top-down: purpose (README) → architecture → modules → lines, plus the
      meta / governance layer.
- [ ] Record findings with altitude, disposition, and a one-line rationale.
- [ ] Apply the value, generality, and drift lenses.
- [ ] Order findings by priority and note dependencies between them.
- [ ] Propose one issue per actionable finding, naming the leaf template to use.
- [ ] If nothing is actionable, say so explicitly.

## Acceptance Criteria

- [ ] Every finding has an altitude and a disposition.
- [ ] The report includes an explicit "no action" section.
- [ ] Each actionable finding maps to a proposed issue and a leaf template.
- [ ] No files were changed by this task.

## Final Report

Triage table:

| Finding | Altitude | Disposition | Target template | Rationale |
| --- | --- | --- | --- | --- |

- No action needed:
- Proposed issues, in priority order:
- Dependencies between proposed issues:

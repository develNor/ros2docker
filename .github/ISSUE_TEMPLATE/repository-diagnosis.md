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
- "No action needed" is a valid, first-class outcome — but it must be *earned*:
  name the probe you actually ran and what a problem would have looked like. The
  absence of surface markers (no TODO/FIXME, green tests) is not evidence on its
  own — for code that means an architecture and proportionality read; for a doc,
  a "does this still earn its place" read.
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

- **Value / proportionality** — a budget, not just a question: every file,
  section, and template must be *load-bearing* (enables a capability or prevents a
  real error class) and *stated once* (no rule restated across files). The target
  is *earns its keep*, not *small* — a model repo may carry substantial machinery
  — so a governance/process layer that grows without adding capability is itself
  an actionable finding to route, not merely note. Test earning-its-keep
  **per artifact** with the deletion counterfactual (the two-layer model in
  [quality-model.md](../../docs/quality-model.md)): for each agent-layer artifact,
  if it were deleted, which owner-visible outcome would change? It earns its place
  only if some owner-entrypoint path
  ([owner-runbook.md](../../docs/owner-runbook.md)) reaches it *and* deleting it
  would change a concrete outcome (a merge it gates, what ships, or what the agent
  produces). A layer-level answer ("the whole agent layer routes work and gates
  surfaces") does **not** justify any single artifact. Artifacts whose deletion
  changes nothing — an audit that never finds anything, a doc no entrypoint needs,
  a review whose verdict does not gate the merge it reviews — are empty weight:
  route them for removal or make them load-bearing. Ask not only "is this
  duplicated?" but "does any owner path need this file or concept at all?"
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
- [ ] Size the layers (product vs governance/process) and probe the product on
      its own terms — architecture and proportionality, not just surface markers
      — before assigning any disposition.
- [ ] For each leaf template individually, state the owner-visible outcome that
      would change if it were deleted; flag any with "none" (reached but inert) as
      empty weight — a layer-level "it all routes work" answer does not satisfy this.
- [ ] Order findings by priority and note dependencies between them.
- [ ] Propose one issue per actionable finding, naming the leaf template to use.
- [ ] If nothing is actionable, say so explicitly.

## Acceptance Criteria

- [ ] Every finding has an altitude and a disposition.
- [ ] The report includes an explicit "no action" section.
- [ ] Every `None` disposition cites the probe that would have caught a problem,
      not just the absence of surface markers.
- [ ] The report states the product-vs-governance size ratio and whether the
      governance layer earns its current size.
- [ ] The report lists every leaf template with the owner-visible outcome that
      would change if it were deleted; any with "none" is flagged as empty weight
      (not blessed by the layer's overall value).
- [ ] Each actionable finding maps to a proposed issue and a leaf template.
- [ ] No files were changed by this task.

## Final Report

Triage table:

| Finding | Altitude | Disposition | Target template | Rationale |
| --- | --- | --- | --- | --- |

- No action needed (each with the probe that would have caught a problem):
- Layer sizes and proportionality verdict (product vs governance/process):
- Leaf reachability (each leaf template → the owner-visible outcome that would
  change if deleted; "none" = empty weight to route):
- Proposed issues, in priority order:
- Dependencies between proposed issues:

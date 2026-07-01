# Quality Model: Hard and Soft Checks

This repository is **issue-driven** and largely **agent-built**. That has a known
failure mode: agents tend to add the least-invasive lines that make a behavior
work rather than redesign, and work arrives as a stream of independent issues. So
the codebase grows *organically* and drifts:

- names and framing stop matching what the code actually does;
- interfaces erode through patches instead of being redesigned;
- README/docs fall behind or contradict the code;
- abstractions and dead code accumulate.

Some of that drift is mechanically detectable; much of it is not. So the project
uses **two layers of checks**.

## Hard checks (deterministic, in CI)

Hard checks are objective and block merges. They live in CI and the `justfile`,
and the [contract tests](../tests/contract/) encode repository invariants
(config surface, docs reachability, link resolution, profile/just-target
consistency, security config, …). If a hard check can express a rule, it should:
deterministic enforcement beats review (*prefer deterministic checks over
judgement*). See [ci.md](ci.md) for what runs.

## Soft checks (agent-run audits / review passes)

Some properties are not reliably algorithmic — "is this name still apt?", "is
this doc still relevant and correct?", "did this interface erode?". For those the
project uses **soft checks**: focused audit and review tasks, captured as reusable
[issue templates](../.github/ISSUE_TEMPLATE/), run periodically — especially
before a release. They are checklists, not gates;
deliberately *not* wired into CI (a "did an audit run recently?" gate would be
brittle and low-value — see #98).

## The task hierarchy

Issue templates must live flat in `.github/ISSUE_TEMPLATE/` (GitHub's chooser has
no subfolders), but they form a hierarchy of **orchestrators** and **leaf tasks**:

```
owner-runbook  (the owner's entrypoints — see owner-runbook.md)
├─ quality-workflow            orchestrator: diagnose first, then one focused PR per triaged goal
│  ├─ repository-diagnosis     keystone: read-only; triage findings, may find nothing
│  ├─ test-ci-audit            leaf: tests, CI wiring, coverage of documented behavior
│  ├─ documentation-audit      leaf: justify-or-delete docs; remove drift/duplication/bloat
│  ├─ implementation-cleanup   leaf: small, safe simplification of code
│  └─ redesign                 leaf: bold, cross-cutting redesign of a diagnosed concern
└─ release-process             leaf: prepare, validate, and publish a version (see release.md)
work-item                      leaf: any other actionable change (not an owner entrypoint)
```

| Template | Role |
|---|---|
| [quality-workflow](../.github/ISSUE_TEMPLATE/quality-workflow.md) | Orchestrates the soft-check pass: diagnose first, then a focused PR per triaged goal. |
| [repository-diagnosis](../.github/ISSUE_TEMPLATE/repository-diagnosis.md) | Read-only diagnosis and triage that routes the pass. The planning keystone. |
| [test-ci-audit](../.github/ISSUE_TEMPLATE/test-ci-audit.md) | Audit tests, CI wiring, and coverage of documented behavior. |
| [documentation-audit](../.github/ISSUE_TEMPLATE/documentation-audit.md) | Justify-or-delete public docs; remove duplicated, stale, or disproportionate text. |
| [implementation-cleanup](../.github/ISSUE_TEMPLATE/implementation-cleanup.md) | Small, safe simplification that preserves the public contract. |
| [redesign](../.github/ISSUE_TEMPLATE/redesign.md) | Bold, cross-cutting redesign of a single diagnosed concern, backed by the tests. |
| [work-item](../.github/ISSUE_TEMPLATE/work-item.md) | Track a single actionable change. |
| [release-process](../.github/ISSUE_TEMPLATE/release-process.md) | Prepare, validate, describe, and publish a release. |

## Task contract

Every template shares one contract, whatever the task. Templates link it instead
of restating it, and add only the constraints specific to their own goal:

- **Workflow** — [CONTRIBUTING.md](../CONTRIBUTING.md): branch setup, local
  checks, CI, PR modes, merge policy, and releases.
- **Quality** — [DEVELOPMENT_PRINCIPLES.md](../DEVELOPMENT_PRINCIPLES.md):
  quality rules and the definition of done.
- **Tracking** — [work-items.md](work-items.md): issue ownership, comments,
  labels, and PR linkage.
- **Test integrity** — never skip, weaken, or delete tests or CI checks, and do
  not hide failures behind broad skips or xfails, to make a change pass.

A read-only task follows only the parts that apply — quality rules and tracking
— since it opens no PR and changes no tests; see
[repository-diagnosis](../.github/ISSUE_TEMPLATE/repository-diagnosis.md).

## Two layers: owner and agent

The repository has two audiences, and every file serves one:

- **Owner layer** — the handful of entrypoints the owner invokes, collected in
  [owner-runbook.md](owner-runbook.md): the quality maintenance pass, cutting a
  release, and the three owner-only gates. For whoever owns the repo, this is all
  they touch.
- **Agent layer** — the issue templates and the supporting docs agents read while
  executing those entrypoints, not invoked directly at the owner layer.

This split makes proportionality decidable — **per artifact, not per layer.** For
each agent-layer artifact ask the counterfactual: *if it were deleted, which
owner-visible outcome would change?* It earns its place only when some
owner-entrypoint path **reaches** it *and* deleting it would change a concrete
outcome (a merge it gates, what ships, or what the agent produces). "The layer as
a whole routes work and gates surfaces" does **not** justify any single artifact —
that reasoning lets an inert step ride on its neighbours. Artifacts whose deletion
changes nothing — an audit that never finds anything, a doc no entrypoint needs, a
review whose verdict does not gate the merge it reviews — are empty weight: the
diagnosis routes them for removal or for being made load-bearing. The
`value / proportionality` lens asks this first: not "is this duplicated?" but
"does any owner path need this at all?"

## Diagnosis first: route by need, not by reflex

The soft-check pass does not fire every leaf on every run. It starts with one
read-only pass — [repository-diagnosis](../.github/ISSUE_TEMPLATE/repository-diagnosis.md)
— that takes in the whole repo and classifies each finding two ways:

- **Altitude** — where it lives: *vision* (does the README still match the
  repo?), *architecture*, *module / interface*, *line / local*, and *meta /
  governance* (do the docs and process earn their keep, proportionate to the
  product?).
- **Disposition** — what it needs: *none* (healthy — a first-class outcome,
  earned with an actual probe, not just the absence of surface markers),
  *inline*, *scoped* (a focused audit or cleanup leaf), or *redesign* (a bold,
  cross-cutting change routed to
  [redesign](../.github/ISSUE_TEMPLATE/redesign.md)).

Three lenses apply across altitudes: **value / proportionality** — a budget, not
just a question: each artifact must earn its keep (tested per-artifact by the deletion
counterfactual in *Two layers* above) and be stated once, so a layer that grows without adding
capability is itself a routable finding — **generality**
(reusable files avoid repo-specific names except where they must live — badges,
CODEOWNERS, packaging, local config), and **drift** (do documented commands and
settings match reality?).

The orchestrator then creates and completes exactly the issues triage names. A
healthy repo legitimately produces **zero PRs** — that is success, not a failed
run. Each pass reports its net line delta (product vs governance) so the workflow
is accountable to *reduction*, not just activity.

## Depth comes from decomposition, not one big pass

A natural instinct is "one prompt that does everything" — audit, refactor,
document, review, release in a single run. **Don't.** A single forward pass
splits attention across categories and does each one shallowly, with no fresh
perspective between them. Depth comes from **decomposition + fresh context per
goal**: that is exactly why `quality-workflow` mandates *one focused PR per goal,
each started from a fresh `origin/main`, dependent goals waiting for prerequisites
to merge*.

So a single owner entrypoint is fine — but it should **orchestrate** focused,
fresh-context tasks (a separate issue + PR, ideally a separate agent run, per
goal), not collapse them into one pass. The command is one; the execution is
many. The concrete entrypoints are collected in the
[owner runbook](owner-runbook.md).

Decomposition also has a *direction*. The audit leaves decompose **horizontally**
by artifact — tests, then docs, then code — and each stays in its lane. Some
improvements only cohere as a **vertical** slice: one concept's code, tests,
docs, and naming changed together. That is what
[redesign](../.github/ISSUE_TEMPLATE/redesign.md) is for — triage routes a single
concern to it, and it is licensed to cross the artifact lanes and be bold, backed
by the tests. Cleanup is small and safe; redesign is bold and scoped; diagnosis
decides which, or neither.

## Evolving this machinery

This machinery is **stable by default** — "adaptable" (cheap to change) is not
"needs adapting" (a treadmill). Change a template or check only in reaction to a
*concrete, observed misbehavior* — the same evidence bar as a bug fix,
never on a schedule or a hunch. The method: observe the misbehavior → encode the
fix as a hard check (preferred) or a forcing question at the point of work →
verify by re-running the workflow. Apply the *Two layers* reachability test to the
rule itself first: name the real run where it would have changed the outcome; if
it cannot be named, do not add it. The code-owners review is where this judgment
lives — it is the meta-check and the top of the ladder, with no level above it to
automate.

## Document ownership

Each public-facing document owns a distinct concern; keep content in its owner and
link rather than restate (this map is what the `documentation-audit` pass
enforces):

- **README**: user-facing purpose, installation, quick start, common usage.
- **CONTRIBUTING**: contributor workflow, PR expectations, local checks.
- **DEVELOPMENT_PRINCIPLES**: quality rules and the definition of done.
- **docs/**: focused supporting guides — [owner-runbook.md](owner-runbook.md)
  (the owner's entrypoints and the three gates), [ci.md](ci.md),
  [configuration.md](configuration.md), [release.md](release.md),
  [work-items.md](work-items.md), and this file. Repository policy that lives
  outside git is mirrored in
  [github-repository-settings.md](github-repository-settings.md).
- **Issue templates** (`.github/ISSUE_TEMPLATE/`): reusable, executable task
  recipes — the *do*, where the docs are the *read*.

## See also

- [owner-runbook.md](owner-runbook.md) — the handful of prompts the owner runs
  and the three owner-only gates (admin, CI/tests/deps, releases).
- [work-items.md](work-items.md) — where work lives and how issues drive it.
- [release.md](release.md) — the owner release runbook.

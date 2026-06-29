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
deterministic enforcement beats review. See [ci.md](ci.md) for what runs and
[quality-audit.md](quality-audit.md) for the rule of thumb (*prefer deterministic
checks over judgement*).

## Soft checks (agent-run audits / review passes)

Some properties are not reliably algorithmic — "is this name still apt?", "is
this doc still relevant and correct?", "did this interface erode?". For those the
project uses **soft checks**: focused audit and review tasks, captured as reusable
[issue templates](../.github/ISSUE_TEMPLATE/), that an agent (or human) runs
periodically — especially before a release. They are checklists, not gates;
deliberately *not* wired into CI (a "did an audit run recently?" gate would be
brittle and low-value — see #98).

## The task hierarchy

Issue templates must live flat in `.github/ISSUE_TEMPLATE/` (GitHub's chooser has
no subfolders), but they form a hierarchy of **orchestrators** and **leaf tasks**:

```
release-readiness  (owner runbook, see release.md)
└─ quality-workflow            orchestrator: one focused PR per goal, in order
   ├─ test-ci-audit            leaf: tests, CI wiring, coverage of documented behavior
   ├─ documentation-audit      leaf: align docs, remove drift/duplication
   ├─ implementation-cleanup   leaf: simplify code, remove dead/over-abstracted paths
   └─ maintainer-review        leaf: strict review of a PR
work-item        leaf: any other actionable change
release-process  leaf: prepare, validate, and publish a version
```

| Template | Role |
|---|---|
| [quality-workflow](../.github/ISSUE_TEMPLATE/quality-workflow.md) | Orchestrates the soft-check pass as a sequence of focused PRs. |
| [test-ci-audit](../.github/ISSUE_TEMPLATE/test-ci-audit.md) | Audit tests, CI wiring, and coverage of documented behavior. |
| [documentation-audit](../.github/ISSUE_TEMPLATE/documentation-audit.md) | Align public docs; remove duplicated or stale text. |
| [implementation-cleanup](../.github/ISSUE_TEMPLATE/implementation-cleanup.md) | Simplify implementation while preserving the public contract. |
| [maintainer-review](../.github/ISSUE_TEMPLATE/maintainer-review.md) | Review a PR from a strict maintainer perspective. |
| [work-item](../.github/ISSUE_TEMPLATE/work-item.md) | Track a single actionable change. |
| [release-process](../.github/ISSUE_TEMPLATE/release-process.md) | Prepare, validate, describe, and publish a release. |

## Depth comes from decomposition, not one big pass

A natural instinct is "one prompt that does everything" — audit, refactor,
document, review, release in a single run. **Don't.** A single forward pass
splits attention across categories and does each one shallowly, with no fresh
perspective between them. Depth comes from **decomposition + fresh context per
goal**: that is exactly why `quality-workflow` mandates *one focused PR per goal,
each started from a fresh `origin/main`, dependent goals waiting for prerequisites
to merge*.

So a single human entrypoint is fine — but it should **orchestrate** focused,
fresh-context tasks (a separate issue + PR, ideally a separate agent run, per
goal), not collapse them into one pass. The command is one; the execution is
many. The concrete entrypoint is the owner release runbook in
[release.md](release.md).

## Document ownership

Each public-facing document owns a distinct concern; keep content in its owner and
link rather than restate (this map is what the `documentation-audit` pass
enforces):

- **README**: user-facing purpose, installation, quick start, common usage.
- **CONTRIBUTING**: contributor workflow, PR expectations, local checks.
- **DEVELOPMENT_PRINCIPLES**: quality rules and the definition of done.
- **docs/**: focused supporting guides — [agentic-workflow.md](agentic-workflow.md)
  (the human + agent model and the three gates), [ci.md](ci.md),
  [configuration.md](configuration.md), [release.md](release.md),
  [work-items.md](work-items.md), [quality-audit.md](quality-audit.md), and this
  file.
- **Issue templates** (`.github/ISSUE_TEMPLATE/`): reusable, executable task
  recipes — the *do*, where the docs are the *read*.

## See also

- [agentic-workflow.md](agentic-workflow.md) — autonomy by default and the three
  human-only gates (admin, CI/tests/deps, releases).
- [work-items.md](work-items.md) — where work lives and how issues drive it.
- [release.md](release.md) — the owner release runbook.

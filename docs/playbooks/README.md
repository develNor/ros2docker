# Playbooks

These playbooks are reusable task recipes for maintainers, contributors, and
agents. They describe proven workflows for focused repository work, but they do
not replace the canonical project rules.

Canonical rules live in:

- [CONTRIBUTING.md](../../CONTRIBUTING.md) for setup, checks, CI, PR workflow,
  merge policy, and releases.
- [DEVELOPMENT_PRINCIPLES.md](../../DEVELOPMENT_PRINCIPLES.md) for quality
  rules and definition of done.
- [AGENTS.md](../../AGENTS.md) for agent-specific operating rules.

## Available Playbooks

- [quality-workflow.md](quality-workflow.md): run the multi-PR quality workflow.
- [test-ci-audit.md](test-ci-audit.md): audit tests, CI wiring, and coverage of
  documented behavior.
- [documentation-audit.md](documentation-audit.md): align public-facing docs and
  remove duplicated or stale process text.
- [implementation-cleanup.md](implementation-cleanup.md): simplify
  implementation while preserving the public contract.
- [maintainer-review.md](maintainer-review.md): review a PR from a strict
  maintainer perspective.
- [release-process.md](release-process.md): prepare, validate, describe, and
  publish a release.

## Playbook Format

Prefer concise playbooks with:

- a clear goal,
- when to use it,
- scope and rules,
- ordered tasks,
- checks to run,
- the expected final report.

Keep policy details in the canonical docs and link to them from playbooks
instead of duplicating them.

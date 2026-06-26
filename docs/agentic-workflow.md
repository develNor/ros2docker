# Agentic Workflow

This repository is a model for human + multi-agent local development. The goal is
**maximum agent autonomy with the minimum set of human-only gates** needed to
stay safe. This document explains the model; the enforcing settings live in
[github-repository-settings.md](github-repository-settings.md) and the day-to-day
operating rules in [../AGENTS.md](../AGENTS.md).

## Identity

Agents act as a dedicated bot account, **`develNor-agent`**, which is separate
from the human owner (`develNor`). The bot is a non-admin **collaborator** and
authenticates with a fine-grained, repository-scoped token kept locally in
`.agents/github.env` (gitignored).

Two benefits follow:

- **Clean attribution.** Issues, comments, PRs, and commits are natively the
  bot's, so agent work is distinct from the human's — no self-review theater and
  no `Co-Authored-By` trailers (see [../AGENTS.md](../AGENTS.md)).
- **Least privilege by construction.** On a personal (user-owned) repository a
  non-admin collaborator simply cannot reach the admin tier, so most of the
  "keep this human-only" list needs no extra configuration.

## Autonomy by default

Agents are expected to work end to end without asking, including:

- create, refine, label, and close issues;
- open pull requests and enable auto-merge (`--squash`);
- read repository code, issues, checks, and Actions logs;
- trigger, re-run, and inspect CI workflows;
- edit workflow files and tests as part of a change.

A PR that does **not** touch an owned path (below) merges automatically once
`ci-success` is green. No human approval is required for the common case.

## The three human-only gates

Everything an agent should *not* be able to do alone reduces to three gates:

1. **Repository administration** — settings, rulesets and branch protection,
   secrets, Actions policy, visibility, and deletion/transfer. Reserved to the
   owner automatically because the bot is a non-admin collaborator.

2. **CI / tests / dependencies** — changes that could weaken the harness.
   `.github/CODEOWNERS` assigns the owner to that policy surface, and the
   `Protect main` ruleset has **require Code Owner review** on with the owner on
   the bypass list. A bot PR touching those files waits for the owner's approval
   (the bot is a different account and cannot self-approve); the owner's own PRs
   bypass. This is the only added gate on the otherwise-autonomous flow.

3. **Releases** — `v*` tags are restricted to the owner, and the `pypi`
   deployment environment requires the owner as a reviewer, so the irreversible
   publish step always waits for a human.

## Why this shape

- The CI harness can only be weakened by editing a small, enumerated set of
  files; gating exactly those keeps autonomy everywhere else.
- A required Code Owner review is meaningful only with two identities — with a
  single account the owner would merely be gating themselves and could not
  self-approve. The bot account is what makes the gate real.
- "What protections are active" is answered by the committed contract in
  [github-repository-settings.md](github-repository-settings.md); the live
  GitHub settings remain the source of truth and are mirrored there, so agents do
  not need admin read access to understand policy.

## Day-to-day

See [../CONTRIBUTING.md](../CONTRIBUTING.md) for the shared workflow,
[../AGENTS.md](../AGENTS.md) for agent operating rules (worktree-per-issue,
auto-merge default, credentials, attribution), and
[work-items.md](work-items.md) for issue-driven tracking.

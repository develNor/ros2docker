# Work Items

Use GitHub Issues for work items, repo files for reusable knowledge, and local
files only for private scratch.

## Where Work Lives

- Real actionable work lives in GitHub Issues so it can be discussed, assigned,
  linked to PRs, and closed by merges.
- Reusable work recipes live in
  [.github/ISSUE_TEMPLATE/](../.github/ISSUE_TEMPLATE/).
- In-flight PR checklists live in the PR body, not in committed todo files.
- Private scratch notes stay outside the repository unless they become reusable
  project knowledge.
- Project policy and definition of done live in committed repository docs.

## Issue-Driven Workflow

Use the matching issue template for reusable work:

```bash
gh issue create --repo develNor/ros2docker --template work-item.md
gh issue create --repo develNor/ros2docker --template documentation-audit.md
gh issue create --repo develNor/ros2docker --template test-ci-audit.md
```

Then work from an up-to-date topic branch:

```bash
git fetch --prune
git switch -c docs/example-work origin/main
```

Open a draft review PR by default:

```bash
gh pr create \
  --draft \
  --base main \
  --title "Document example work" \
  --body "Fixes #123

Summary:
- Adds the documented workflow.

Checks:
- just docs"
```

Use `Fixes #123` or `Closes #123` when the PR should close the issue after
merge. Use `Refs #123` when the PR relates to the issue but does not complete
it.

## Triage Labels

Use `agent-ready` only when an issue is accepted and ready for an agent or
contributor to pick up.

Use `backlog` for work that is tracked for later and not currently scheduled.

Use `maybe` with `backlog` for concerns, ideas, or proposals that should be
saved for later consideration but are not yet accepted. A `maybe` issue should
describe the decision to make before implementation, and should not be treated
as ready to start until the label is removed or replaced with `agent-ready`.

## Recurring Issues

Recurring maintenance should start as issues too. Keep the recurring recipe in
the matching issue template, then create the issue manually or with a scheduled
workflow after the cadence is proven.

This repository does not use scheduled issue creation yet. If scheduled issues
are added later, the workflow should create issues only; local agents and
contributors still own editing, testing, and opening PRs.

## Agent Use

Agents should read [AGENTS.md](../AGENTS.md), choose the matching issue
template, work on one issue at a time, link the PR to the issue, and report the
PR URL and CI status after opening or updating the PR.

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

## Recurring Issues

Recurring maintenance should start as issues too. Keep the recurring recipe in
the matching issue template, then create the issue manually or with a scheduled
workflow after the cadence is proven.

This repository does not use scheduled issue creation yet. If scheduled issues
are added later, the workflow should create issues only; contributors still own
editing, testing, and opening PRs.

## Working An Issue

Before implementation starts, create or choose a well-scoped issue with the
matching template when one applies.

While working:

- Assign yourself to the issue.
- Keep one coherent change per issue and PR.
- Link the PR with `Fixes #<number>`, `Closes #<number>`, or `Refs #<number>`.
- If implementation shows that the issue assumptions are wrong or incomplete,
  add an issue comment describing the discovery and the chosen scope.
- If the work is not possible or not sensible, add an issue comment explaining
  why and apply a fitting triage label such as `question`, `invalid`, or
  `help wanted`.
- Report the PR URL and CI status after opening or updating the PR.

# AGENTS.md

## Read first

- `README.md` for user-facing behavior.
- `CONTRIBUTING.md` for development setup, local checks, CI, PR workflow, and merge policy.
- `DEVELOPMENT_PRINCIPLES.md` for project quality rules.
- `tests/contract/test_public_config_surface.py` before changing README, examples, or config keys.

## Pull Request Workflow

All requested repository changes must be made through a pull request. Do not commit directly on `main`. Treat local `main` as a read-only mirror of `origin/main`.

Default mode is review-only. For review-only work:

- Create a short-lived topic branch from up-to-date `origin/main`.
- Make the change.
- Run the local checks described in `CONTRIBUTING.md`.
- Commit and push the branch.
- Open a draft pull request.
- Print the PR URL.
- Do not enable auto-merge.

A draft review PR is expected to run lightweight CI. It is not expected to be mergeable yet.

Use autonomous auto-merge mode only when the user explicitly asks for autonomous iteration, auto-merge, or completion without further review. For autonomous work:

- Create a short-lived topic branch from up-to-date `origin/main`.
- Make the change.
- Run the relevant local checks, including fast E2E when applicable.
- Open a non-draft pull request.
- Enable auto-merge.
- Monitor required CI.
- Fix failures on the same PR branch.
- Repeat until the PR merges or is clearly blocked.

All merges into `main` require the `ci-success` check. `ci-success` includes lightweight checks and fast E2E for ready PRs. Nightly full E2E is separate and does not replace the required PR merge gate.

## Git Workflow

Do not push to `main`. Treat local `main` as a clean mirror of `origin/main`.

For each change, create a short-lived topic branch from an up-to-date `origin/main`. Open a pull request for that branch. Enable auto-merge only when the PR is ready. After opening or updating a PR, check the PR status and CI result. If CI fails or auto-merge is blocked, fix the same PR branch and repeat.

After a PR has merged, fetch/prune, update local `main`, delete the merged topic branch, and start the next task from fresh `main`.

Useful commands:

```bash
git fetch --prune
git switch -c <type>/<short-topic> origin/main
gh pr create --draft --base main --title "<title>" --body "<summary>"
gh pr ready <number>
gh pr merge <number> --auto --squash --delete-branch
gh pr view <number> --comments
gh pr checks <number> --required
gh pr checks <number> --watch --required
```

# AGENTS.md

These instructions are for agents working in this repository. They add local
operating details; they do not replace the shared contributor workflow. Rules in
the project docs apply to agents the same way they apply to other developers.

Read first:

- `README.md` for user-facing behavior.
- `CONTRIBUTING.md` for setup, checks, CI, PR workflow, merge policy, and releases.
- `DEVELOPMENT_PRINCIPLES.md` for quality rules.
- `docs/work-items.md` for issue-driven work tracking.
- `.github/ISSUE_TEMPLATE/` for reusable task recipes when a request matches one.
- `docs/github-repository-settings.md` before changing GitHub repository policy or workflows.
- `tests/contract/test_public_config_surface.py` before changing README config examples, packaged examples, schemas, or config keys.

Shared workflow:

- Follow `CONTRIBUTING.md` for branch setup, local setup, check selection, CI, PR workflow, merge policy, and releases.
- Start every change from an up-to-date `origin/main` topic branch. Do not push directly to `main`.
- Create or use a well-scoped GitHub issue before implementation for actionable work. Use the matching issue template when one applies.
- Assign yourself to the issue while working on it.
- Link the PR to the issue with `Fixes #<number>`, `Closes #<number>`, or `Refs #<number>`.
- If implementation shows that the issue assumptions are wrong or incomplete, add an issue comment describing the discovery and the chosen scope.
- If the work is not possible or not sensible, add an issue comment explaining why and apply a fitting label such as `question`, `invalid`, or `help wanted`.
- Update README, docs, examples, schemas, and tests when CLI, config, API, Docker, or public behavior changes.
- Do not skip, weaken, or delete tests/CI to make a change pass.
- Default to a draft review PR with no auto-merge. Use autonomous auto-merge only when explicitly requested.
- After opening or updating a PR, check PR/CI status and print the PR URL.

Local concurrency:

- Check `git status --short --branch` before editing.
- Treat unrelated local changes as user-owned and do not revert them.
- Keep each issue on its own topic branch or worktree when parallel work is active.
- Do not duplicate shared contributor policy here. Update the shared docs when a rule applies to every developer.

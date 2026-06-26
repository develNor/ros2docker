# AGENTS.md

These instructions are for agents working in this repository. They add local
operating details; they do not replace the shared contributor workflow. Rules in
the project docs apply to agents the same way they apply to other developers.

Read first:

- `README.md` for user-facing behavior.
- `CONTRIBUTING.md` for setup, checks, CI, PR workflow, merge policy, and releases.
- `DEVELOPMENT_PRINCIPLES.md` for quality rules.
- `docs/work-items.md` for issue-driven work tracking.
- `docs/agentic-workflow.md` for the human + multi-agent model: identity, autonomy, and the human-only gates.
- `.github/ISSUE_TEMPLATE/` for reusable task recipes when a request matches one.
- `docs/github-repository-settings.md` before changing GitHub repository policy or workflows.
- `tests/contract/test_public_config_surface.py` before changing README config examples, packaged examples, schemas, or config keys.

Shared workflow:

- Follow `CONTRIBUTING.md` for branch setup, local setup, check selection, CI, PR workflow, merge policy, and releases.
- Start every change in a fresh git worktree created from an up-to-date `origin/main`, never in the user's primary checkout. Do not push directly to `main`. See "Worktrees" below.
- Create or use a well-scoped GitHub issue before implementation for actionable work. Use the matching issue template when one applies.
- Assign yourself to the issue while working on it.
- Link the PR to the issue with `Fixes #<number>`, `Closes #<number>`, or `Refs #<number>`.
- If implementation shows that the issue assumptions are wrong or incomplete, add an issue comment describing the discovery and the chosen scope.
- If the work is not possible or not sensible, add an issue comment explaining why and apply a fitting label such as `question`, `invalid`, or `help wanted`.
- Update README, docs, examples, schemas, and tests when CLI, config, API, Docker, or public behavior changes.
- Do not skip, weaken, or delete tests/CI to make a change pass.
- Open the PR ready and enable auto-merge by default: `gh pr merge --auto --squash --delete-branch` (squash is the only allowed method). Use a draft PR with no auto-merge only when the user asks for review before merge. Never bypass protected-branch rules, required checks, or required Code Owner review; if auto-merge is blocked, record why in the PR.
- After opening or updating a PR, check the real remote PR/CI state with `gh pr view` / `gh pr checks` and report the actual status with the PR URL — not just that a branch was pushed.

Worktrees:

- Use one git worktree per issue, created from `origin/main`:
  `git worktree add ../ros2docker-issue-<NN> -b issue-<NN>-<short-purpose> origin/main`.
  Name the branch so the issue and purpose are obvious.
- Keep the user's primary checkout untouched. Treat unrelated dirty state in any
  checkout or worktree as user-owned; do not clean, reset, or reuse it.
- Check `git status --short --branch` before editing.
- Remove the worktree after the PR merges: `git worktree remove ../ros2docker-issue-<NN>`.
- Do not duplicate shared contributor policy here. Update the shared docs when a rule applies to every developer.

Local GitHub credentials:

- Agents act as the dedicated bot account `develNor-agent`, not as the human
  owner. Because this repository is owned by another personal account
  (`develNor`) and the bot only collaborates on it, the bot authenticates with a
  **classic** personal access token with the `repo` and `workflow` scopes. (A
  fine-grained PAT cannot grant write to a repo owned by a different personal
  account; a GitHub App is the granular alternative — see
  `docs/agentic-workflow.md`.)
- Store the token only in `.agents/github.env`, which is gitignored and must
  stay local to this checkout. Do not print it, commit it, copy it into logs, or
  persist it anywhere else; source it only for the command that needs it.
- Use it for issue, PR, auto-merge, Actions/CI, and branch/push operations.
  Expected format:

  ```bash
  GITHUB_REPO=develNor/ros2docker
  GH_TOKEN=ghp_...
  GIT_AUTHOR_NAME=develNor-agent
  GIT_AUTHOR_EMAIL=<bot-id>+develNor-agent@users.noreply.github.com
  ```

- Least privilege still holds without per-repo token scoping: the bot is a
  non-admin collaborator, so it cannot change repository settings, rulesets, or
  secrets, cannot create `v*` release tags, and cannot approve the `pypi`
  deployment — those are owner-only by design. The token's blast radius is only
  the repositories the bot collaborates on. See `docs/agentic-workflow.md`.

Identity and attribution:

- Author commits and PRs as `develNor-agent` (set `user.name` / `user.email`
  from `.agents/github.env`) so agent work is natively attributable and distinct
  from the human owner.
- Do not add contribution-attribution trailers or footers to commits or PR
  descriptions: no `Co-Authored-By:` lines and no "Generated with …" footers.
  The bot account already identifies agent work; keep messages free of extra
  authorship attribution.

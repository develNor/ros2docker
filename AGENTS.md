# AGENTS.md

These instructions are for agents working in this repository. They add local
operating details; they do not replace the shared contributor workflow. Rules in
the project docs apply to agents the same way they apply to other developers.

This file is intentionally **generic and reusable**: it assumes only the minimum
needed to work sensibly. Project- and machine-specific identities, credentials,
and any permission choices the local user makes live in a gitignored local
config — see "Local instance configuration" at the end.

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
- A fresh worktree does not inherit the primary checkout's virtualenv, so the
  `pre-commit` git hook is not on `PATH`: `git commit` then aborts with
  "`pre-commit` not found" and a following `git push` can push a branch with no
  commits. Run `just setup` in the worktree, or put the repo's `.venv/bin` on
  `PATH`, before committing — and confirm the commit landed (`git log`) before
  pushing.
- Remove the worktree after the PR merges: `git worktree remove ../ros2docker-issue-<NN>`.
- Do not duplicate shared contributor policy here. Update the shared docs when a rule applies to every developer.

GitHub identity:

- GitHub writes (issues, PRs, comments, labels, commits, pushes, auto-merge, and
  Actions) are made through a **dedicated bot account that is separate from the
  human owner**, so agent work is natively attributable. Authenticate as the bot
  before any write.
- Do not add contribution-attribution trailers or footers to commits or PR
  descriptions: no `Co-Authored-By:` lines and no "Generated with …" footers. The
  bot account already identifies agent work.
- The concrete bot account, its credential handling, and any permission choices
  the local user makes — what may run under which identity, and what must be
  asked first — are **not part of this shared contract**. Assume only the minimum
  above and neither grant nor restrict further here; those choices belong to the
  local user and live in the local instance config below.

Local instance configuration:

- Project- and machine-specific agent details live in `.agents/agents.local.md`
  (the `.agents/` directory is gitignored). This keeps concrete account names,
  credential handling, and the local user's permission grants out of these
  reusable shared docs. **Read that file before acting.** Claude Code auto-loads
  it through the import below; an agent whose harness does not import should read
  it directly. `docs/agentic-workflow.md` documents the expected shape for anyone
  reusing this setup.
- `CLAUDE.md` is a symlink to this file so Claude Code — which auto-loads
  `CLAUDE.md`, not `AGENTS.md` — gets these instructions; Codex reads `AGENTS.md`
  directly. Keep the shared policy in `AGENTS.md` as the single source.

@.agents/agents.local.md

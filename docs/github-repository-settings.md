# GitHub Repository Settings

These settings live outside git and must be verified in GitHub. This document
records the expected contract for repository `develNor/ros2docker`; the GitHub
UI, CLI, and API remain the live source of truth.

## Branch And Ruleset Protection

Expected state:

- `main` is protected by a repository ruleset named `Protect main`.
- The ruleset target is the default branch.
- The ruleset enforcement is active.
- The ruleset requires pull requests.
- The ruleset allows squash merges.
- The ruleset prevents branch deletion.
- The ruleset prevents non-fast-forward updates.
- The ruleset requires the `ci-success` status check.
- The ruleset requires CodeQL code scanning.
- The ruleset requires review from Code Owners (see Code Owners below).
- The ruleset keeps `required_approving_review_count` at `0` (Code Owner review
  only gates PRs that touch owned paths; everything else can auto-merge on green).
- The ruleset bypass list contains the **Repository admin** role, so the owner is
  never blocked by these rules; the `develNor-agent` bot is not a bypass actor.

Verify:

```bash
gh ruleset list --repo develNor/ros2docker --parents
gh ruleset check main --repo develNor/ros2docker
gh ruleset view 17518226 --repo develNor/ros2docker
```

For full JSON/API inspection:

```bash
gh api repos/develNor/ros2docker/rulesets
gh api repos/develNor/ros2docker/rulesets/17518226
```

The expected required status check is:

```text
ci-success
```

See [docs/ci.md](ci.md) for the workflow contract behind that aggregate check.

## Agent Identity And Human-Only Gates

Agents operate as the dedicated bot account `develNor-agent`, separate from the
human owner. See [docs/agentic-workflow.md](agentic-workflow.md) for the model
and the rationale; this section records the settings that enforce it.

Expected state:

- **Bot collaborator.** `develNor-agent` is a collaborator with the **write**
  (`push`) role and no admin. On a personal (user-owned) repository a non-admin
  collaborator cannot change settings, rulesets, secrets, or delete/transfer the
  repository, so the entire admin tier is reserved to the owner automatically.
- **Bot token.** A **classic** PAT on `develNor-agent` with the `repo` and
  `workflow` scopes. A fine-grained PAT cannot authorize write on a repository
  owned by a different personal account, so it does not work here; a GitHub App
  installed on this repo is the granular, per-repo alternative if finer scoping
  is wanted later. Least privilege still holds because the bot is a non-admin
  collaborator: the token cannot reach repository settings, secrets, rulesets,
  `v*` tags, or the `pypi` environment regardless of its scopes.
- **Code Owners.** `.github/CODEOWNERS` assigns `@develNor` to the CI/test/deps
  and release policy surface (`.github/**`, `.pre-commit-config.yaml`,
  `pyproject.toml`, `justfile`, `codecov.yml`, `tests/contract/**`,
  `docs/release.md`, `docs/release-notes/**`). With **require review from Code
  Owners** on the ruleset, a bot PR touching those paths needs the owner's
  approval — which works only because the bot is a different account and cannot
  self-approve. PRs that touch nothing owned auto-merge with zero approvals.
- **Releases are owner-only.** A tag ruleset restricts creation of `v*` tags to
  the Repository admin role, and the `pypi` deployment environment lists
  `develNor` as a **required reviewer**, so the irreversible Trusted Publishing
  step waits for an explicit human approval even if a tag is created.
- **Fork pull requests on a public repository** still run with a read-only
  `GITHUB_TOKEN` and no secrets; the Actions *"require approval for outside/
  first-time contributors"* setting governs whether their workflows run.

Verify:

```bash
# Bot collaborator has push (not admin).
gh api repos/develNor/ros2docker/collaborators/develNor-agent/permission

# CODEOWNERS is valid and has no unknown owners.
gh api repos/develNor/ros2docker/codeowners/errors

# Ruleset: require_code_owner_review true, required_approving_review_count 0,
# bypass list includes the Repository admin role.
gh api repos/develNor/ros2docker/rulesets/17518226

# Tag protection ruleset for v* exists.
gh ruleset list --repo develNor/ros2docker --parents

# pypi environment has develNor as a required reviewer.
gh api repos/develNor/ros2docker/environments/pypi

# Actions approval policy for outside contributors.
gh api repos/develNor/ros2docker/actions/permissions/workflow
```

## CodeQL Default Setup

Expected state:

- CodeQL default setup is configured.
- Languages are `actions` and `python`.
- Query suite is `default`.
- Threat model is `remote`.
- Schedule is `weekly`.

Verify:

```bash
gh api repos/develNor/ros2docker/code-scanning/default-setup
```

## Repository Metadata

Expected state:

- Description: `Build and run ROS 2 Docker workspaces from versioned config files.`
- Homepage: `https://pypi.org/project/ros2docker/`
- Topics include `ros2`, `docker`, `robotics`, `colcon`, `ros`,
  `devcontainer`, and `docker-compose`.

Verify:

```bash
gh repo view develNor/ros2docker --json description,homepageUrl,repositoryTopics
```

Update:

```bash
gh repo edit develNor/ros2docker \
  --description "Build and run ROS 2 Docker workspaces from versioned config files." \
  --homepage "https://pypi.org/project/ros2docker/" \
  --add-topic ros2,docker,robotics,colcon,ros,devcontainer,docker-compose
```

## Labels

Expected labels for issue-driven work:

- `ready`: issue is accepted and ready for a contributor to pick up.
- `backlog`: issue is tracked for later and is not currently scheduled.
- `maybe`: issue is saved for later consideration; not yet accepted or ready.
- `documentation`: documentation-only or documentation-heavy work.
- `bug`: incorrect behavior.
- `enhancement`: new feature or request.
- `dependencies`: dependency updates.
- `github_actions`: GitHub Actions updates.
- `python`: Python dependency or source updates.

Verify:

```bash
gh label list --repo develNor/ros2docker --limit 200
```

Create the `ready` label if it is missing:

```bash
gh label create ready \
  --repo develNor/ros2docker \
  --description "Ready for a contributor to pick up" \
  --color 5319e7
```

Create the `backlog` label if it is missing:

```bash
gh label create backlog \
  --repo develNor/ros2docker \
  --description "Tracked for later and not currently scheduled" \
  --color cfd3d7
```

Create the `maybe` label if it is missing:

```bash
gh label create maybe \
  --repo develNor/ros2docker \
  --description "Saved for later consideration; not yet accepted or ready" \
  --color fbca04
```

## Drift Handling

If GitHub state differs from this document, treat it as drift. Either update the
live GitHub setting to match the documented contract, or update this document in
a PR if the repository policy intentionally changed.

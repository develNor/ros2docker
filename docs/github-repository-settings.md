# GitHub Repository Settings

These settings live outside git and must be verified in GitHub. This document
records the expected contract for repository `develNor/ros2docker`; the GitHub
UI, CLI, and API remain the live source of truth. See **Auditing this contract**
below for how to inspect live state and handle drift.

## Branch And Ruleset Protection

`main` is protected by a repository ruleset named `Protect main`:

- The target is the default branch and enforcement is active.
- It requires pull requests and allows squash merges only.
- It prevents branch deletion and non-fast-forward updates.
- It requires the `ci-success` status check and CodeQL code scanning.
- It requires review from Code Owners (see **Agent Identity And Owner-Only
  Gates**).
- It keeps `required_approving_review_count` at `0`, so Code Owner review only
  gates PRs that touch owned paths; everything else can auto-merge on green.
- Its bypass list contains the **Repository admin** role, so the owner is never
  blocked; the `develNor-agent` account is not a bypass actor.

`ci-success` is the single required status check. See [docs/ci.md](ci.md) for the
workflow contract behind that aggregate.

## Release Tag Protection

Stable release tags are protected by a repository ruleset named
`release protection`:

- The target is tags matching `refs/tags/v*` and enforcement is active.
- It protects tag creation, updates, deletion, and non-fast-forward updates.
- It requires the `ci-success` status check on the tagged commit, including when
  the tag is created.
- Its bypass list contains the **Repository admin** role.

Practical effect: only a repository admin can create, update, or delete stable
release tags, and the tag push succeeds only when the target commit already has
`ci-success`.

## Agent Identity And Owner-Only Gates

Agents operate as the dedicated account `develNor-agent`, separate from the
owner. The identity choice and its rationale are instance-specific and live in
the local instance config; this section records the settings that enforce the
gates.

- **Agent collaborator.** `develNor-agent` is a collaborator with the **write**
  (`push`) role and no admin. On a personal (user-owned) repository a non-admin
  collaborator cannot change settings, rulesets, secrets, or delete/transfer the
  repository, so the entire admin tier is reserved to the owner automatically.
- **Agent token.** A **classic** PAT on `develNor-agent` with the `repo` and
  `workflow` scopes. A fine-grained PAT cannot authorize write on a repository
  owned by a different personal account; a GitHub App installed on this repo is
  the granular, per-repo alternative if finer scoping is wanted later. Least
  privilege still holds because the agent account is a non-admin collaborator: the
  token cannot reach repository settings, secrets, rulesets, `v*` tags, or the
  `pypi` environment regardless of its scopes.
- **Code Owners.** `.github/CODEOWNERS` assigns `@develNor` to the CI/test/deps
  and release policy surface (`.github/**`, `.pre-commit-config.yaml`,
  `pyproject.toml`, `justfile`, `codecov.yml`, `tests/contract/**`,
  `docs/release.md`, `docs/release-notes/**`). With **require review from Code
  Owners** on the ruleset, an agent-account PR touching those paths needs the
  owner's approval — which works only because it is a different account and
  cannot self-approve. PRs that touch nothing owned auto-merge with zero
  approvals.
- **Releases are owner-only.** The `v*` tag ruleset restricts tag creation to the
  Repository admin role and requires `ci-success` on the tagged commit, and the
  `pypi` deployment environment lists `develNor` as a **required reviewer**, so
  the irreversible Trusted Publishing step waits for an explicit owner approval
  even if a tag is created.
- **Fork pull requests on a public repository** run with a read-only
  `GITHUB_TOKEN` and no secrets; the Actions *"require approval for outside/
  first-time contributors"* setting governs whether their workflows run.

## Repository Merge Settings

- Auto-merge is allowed.
- Squash merge is the only enabled merge method; merge commits and rebase merges
  are disabled.
- Branches are deleted automatically after merge.
- Update-branch is disabled.

## GitHub Actions Settings

- GitHub Actions is enabled; all actions are allowed and SHA pinning is not
  required.
- The default `GITHUB_TOKEN` workflow permission is read-only.
- Workflows cannot approve pull request reviews.
- Outside contributor workflow approval is required for first-time contributors.

## CodeQL Default Setup

- CodeQL default setup is configured.
- Languages are `actions` and `python`; query suite `default`; threat model
  `remote`; schedule `weekly`.

## Security And Secrets

- Dependabot security updates are enabled.
- Secret scanning is enabled; non-provider patterns, push protection, and
  validity checks are disabled.
- A `CODECOV_TOKEN` Actions secret exists for Codecov upload from the Python 3.12
  non-Docker check.
- No repository Actions variables are required.

## Repository Metadata

- Description: `Build and run ROS 2 Docker workspaces from versioned config files.`
- Homepage: `https://pypi.org/project/ros2docker/`
- Topics include `ros2`, `docker`, `robotics`, `colcon`, `ros`, `devcontainer`,
  and `docker-compose`.

## Labels

Labels for issue-driven work:

- `ready`: issue is accepted and ready for a contributor to pick up.
- `backlog`: issue is tracked for later and is not currently scheduled.
- `maybe`: issue is saved for later consideration; not yet accepted or ready.
- `documentation`: documentation-only or documentation-heavy work.
- `bug`: incorrect behavior.
- `enhancement`: new feature or request.
- `dependencies`: dependency updates.
- `github_actions`: GitHub Actions updates.
- `python`: Python dependency or source updates.

## Auditing this contract

GitHub is the live source of truth; this file is the expected contract. Inspect
or audit the live state with:

```bash
gh api repos/develNor/ros2docker/rulesets                       # branch + tag rulesets
gh api repos/develNor/ros2docker --jq '.security_and_analysis'  # security + secret scanning
gh api repos/develNor/ros2docker --jq \
  '{allow_auto_merge, allow_squash_merge, allow_merge_commit, allow_rebase_merge, delete_branch_on_merge, allow_update_branch}'
gh api repos/develNor/ros2docker/actions/permissions
gh api repos/develNor/ros2docker/code-scanning/default-setup
gh api repos/develNor/ros2docker/environments/pypi              # pypi required reviewer
gh api repos/develNor/ros2docker/collaborators/develNor-agent/permission
gh api repos/develNor/ros2docker/codeowners/errors
gh repo view develNor/ros2docker --json description,homepageUrl,repositoryTopics
gh label list --repo develNor/ros2docker --limit 200
gh secret list --repo develNor/ros2docker
```

If live state differs from this document, treat it as drift: either fix the
GitHub setting to match the contract, or update this document in a PR if the
policy changed intentionally.

# GitHub Repository Settings

These settings live outside git and must be verified in GitHub. This document
records the expected contract for this repository; the GitHub UI, CLI, and API
remain the live source of truth. See **Auditing this contract** below for how to
inspect live state and handle drift.

## Branch And Ruleset Protection

`main` is protected by a repository ruleset named `Protect main`:

- The target is the default branch and enforcement is active.
- It requires pull requests and allows squash merges only.
- It prevents branch deletion and non-fast-forward updates.
- It requires the `ci-success` status check and CodeQL code scanning.
- It requires review from code-owners (see **Access Tiers And Owner-Only
  Gates**).
- It keeps `required_approving_review_count` at `0`, so review by code-owners only
  gates PRs that touch owned paths; everything else can auto-merge on green.
- Its bypass list contains the **Repository admin** role, so the owner is never
  blocked; a non-admin collaborator is not a bypass actor.

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

## Access Tiers And Owner-Only Gates

Non-owners contribute as non-admin collaborators, separate from the owner. Who or
what drives a collaborator, and what credentials it uses, are instance-specific
and live in the local config; this section records the settings that enforce the
gates.

- **Non-admin collaborators.** Collaborators have the **write** (`push`) role and
  no admin. On a personal (user-owned) repository a non-admin collaborator cannot
  change settings, rulesets, secrets, or delete/transfer the repository, so the
  entire admin tier is reserved to the owner automatically.
- **Least privilege.** A non-admin collaborator's credentials cannot reach
  repository settings, secrets, rulesets, `v*` tags, or the `pypi` environment
  regardless of token scopes, so their blast radius is only the collaboration
  itself. The credential type is instance-specific and lives in the local config.
- **Code-owners review.** `.github/CODEOWNERS` assigns code-owners to the
  CI/test/deps and release policy surface (`.github/**`, `.pre-commit-config.yaml`,
  `pyproject.toml`, `justfile`, `codecov.yml`, `tests/contract/**`,
  `docs/release.md`, `docs/release-notes/**`). With **require review from Code
  Owners** on the ruleset, a PR touching those paths needs a review from
  code-owners; its author cannot supply that review unless they are a code-owner.
  PRs that touch nothing owned auto-merge with zero approvals.
- **Releases are owner-only.** The `v*` tag ruleset restricts tag creation to the
  Repository admin role and requires `ci-success` on the tagged commit, and the
  `pypi` deployment environment lists the owner as a **required reviewer**, so
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
REPO=<owner>/<repo>    # this repository
COLLAB=<collaborator>  # the non-admin collaborator whose access you check
gh api repos/$REPO/rulesets                       # branch + tag rulesets
gh api repos/$REPO --jq '.security_and_analysis'  # security + secret scanning
gh api repos/$REPO --jq \
  '{allow_auto_merge, allow_squash_merge, allow_merge_commit, allow_rebase_merge, delete_branch_on_merge, allow_update_branch}'
gh api repos/$REPO/actions/permissions
gh api repos/$REPO/code-scanning/default-setup
gh api repos/$REPO/environments/pypi              # pypi required reviewer
gh api repos/$REPO/collaborators/$COLLAB/permission
gh api repos/$REPO/codeowners/errors
gh repo view $REPO --json description,homepageUrl,repositoryTopics
gh label list --repo $REPO --limit 200
gh secret list --repo $REPO
```

If live state differs from this document, treat it as drift: either fix the
GitHub setting to match the contract, or update this document in a PR if the
policy changed intentionally.

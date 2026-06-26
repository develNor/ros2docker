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

## Code Owners And Contributor Rights

The owner-vs-contributor boundary for this repository is enforced by GitHub
access control, not by a CI job. The relevant settings:

- **Code Owners.** `.github/CODEOWNERS` (in git) assigns `@develNor` as the
  required reviewer for the files that define automation: `.github/**`,
  `pyproject.toml`, `justfile`, and `codecov.yml`. For this to be enforced, the
  `Protect main` ruleset must have **require review from Code Owners** enabled.
  Together they mean a contributor — or an agent — cannot change CI, build
  tooling, packaging, or the release pipeline without explicit owner approval,
  even though `ci-success` would otherwise be green.

- **Fork pull requests on a public repository.** Outside collaborators' fork PRs
  already run with a **read-only `GITHUB_TOKEN`** and **no access to repository
  secrets**, so contributor PRs cannot exfiltrate secrets or self-approve. The
  Actions setting *"Require approval for all external contributors"* (or
  first-time contributors) controls whether their workflows run at all.

- **No write access for contributors.** `main` is protected (PRs required,
  `ci-success` + CodeQL required, squash-only, no force-push, no deletion), so
  contributors cannot push to `main` or merge without passing the gate and any
  required Code Owner review.

Verify:

```bash
# CODEOWNERS is valid and has no unknown owners.
gh api repos/develNor/ros2docker/codeowners/errors

# Ruleset requires Code Owner review (look for required_review with
# require_code_owner_review: true under the pull_request rule).
gh api repos/develNor/ros2docker/rulesets/17518226

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
  --description "Saved for later consideration; not yet accepted or agent-ready" \
  --color fbca04
```

## Drift Handling

If GitHub state differs from this document, treat it as drift. Either update the
live GitHub setting to match the documented contract, or update this document in
a PR if the repository policy intentionally changed.

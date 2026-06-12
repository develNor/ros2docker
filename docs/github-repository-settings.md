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

- `agent-ready`: issue is ready for an agent or contributor to pick up.
- `backlog`: issue is tracked for later and is not currently scheduled.
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

Create the `agent-ready` label if it is missing:

```bash
gh label create agent-ready \
  --repo develNor/ros2docker \
  --description "Ready for an agent or contributor to pick up" \
  --color 5319e7
```

Create the `backlog` label if it is missing:

```bash
gh label create backlog \
  --repo develNor/ros2docker \
  --description "Tracked for later and not currently scheduled" \
  --color cfd3d7
```

## Drift Handling

If GitHub state differs from this document, treat it as drift. Either update the
live GitHub setting to match the documented contract, or update this document in
a PR if the repository policy intentionally changed.

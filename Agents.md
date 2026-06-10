# AGENTS.md

## Project

`ros2docker` is a Python CLI and API for building and running ROS 2 Docker workspaces from JSON-with-comments config files.

## Read first

- `README.md` for user-facing behavior.
- `DEVELOPMENT_PRINCIPLES.md` for project rules.
- `tests/test_docs.py` before changing README, examples or config keys

## Standard commands

Use these commands from the repository root:

- `just setup`
- `just lint`
- `just test-unit`
- `just docs`
- `just test-e2e-fast`
- `just test-e2e-slow`
- `just check`

## Required workflow

1. Make the smallest coherent change.
2. Update tests for changed behavior.
3. Update docs if CLI, config, Docker behavior, examples, or public API changed.
4. Run `just check`.
5. If Docker behavior changed, also run `make test-e2e-fast`.
6. Include command results in the PR summary.

## Hard constraints

- Do not push to `main`.
- Do not add compatibility layers unless explicitly requested.
- Do not add runtime dependencies without justification.
- Do not update E2E expectations or snapshots without explaining the behavior change.
# AGENTS.md

Read first:

- `README.md` for user-facing behavior.
- `CONTRIBUTING.md` for setup, checks, CI, PR workflow, merge policy, and releases.
- `DEVELOPMENT_PRINCIPLES.md` for quality rules.
- `docs/playbooks/` for reusable task recipes when a request matches one.
- `tests/contract/test_public_config_surface.py` before changing README config examples, packaged examples, schemas, or config keys.

Operational rules:

- Follow `CONTRIBUTING.md` for branch setup, local setup, check selection, CI, PR workflow, merge policy, and releases.
- Start every change from an up-to-date `origin/main` topic branch. Do not push directly to `main`.
- Update README, docs, examples, schemas, and tests when CLI, config, API, Docker, or public behavior changes.
- Do not skip, weaken, or delete tests/CI to make a change pass.
- Default to a draft review PR with no auto-merge. Use autonomous auto-merge only when explicitly requested.
- After opening or updating a PR, check PR/CI status and print the PR URL.

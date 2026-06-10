# CI

This repository uses GitHub Actions.

## Workflows

`CI` runs automatically on pull requests, pushes to `main`, and manual dispatch.
It checks:

- Ruff lint and format
- Python byte compilation
- Mypy
- Unit tests
- Contract tests
- Package build, metadata validation, and wheel-content validation

`Docker E2E` is manual because it builds and runs ROS Docker images. Use it when
Docker behavior, ROS runtime behavior, or build resources change. The `fast`
suite matches `just test-e2e-fast`; the `all` suite also runs slow tests.

## Activate On GitHub

1. Push this workflow config to GitHub.
2. Open `https://github.com/develNor/ros2docker`.
3. Go to the `Actions` tab.
4. If GitHub shows an enable prompt, choose `I understand my workflows, go ahead and enable them`.
5. Open the `CI` workflow and confirm it ran for the pushed branch or pull request.
6. To run Docker checks, open `Actions`, choose `Docker E2E`, click `Run workflow`, select `fast` or `all`, then start it.

No repository secrets are required for these workflows.

## Recommended Branch Protection

After `CI` has run once, make it required for `main`:

1. Open repository `Settings`.
2. Go to `Branches`.
3. Add or edit the branch protection rule for `main`.
4. Enable `Require status checks to pass before merging`.
5. Select these required checks:
   - `Python 3.10`
   - `Python 3.12`
   - `Package`
6. Enable `Require branches to be up to date before merging` if you want PRs
   rebased or merged with the latest `main` before merge.

Keep `Docker E2E` manual unless you are comfortable with slower and more
network-sensitive PR checks.

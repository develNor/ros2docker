Goal: Ship a ros2docker release PR.

Tasks:
- Start from fresh origin/main topic branch.
- Update Dockerfile base image digest.
- Update pinned external tool versions:
  - ZENOH_VERSION
  - ZENOH_ROS2DDS_VERSION
  - MCAP_CLI_VERSION
- Update checksum verification where already specified.
- Run Docker-relevant checks:
  - just lint
  - just typecheck
  - just test-unit
  - just test-contract
  - just docs
  - just package
  - just test-e2e-fast
- Update docs/tests if public Docker behavior changed.
- Open autonomous PR and report the PR URL/CI status.
- Do tag and publish

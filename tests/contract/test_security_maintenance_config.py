from __future__ import annotations

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DEPENDABOT_PATH = PACKAGE_ROOT / ".github" / "dependabot.yml"
IMAGE_SCAN_PATH = PACKAGE_ROOT / ".github" / "workflows" / "image-scan.yml"
MERGE_GATE_PATH = PACKAGE_ROOT / ".github" / "workflows" / "pr-merge-gate.yml"
CODEOWNERS_PATH = PACKAGE_ROOT / ".github" / "CODEOWNERS"

CI_SUCCESS_NEEDS = "needs: [static-checks, tests, package, fast-e2e, workflow-lint, dependency-review]"


def test_dependabot_groups_weekly_actions_and_python_updates() -> None:
    dependabot = DEPENDABOT_PATH.read_text(encoding="utf-8")

    assert 'package-ecosystem: "github-actions"' in dependabot
    assert 'package-ecosystem: "pip"' in dependabot
    assert dependabot.count('interval: "weekly"') == 2
    assert "github-actions:" in dependabot
    assert "python-dependencies:" in dependabot
    assert "auto-merge" not in dependabot.lower()


def test_image_scan_is_advisory_and_not_a_pr_check() -> None:
    image_scan = IMAGE_SCAN_PATH.read_text(encoding="utf-8")
    merge_gate = MERGE_GATE_PATH.read_text(encoding="utf-8")

    assert "name: image-scan" in image_scan
    assert '- cron: "23 3 * * *"' in image_scan
    assert "workflow_dispatch:" in image_scan
    assert "pull_request:" not in image_scan
    assert "permissions:\n  contents: read" in image_scan
    assert "IMAGE_TAG: ros2docker:scan-${{ github.sha }}" in image_scan
    assert ".venv/bin/ros2docker build" in image_scan
    assert "aquasecurity/trivy-action@v0.36.0" in image_scan
    assert 'exit-code: "0"' in image_scan
    assert "actions/upload-artifact@v7" in image_scan
    assert "image-scan" not in merge_gate
    assert "trivy" not in merge_gate.lower()


def test_pr_merge_gate_runs_required_dependency_review() -> None:
    merge_gate = MERGE_GATE_PATH.read_text(encoding="utf-8")

    assert "dependency-review:" in merge_gate
    assert "actions/dependency-review-action@v5" in merge_gate
    assert "fail-on-severity: high" in merge_gate
    assert "fail-on-scopes: runtime, development" in merge_gate
    # Wired into the required aggregate gate, not a standalone advisory check.
    assert CI_SUCCESS_NEEDS in merge_gate


def test_pr_merge_gate_runs_required_workflow_lint() -> None:
    merge_gate = MERGE_GATE_PATH.read_text(encoding="utf-8")

    assert "workflow-lint:" in merge_gate
    # actionlint is pinned by version and checksum, matching the pre-commit hook.
    assert "ACTIONLINT_VERSION: 1.7.12" in merge_gate
    assert "ACTIONLINT_SHA256: 8aca8db96f1b94770f1b0d72b6dddcb1ebb8123cb3712530b08cc387b349a3d8" in merge_gate
    assert "actionlint -color" in merge_gate
    # Part of the required aggregate gate.
    assert CI_SUCCESS_NEEDS in merge_gate


def test_required_merge_gate_jobs_do_not_mask_failures() -> None:
    merge_gate = MERGE_GATE_PATH.read_text(encoding="utf-8")

    # Required gates must fail loudly: no advisory escape hatches or masking.
    assert "continue-on-error: true" not in merge_gate
    assert "|| true" not in merge_gate


def test_codeowners_protects_ci_definition() -> None:
    codeowners = CODEOWNERS_PATH.read_text(encoding="utf-8")

    # CI/build/packaging changes require maintainer review (owner-vs-contributor
    # boundary), enforced together with the "require Code Owner review" ruleset.
    assert "/.github/" in codeowners
    assert "/pyproject.toml" in codeowners
    assert "@develNor" in codeowners

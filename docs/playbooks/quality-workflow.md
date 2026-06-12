You are executing a multi-goal repository quality workflow.

Repository rules:
- Follow AGENTS.md.
- Use the playbooks in docs/playbooks/.
- Create one focused autonomous PR per goal.
- Do not combine unrelated goals into one PR.
- Start every goal from latest main.
- Do not start a dependent goal until prerequisite PRs are merged.
- If CI fails, fix the current PR instead of starting a new goal.
- If a PR cannot be merged automatically, stop and report the blocker.
- Do not weaken tests, remove checks, or hide failures with broad skips/xfails.
- Keep each PR small and reviewable.

Execution plan:
1. Run docs/playbooks/test-ci-audit.md and create a PR.
2. Run docs/playbooks/documentation-audit.md and create a PR.
3. After both are merged into main, run docs/playbooks/implementation-cleanup.md and create a PR.
4. After the cleanup PR is created, run docs/playbooks/maintainer-review.md as a review-only task.

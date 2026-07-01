# Owner Runbook

`ros2docker` is issue-driven and largely agent-built (see
[agentic-workflow.md](agentic-workflow.md)). As the **owner** you drive it with a
*handful of prompts* — you should never open an issue template yourself.
Everything under [`.github/ISSUE_TEMPLATE/`](../.github/ISSUE_TEMPLATE/) and the
rest of `docs/` is the **agent layer**: material agents read while executing the
prompts below.

## Quality maintenance pass

Run anytime, and as the first step before a release. Prompt the agent:

> Run the repository quality workflow autonomously: first run the
> repository-diagnosis pass, then create and complete the focused PRs it triages
> (audits, cleanup, or redesign) in priority order — each started from a fresh
> `origin/main`, opened ready with auto-merge. If the diagnosis finds nothing
> actionable, report that and stop. Report each PR URL and CI status.

The pass diagnoses first and may find nothing — a healthy repo yields zero PRs,
which is success. It runs as several small, focused PRs rather than one big pass
(see [quality-model.md](quality-model.md)). **Your part:** approve the Code Owner
review on any PR that touches an owned path — the bot cannot self-approve those.

## Cut a release

Follow the [release runbook](release.md). Its first step is the quality pass
above; it then prepares the release PR and publishes the version.

## What only you can do (the three gates)

Agents act as a non-admin bot, so three steps are yours alone: Code Owner
approvals on owned paths, creating the `vX.Y.Z` tag, and approving the `pypi`
deployment. See [agentic-workflow.md](agentic-workflow.md).

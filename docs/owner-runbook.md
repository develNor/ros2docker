# Owner Runbook

`ros2docker` is issue-driven and largely agent-built. The **owner** can drive it
with a *handful of prompts* and usually leaves opening issue templates to the
agents. Everything under [`.github/ISSUE_TEMPLATE/`](../.github/ISSUE_TEMPLATE/)
and the rest of `docs/` is the **agent layer**: material agents read while
executing the prompts below.

## Quality maintenance pass

Run anytime, and as the first step before a release. Prompt an agent:

> Run the repository quality workflow autonomously: first run the
> repository-diagnosis pass, then create and complete the focused PRs it triages
> (audits, cleanup, or redesign) in priority order — each started from a fresh
> `origin/main`, opened ready with auto-merge. If the diagnosis finds nothing
> actionable, report that and stop. Report each PR URL and CI status.

The pass diagnoses first and may find nothing — a healthy repo yields zero PRs,
which is success. It runs as several small, focused PRs rather than one big pass
(see [quality-model.md](quality-model.md)). A PR that touches an owned path needs
a review from code-owners that its author cannot self-approve.

## Cut a release

Follow the [release runbook](release.md). Its first step is the quality pass
above; it then prepares the release PR and publishes the version.

## What only the owner can do (the three gates)

Because non-owners act as non-admin collaborators, three steps stay with the
owner: code-owners review on owned paths, creating the `vX.Y.Z` tag, and approving
the `pypi` deployment. See
[github-repository-settings.md](github-repository-settings.md) for the settings
that enforce them.

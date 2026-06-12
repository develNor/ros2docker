Goal: Make public-facing documentation consistent, non-redundant, and aligned with the repository.

Scope:

* README
* CONTRIBUTING
* DEVELOPMENT / development principles
* AGENTS.md
* docs/
* package metadata if it contains public-facing descriptions

Document ownership:

* README: user-facing purpose, installation, quick start, common usage
* CONTRIBUTING: human contribution workflow, PR expectations, local checks
* DEVELOPMENT: technical setup, architecture, testing, release process
* AGENTS.md: agent-specific operating rules only; link to human docs instead of duplicating them

Tasks:

* Find duplicated, stale, contradictory, or misplaced information.
* Prefer one canonical location plus links instead of repeated prose.
* Ensure documented workflows match actual repo configuration.
* Ensure agent instructions do not diverge from human development instructions.
* Keep AGENTS.md short and focused on agent behavior.
* Do not change source code unless needed to correct a documented command.

Report:

* final responsibility of each public-facing document
* moved or removed duplicated content
* remaining intentional duplication
* any documented behavior that still lacks implementation or tests

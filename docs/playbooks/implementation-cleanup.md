Goal: Improve implementation quality while preserving the current public contract.

Scope:
- source code
- tests only where needed to preserve or clarify behavior
- no documentation changes except small corrections caused by code cleanup

Rules:
- Do not change documented CLI/API behavior unless tests and docs prove the old behavior was stale or broken.
- Do not remove tests.
- Do not weaken assertions.
- Do not add compatibility layers unless strictly required.
- Prefer clean removal of obsolete/dead code.
- Prefer the simplest implementation that satisfies the tests.
- Avoid broad rewrites.

Check for:
- dead code
- unused imports/functions/classes/constants
- redundant branches
- duplicated logic
- over-abstracted helpers
- unclear names
- unnecessary compatibility paths
- unnecessarily complicated control flow
- tests that no longer test meaningful behavior

After changes:
- run formatter/linter
- run unit tests
- run fast e2e tests if configured
- summarize simplifications and test coverage

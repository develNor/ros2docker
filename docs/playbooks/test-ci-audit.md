Goal: Audit and improve test coverage and test execution wiring.

Scope:

* tests/
* pyproject.toml or equivalent test config
* .pre-commit-config.yaml
* .github/workflows/*
* README/CONTRIBUTING/DEVELOPMENT sections that describe testing
* minimal source changes only if required to make behavior testable

Tasks:

* Map every public README-documented command, feature, and CLI behavior to existing tests where feasible.
* Add missing tests for feasible documented behavior.
* Identify behavior that cannot sensibly be tested automatically and document a manual verification check.
* Verify that tests are collected and actually executed.
* Check that tests are not accidentally skipped, deselected, ignored, or excluded by config.
* Ensure skipped/xfail tests have explicit reasons.
* Verify that local test instructions, pre-commit, CI, and release validation are consistent.
* Do not refactor implementation except where needed to make tests possible.

Run the relevant checks and report:

* changed files
* added or fixed tests
* remaining untested behavior and why
* commands run and results

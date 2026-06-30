# Development Principles

## Definition of Done

A change is complete only if:

1. Public behavior is implemented.
2. Unit tests or contract tests cover the behavior.
3. Docker/E2E tests are added or updated when Docker runtime behavior changes.
4. README and examples are updated when CLI, config, API, or Docker behavior changes.
5. `tests/contract/test_public_config_surface.py` passes.
6. `just check` passes.
7. Obsolete behavior is removed unless compatibility is explicitly requested.

## Compatibility Policy

This project prefers clean migrations.

Do not preserve legacy behavior unless the issue explicitly asks for it.
Do not add silent aliases for old config keys.
Do not keep duplicate implementations.

## Testing Policy

Test public behavior and contracts.

Prefer:
- CLI dry-run output tests,
- config validation tests,
- command rendering tests,
- docs/config-surface tests,
- Docker smoke tests for runtime behavior.

Avoid:
- tests of private helper implementation details,
- brittle exact formatting assertions unless formatting is the contract,
- excessive mocks around pure command rendering,
- tests that duplicate production logic.

## Docker Policy

Dockerfile changes require at least one fast E2E smoke test or an explanation why none is needed.

Downloaded external binaries should be versioned and, where practical, checksum-verified.
Long/fragile ROS integration checks belong in nightly CI, not mandatory PR CI.

## Dependency Policy

New dependencies require:
- why the dependency is needed,
- why stdlib or existing dependencies are insufficient,
- whether it is runtime or dev-only,
- license/security consideration.

## Quality and Review

- Public-facing claims must be backed by code, tests, or an explicit manual
  verification note; if a behavior cannot sensibly be tested automatically,
  document the manual check.
- Prefer the simplest implementation that satisfies the tests; justify any
  abstraction.
- For every non-trivial change, record what changed, why it is safe, and which
  tests or checks cover it.

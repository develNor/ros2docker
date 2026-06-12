# Release

Releases are built by `.github/workflows/release.yml`.

## Version Source

Package versions come from Git tags through `setuptools-scm`. The project does
not keep a static version in `pyproject.toml` or `src/ros2docker/__init__.py`.

## Tags

Stable releases use tags in this form:

```text
vX.Y.Z
```

Push the tag from the release commit after the release PR has merged.

## Release Notes

Each stable release must include a release notes file matching the tag:

```text
docs/release-notes/vX.Y.Z.md
```

Start from [docs/release-notes/TEMPLATE.md](release-notes/TEMPLATE.md). The
release notes should summarize the diff from the previous release tag to the
new tag, including user-facing CLI, config, API, Docker, dependency, packaging,
compatibility, and migration changes.

The GitHub Release job fails if the tag-specific release notes file is missing.
When present, that file becomes the GitHub Release description.

## Validation

The release workflow runs non-Docker checks on Python 3.10 and 3.12, validates
package artifacts, verifies tag-specific release notes, and runs fast Docker E2E
before publishing:

```bash
just lint
just typecheck
just test-unit
just test-contract
just docs
just package
just test-e2e-fast
```

## Publishing

Tag pushes publish to PyPI through Trusted Publishing and create a GitHub
Release with the release notes, wheel, source distribution, and `SHA256SUMS`.

Manual workflow dispatch publishes only to TestPyPI. Use it for release
rehearsals before cutting a PyPI tag.

Trusted Publishers must be configured for repository `develNor/ros2docker`,
workflow `.github/workflows/release.yml`, and environments `pypi` and
`testpypi`.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the canonical maintainer release
workflow.

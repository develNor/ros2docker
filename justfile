python := ".venv/bin/python"
bin := ".venv/bin"

default:
	@just --list

venv:
	python3 -m venv .venv
	{{python}} -m pip install --upgrade pip

setup: venv
	{{python}} -m pip install -e ".[dev]"
	{{python}} -m pre_commit install

format:
	{{python}} -m ruff check --fix .
	{{python}} -m ruff format .

lint:
	{{python}} -m ruff check .
	{{python}} -m ruff format --check .
	{{python}} -m py_compile src/ros2docker/*.py

typecheck:
	{{python}} -m mypy

# Lint GitHub Actions workflows with actionlint (via the pinned pre-commit
# docker hook, which bundles shellcheck). Mirrors the workflow-lint CI job.
lint-workflows:
	{{python}} -m pre_commit run actionlint-docker --all-files

test: test-unit test-contract

test-unit:
	{{python}} -m pytest -q tests/unit

test-contract:
	{{python}} -m pytest -q tests/contract

coverage: test-nondocker-cov

test-nondocker-cov:
	{{python}} -m pytest -q tests/unit tests/contract --cov=ros2docker --cov-report=term-missing --cov-report=xml:coverage.xml

test-e2e-fast:
	ROS2DOCKER_RUN_E2E=1 {{python}} -m pytest -q tests/e2e -m "e2e and not slow"

test-e2e-slow:
	ROS2DOCKER_RUN_E2E=1 ROS2DOCKER_RUN_SLOW_E2E=1 {{python}} -m pytest -q tests/e2e -m e2e

docs:
	{{python}} -m pytest -q tests/contract/test_public_config_surface.py tests/contract/test_readme_examples.py tests/contract/test_markdown_links.py

package: _package-build _package-smoke

_package-build:
	rm -rf build dist src/*.egg-info
	{{python}} -m build
	{{python}} -m twine check dist/*
	{{bin}}/check-wheel-contents dist/*.whl

_package-smoke:
	#!/usr/bin/env bash
	set -euo pipefail

	tmpdir="$(mktemp -d)"
	trap 'rm -rf "$tmpdir"' EXIT

	"{{python}}" -m venv "$tmpdir/venv"
	"$tmpdir/venv/bin/python" -m pip install dist/*.whl
	"$tmpdir/venv/bin/ros2docker" --version
	"$tmpdir/venv/bin/python" -m ros2docker --version
	"$tmpdir/venv/bin/ros2docker" run --no-build --dry-run -m .
	"$tmpdir/venv/bin/python" - <<'PY'
	from importlib import resources

	expected = (
	    "py.typed",
	    "resources/build/Dockerfile.generic",
	    "resources/build/entrypoint.sh",
	    "resources/examples/ros2docker.json",
	    "resources/schema/ros2docker.schema.json",
	    "resources/profiles/minimal.json",
	    "resources/profiles/desktop.json",
	    "resources/profiles/foxglove.json",
	    "resources/profiles/zenoh.json",
	    "resources/profiles/mcap.json",
	    "resources/profiles/novatel.json",
	    "resources/profiles/project-develnor.json",
	)
	missing = [
	    path
	    for path in expected
	    if not resources.files("ros2docker").joinpath(path).is_file()
	]
	if missing:
	    raise SystemExit(f"missing packaged resources: {', '.join(missing)}")
	PY

docker-build:
	{{bin}}/ros2docker build --dry-run

pre-commit:
	{{python}} -m pre_commit run --all-files

check: lint typecheck test-nondocker-cov docs package

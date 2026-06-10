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

test: test-unit

test-unit:
	{{python}} -m pytest -q tests -m "not e2e"

coverage:
	{{python}} -m pytest -q tests -m "not e2e" --cov=ros2docker --cov-report=term-missing

test-e2e-fast:
	ROS2DOCKER_RUN_E2E=1 {{python}} -m pytest -q tests/e2e -m "e2e and not slow"

test-e2e-slow:
	ROS2DOCKER_RUN_E2E=1 ROS2DOCKER_RUN_SLOW_E2E=1 {{python}} -m pytest -q tests/e2e -m e2e

docs:
	{{python}} -m pytest -q tests/test_docs.py

package:
	rm -rf build dist src/*.egg-info
	{{python}} -m build
	{{python}} -m twine check dist/*
	{{bin}}/check-wheel-contents dist/*.whl

docker-build:
	{{bin}}/ros2docker build --dry-run

pre-commit:
	{{python}} -m pre_commit run --all-files

check: lint typecheck test-unit docs package

.PHONY: setup lint format test test-unit test-e2e-fast test-e2e-slow docs docker-build check

setup:
	python3 -m pip install -e ".[dev]"

format:
	ruff format .

lint:
	ruff check .
	python3 -m py_compile src/ros2docker/*.py

test-unit:
	python3 -m pytest -q tests -m "not e2e"

test-e2e-fast:
	ROS2DOCKER_RUN_E2E=1 python3 -m pytest -q tests/e2e -m "e2e and not slow"

test-e2e-slow:
	ROS2DOCKER_RUN_E2E=1 ROS2DOCKER_RUN_SLOW_E2E=1 python3 -m pytest -q tests/e2e -m e2e

docs:
	python3 -m pytest -q tests/test_docs.py

docker-build:
	ros2docker build --dry-run

check: lint test-unit docs
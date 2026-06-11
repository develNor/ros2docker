"""ros2docker package."""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

_DISTRIBUTION_NAME = "ros2docker"


def _package_version() -> str:
    try:
        return version(_DISTRIBUTION_NAME)
    except PackageNotFoundError:
        return "0+unknown"


__version__ = _package_version()

__all__ = ["__version__"]

from __future__ import annotations

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def test_package_version_is_derived_from_scm_metadata() -> None:
    pyproject = (PACKAGE_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    package_init = (PACKAGE_ROOT / "src" / "ros2docker" / "__init__.py").read_text(encoding="utf-8")

    assert '\nversion = "0.1.0"' not in pyproject
    assert 'dynamic = ["version"]' in pyproject
    assert "setuptools-scm" in pyproject
    assert '__version__ = "0.1.0"' not in package_init

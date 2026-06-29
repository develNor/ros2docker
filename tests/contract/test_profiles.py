"""Contract tests for the packaged image profiles.

These guard the "nothing previously installed is lost" property: the historical
full image is reproduced by the ``project-develnor`` profile (Dockerfile.generic
with every add-on flag enabled), and the modular add-on profiles declare the
packages/flags they advertise.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from ros2docker.config import load_config, load_profile

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
PROFILES_DIR = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "profiles"
DOCKERFILE_GENERIC_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "Dockerfile.generic"
ARG_RE = re.compile(r"^\s*ARG\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)", re.MULTILINE)

BASE_PROFILES = ("minimal", "desktop")
ADDON_PROFILES = ("foxglove", "zenoh", "mcap", "novatel")
ALL_PROFILES = (*BASE_PROFILES, *ADDON_PROFILES, "project-develnor")

PROFILE_PATHS = sorted(PROFILES_DIR.glob("*.json"))


def _declared_args() -> set[str]:
    return set(ARG_RE.findall(DOCKERFILE_GENERIC_PATH.read_text(encoding="utf-8")))


@pytest.mark.parametrize("name", ALL_PROFILES)
def test_every_profile_loads_and_validates(name: str) -> None:
    # Loading a config that selects the profile runs it through schema validation.
    config = load_config(override={"profile": name})
    assert config["profile"] == name
    assert config["dockerfile"] == "Dockerfile.generic"


@pytest.mark.parametrize("name", ADDON_PROFILES)
def test_addon_profiles_are_composable(name: str) -> None:
    profile = load_profile(name)
    # Add-on profiles do not pin a base image or image name so they compose on
    # top of a base profile.
    assert "image_name" not in profile
    build_args = profile.get("build_args", {})
    assert "BASE_IMAGE" not in build_args


def test_foxglove_profile_declares_characteristic_packages() -> None:
    apt = load_profile("foxglove")["build_args"]["APT_PACKAGES"].split()
    assert "ros-lyrical-foxglove-bridge" in apt
    assert "ros-lyrical-gps-msgs" in apt


def test_zenoh_profile_installs_router_and_rmw() -> None:
    build_args = load_profile("zenoh")["build_args"]
    assert build_args["INSTALL_ZENOH"] == "1"
    assert "ros-lyrical-rmw-zenoh-cpp" in build_args["APT_PACKAGES"].split()


def test_mcap_profile_enables_mcap_flag() -> None:
    assert load_profile("mcap")["build_args"]["INSTALL_MCAP"] == "1"


def test_novatel_profile_enables_novatel_flag() -> None:
    assert load_profile("novatel")["build_args"]["INSTALL_NOVATEL"] == "1"


def test_project_develnor_reproduces_the_full_historical_image() -> None:
    build_args = load_config(override={"profile": "project-develnor"})["build_args"]

    # All optional add-ons are enabled.
    assert build_args["INSTALL_ZENOH"] == "1"
    assert build_args["INSTALL_MCAP"] == "1"
    assert build_args["INSTALL_NOVATEL"] == "1"
    # Cyclone DDS remains the baked-in default RMW.
    assert build_args["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    # The desktop-full base image is pinned by digest.
    assert build_args["BASE_IMAGE"] == "osrf/ros:lyrical-desktop-full-resolute"
    assert build_args["DIGEST"].startswith("@sha256:")

    apt = build_args["APT_PACKAGES"].split()
    for package in (
        "ros-lyrical-foxglove-bridge",
        "ros-lyrical-foxglove-msgs",
        "ros-lyrical-gps-msgs",
        "ros-lyrical-rmw-cyclonedds-cpp",
        "ros-lyrical-rmw-zenoh-cpp",
    ):
        assert package in apt


def test_project_develnor_matches_composed_addons() -> None:
    composed = load_config(
        override={"profile": ["desktop", "foxglove", "zenoh", "mcap", "novatel"]},
    )["build_args"]
    develnor = load_config(override={"profile": "project-develnor"})["build_args"]

    for flag in ("INSTALL_ZENOH", "INSTALL_MCAP", "INSTALL_NOVATEL"):
        assert composed[flag] == develnor[flag] == "1"
    # The composed add-ons cover the same characteristic apt packages.
    assert set(develnor["APT_PACKAGES"].split()) <= set(composed["APT_PACKAGES"].split())


@pytest.mark.parametrize("profile_path", PROFILE_PATHS, ids=lambda p: p.stem)
def test_profile_build_args_are_declared_dockerfile_args(profile_path: Path) -> None:
    # `docker build --build-arg FOO=1` is silently ignored when no `ARG FOO`
    # exists, so a typo'd build-arg key (e.g. INSTALL_ZENO for INSTALL_ZENOH)
    # would install nothing with zero signal. Cross-link the two sides here.
    declared = _declared_args()
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    build_args = profile.get("build_args", {})

    undeclared = sorted(key for key in build_args if key not in declared)
    assert not undeclared, (
        f"{profile_path.name} sets build_args not declared as ARG in "
        f"Dockerfile.generic: {undeclared}. Add the ARG or fix the typo."
    )


def test_profiles_and_args_are_discovered() -> None:
    # Guard against the glob or the ARG parser silently matching nothing, which
    # would make the cross-link test vacuously pass.
    assert PROFILE_PATHS, f"no profile JSON files found in {PROFILES_DIR}"
    assert _declared_args(), "no ARG declarations parsed from Dockerfile.generic"

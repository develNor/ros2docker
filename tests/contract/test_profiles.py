"""Contract tests for the packaged image profiles.

These guard the "nothing previously installed is lost" property: the historical
full image is reproduced by the ``project-develnor`` profile (Dockerfile.generic
with every add-on flag enabled), and the modular add-on profiles declare the
packages/flags they advertise.
"""

from __future__ import annotations

import pytest

from ros2docker.config import load_config, load_profile

BASE_PROFILES = ("minimal", "desktop")
ADDON_PROFILES = ("foxglove", "zenoh", "mcap", "novatel")
ALL_PROFILES = (*BASE_PROFILES, *ADDON_PROFILES, "project-develnor")


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

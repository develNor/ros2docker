from __future__ import annotations

import re
import subprocess
from importlib import resources
from pathlib import Path

from ros2docker.api import build_context

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE_GENERIC_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "Dockerfile.generic"
DOCKERFILE_FULL_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "Dockerfile.full-example"
ENTRYPOINT_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "entrypoint.sh"


def test_packaged_resources_include_typed_marker_build_schema_example_and_bake_context(tmp_path: Path) -> None:
    assert resources.files("ros2docker").joinpath("py.typed").is_file()

    package_resources = resources.files("ros2docker").joinpath("resources")

    assert package_resources.joinpath("build", "Dockerfile.generic").is_file()
    assert package_resources.joinpath("build", "Dockerfile.full-example").is_file()
    assert package_resources.joinpath("build", "entrypoint.sh").is_file()
    assert package_resources.joinpath("examples", "ros2docker.json").is_file()
    assert package_resources.joinpath("schema", "ros2docker.schema.json").is_file()

    for p in ("minimal", "desktop", "foxglove", "zenoh", "project-develnor"):
        assert package_resources.joinpath("profiles", f"{p}.json").is_file()

    config_path = tmp_path / "ros2docker.json"
    config_path.write_text("{}", encoding="utf-8")
    with build_context(config_path) as context_dir:
        assert (context_dir / "Dockerfile").is_file()
        assert (context_dir / "entrypoint.sh").is_file()
        assert (context_dir / "bake_packages").is_dir()


def test_dockerfile_default_base_image_and_digest_are_explicit() -> None:
    for path in (DOCKERFILE_GENERIC_PATH, DOCKERFILE_FULL_PATH):
        dockerfile = path.read_text(encoding="utf-8")
        base_image = re.search(r"^ARG BASE_IMAGE=(?P<value>\S+)$", dockerfile, flags=re.MULTILINE)
        digest = re.search(r"^ARG DIGEST=(?P<value>@sha256:[0-9a-f]{64})$", dockerfile, flags=re.MULTILINE)

        assert base_image is not None
        assert base_image.group("value")
        assert digest is not None


def test_dockerfile_external_downloads_use_versioned_urls() -> None:
    dockerfile = DOCKERFILE_FULL_PATH.read_text(encoding="utf-8")
    active_lines = "\n".join(line for line in dockerfile.splitlines() if not line.lstrip().startswith("#"))
    urls = re.findall(r"https?://[^\s\"']+", active_lines)

    assert "ARG ZENOH_VERSION=" in dockerfile
    assert "ARG ZENOH_ROS2DDS_VERSION=${ZENOH_VERSION}" in dockerfile
    assert re.search(r"^ARG ZENOH_SHA256=[0-9a-f]{64}$", dockerfile, flags=re.MULTILINE)
    assert re.search(r"^ARG ZENOH_ROS2DDS_SHA256=[0-9a-f]{64}$", dockerfile, flags=re.MULTILINE)
    assert "ARG MCAP_CLI_VERSION=" in dockerfile
    assert re.search(r"^ARG MCAP_CLI_SHA256=[0-9a-f]{64}$", dockerfile, flags=re.MULTILINE)
    assert urls
    assert all("/latest/" not in url for url in urls)
    assert any("${ZENOH_VERSION}" in url for url in urls)
    assert any("${ZENOH_ROS2DDS_VERSION}" in url for url in urls)
    assert any("${MCAP_CLI_VERSION}" in url for url in urls)
    assert active_lines.count("sha256sum -c -") >= 3


def test_dockerfile_git_sources_are_pinned_to_commit_refs() -> None:
    dockerfile = DOCKERFILE_FULL_PATH.read_text(encoding="utf-8")
    active_lines = "\n".join(line for line in dockerfile.splitlines() if not line.lstrip().startswith("#"))

    assert re.search(r"^ARG NOVATEL_OEM7_REF=[0-9a-f]{40}$", dockerfile, flags=re.MULTILINE)
    assert "--branch kilted" not in active_lines
    assert 'git fetch --depth 1 origin "${NOVATEL_OEM7_REF}"' in active_lines
    assert "git checkout --detach FETCH_HEAD" in active_lines


def test_entrypoint_passes_bash_syntax_check() -> None:
    subprocess.run(["bash", "-n", str(ENTRYPOINT_PATH)], check=True)

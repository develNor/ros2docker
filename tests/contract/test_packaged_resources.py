from __future__ import annotations

import re
import subprocess
from importlib import resources
from pathlib import Path

from ros2docker.api import build_context

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "Dockerfile"
ENTRYPOINT_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "entrypoint.sh"


def test_packaged_resources_include_build_schema_example_and_bake_context(tmp_path: Path) -> None:
    package_resources = resources.files("ros2docker").joinpath("resources")

    assert package_resources.joinpath("build", "Dockerfile").is_file()
    assert package_resources.joinpath("build", "entrypoint.sh").is_file()
    assert package_resources.joinpath("examples", "ros2docker.json").is_file()
    assert package_resources.joinpath("schema", "ros2docker.schema.json").is_file()

    config_path = tmp_path / "ros2docker.json"
    config_path.write_text("{}", encoding="utf-8")
    with build_context(config_path) as context_dir:
        assert (context_dir / "Dockerfile").is_file()
        assert (context_dir / "entrypoint.sh").is_file()
        assert (context_dir / "bake_packages").is_dir()


def test_dockerfile_default_base_image_and_digest_are_explicit() -> None:
    dockerfile = DOCKERFILE_PATH.read_text(encoding="utf-8")
    base_image = re.search(r"^ARG BASE_IMAGE=(?P<value>\S+)$", dockerfile, flags=re.MULTILINE)
    digest = re.search(r"^ARG DIGEST=(?P<value>@sha256:[0-9a-f]{64})$", dockerfile, flags=re.MULTILINE)

    assert base_image is not None
    assert base_image.group("value")
    assert digest is not None


def test_dockerfile_external_downloads_use_versioned_urls() -> None:
    dockerfile = DOCKERFILE_PATH.read_text(encoding="utf-8")
    active_lines = "\n".join(line for line in dockerfile.splitlines() if not line.lstrip().startswith("#"))
    urls = re.findall(r"https?://[^\s\"']+", active_lines)

    assert "ARG ZENOH_VERSION=" in dockerfile
    assert "ARG ZENOH_ROS2DDS_VERSION=${ZENOH_VERSION}" in dockerfile
    assert "ARG MCAP_CLI_VERSION=" in dockerfile
    assert urls
    assert all("/latest/" not in url for url in urls)
    assert any("${ZENOH_VERSION}" in url for url in urls)
    assert any("${ZENOH_ROS2DDS_VERSION}" in url for url in urls)
    assert any("${MCAP_CLI_VERSION}" in url for url in urls)


def test_entrypoint_passes_bash_syntax_check() -> None:
    subprocess.run(["bash", "-n", str(ENTRYPOINT_PATH)], check=True)

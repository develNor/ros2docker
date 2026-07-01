"""Contract test: ``init`` scaffolds the workspace dir the entrypoint sources.

Hardens a soft cross-artifact probe into a deterministic gate (issue #148).
``ros2docker init`` once scaffolded ``ws/src`` while the runtime entrypoint,
diagnostics, and examples all used ``ws/ros2src`` (fixed in #143). The unit test
for ``init`` had *encoded* the wrong dir, so CI stayed green over the drift. This
check derives both sides from the real artifacts — the directory ``init``
actually creates and the directory the packaged entrypoint actually guards on —
so the two cannot silently diverge again.
"""

from __future__ import annotations

import re
from pathlib import Path

from ros2docker.api import init

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ENTRYPOINT_PATH = PACKAGE_ROOT / "src" / "ros2docker" / "resources" / "build" / "entrypoint.sh"


def _scaffolded_workspace_dir(base_dir: Path) -> str:
    """Return the single workspace source dir ``init`` creates under ``ws/``."""
    init(cwd=base_dir)
    subdirs = sorted(p.name for p in (base_dir / "ws").iterdir() if p.is_dir())
    assert len(subdirs) == 1, f"init should scaffold exactly one workspace source dir, got {subdirs}"
    return subdirs[0]


def _entrypoint_sourced_workspace_dir() -> str:
    """Return the ``/ws/<dir>`` the entrypoint tests for and symlinks into the build.

    "The dir the entrypoint sources" is the one it gates on with
    ``[ -d "/ws/<dir>" ]`` and then symlinks into the colcon workspace. Both
    structural references must name the same dir; prose (echo) lines are ignored.
    """
    text = ENTRYPOINT_PATH.read_text(encoding="utf-8")
    guarded = re.findall(r'-d\s+"/ws/([^"]+)"', text)
    symlinked = re.findall(r"ln\s+\S+\s+/ws/(\S+)", text)
    assert guarded, 'entrypoint.sh must gate the workspace on `[ -d "/ws/<dir>" ]`'
    assert symlinked, "entrypoint.sh must symlink the /ws workspace dir into the colcon workspace"

    dirs = set(guarded) | set(symlinked)
    assert len(dirs) == 1, f"entrypoint.sh references divergent workspace dirs: {sorted(dirs)}"
    (workspace_dir,) = dirs
    return workspace_dir


def test_init_scaffolds_the_workspace_dir_the_entrypoint_sources(tmp_path: Path) -> None:
    assert _scaffolded_workspace_dir(tmp_path) == _entrypoint_sourced_workspace_dir()

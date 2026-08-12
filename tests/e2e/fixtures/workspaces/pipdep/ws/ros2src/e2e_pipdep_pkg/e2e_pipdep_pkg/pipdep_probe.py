from __future__ import annotations

import sys

# `rich` comes from PIP_PACKAGES, so it lives in the venv and nowhere else.
# Importing it at module scope is the whole point: a console script whose
# shebang pins the system interpreter cannot see the venv, and this import is
# what fails first when it does.
import rich


def main() -> None:
    print(f"E2E_PIPDEP:interpreter={sys.executable}", flush=True)
    print(f"E2E_PIPDEP:rich={rich.__file__}", flush=True)

#!/usr/bin/env python3

import os
import sys
import subprocess

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.getters import *

def main():
    docker_command = [
        'docker',
        'exec',
        "-it",
        get_container_name(),
        "bash"
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

if __name__ == "__main__":
    main()

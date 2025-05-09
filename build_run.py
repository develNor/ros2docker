#!/usr/bin/env python3

import subprocess

from utils.getters import *
from build import main as build

def run():
    docker_command = [
        'docker',
        'run',
        *get_docker_run_args(),
        *get_cli_args(),
        get_image_name(),
        *get_run_cmd()
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

def main():
    build()
    run()

if __name__ == "__main__":
    main()

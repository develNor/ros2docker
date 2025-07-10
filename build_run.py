#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_docker_run_args, get_cli_args, get_image_name, get_run_cmd
from build import main as build



def run(run_command=None):
    run_command = run_command or get_run_cmd()

    docker_command = [
        'docker',
        'run',
        *get_docker_run_args(),
        *get_cli_args(),
        get_image_name(),
        *run_command
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    script_dir = os.path.dirname(os.path.realpath(__file__))
    parent_dir = os.path.dirname(script_dir)
    subprocess.run(docker_command, check=True, cwd=parent_dir)

def main(**run_args):
    build()
    run(**run_args)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-c', '--run_command', help='Command to run in the Docker container')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_docker_run_args, get_cli_args, get_image_name, get_run_cmd, get_config_dir
from build import main as build



def run(config_file=None, override=None):
    docker_command = [
        'docker',
        'run',
        *get_docker_run_args(config_file, override),
        *get_cli_args(config_file, override),
        get_image_name(config_file, override),
        *get_run_cmd(config_file, override)
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True, cwd=get_config_dir(config_file))

def main(config_file=None, override=None):
    build(config_file, override)
    run(config_file, override)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-o', '--override')
    parser.add_argument('-f', '--config_file')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
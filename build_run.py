#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_docker_run_args, get_cli_args, get_image_name, get_run_cmd, get_config_dir
from build import main as build



def run(config_file=None, override=None, mount=None):
    docker_command = [
        'docker',
        'run',
        *get_docker_run_args(config_file, override, cli_mount=mount),
        *get_cli_args(config_file, override),
        get_image_name(config_file, override),
        *get_run_cmd(config_file, override)
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True, cwd=get_config_dir(config_file))

def main(config_file=None, override=None, mount=None):
    build(config_file, override)
    run(config_file, override, mount)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Build and run a ROS2 Docker container.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic ROS2 bash (no config needed)
  python build_run.py

  # Mount current directory into /ws
  python build_run.py -m .

  # Mount a specific folder into /ws
  python build_run.py -m /path/to/my/workspace

  # Use a config file
  python build_run.py -f ./example/config.json
"""
    )
    parser.add_argument('-o', '--override', help='JSON string to override config values')
    parser.add_argument('-f', '--config_file', help='Path to config.json file')
    parser.add_argument('-m', '--mount', metavar='PATH',
                        help='Mount a directory into /ws. Use "." for current directory')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
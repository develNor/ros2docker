#!/usr/bin/env python3

import argparse
import os
import sys
import subprocess

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_image_name, get_docker_build_args

def main(config=None):
    docker_build_args = [ '-t', get_image_name(config) ]
    docker_build_args += get_docker_build_args(config)

    docker_command = [
        'docker', 'build', *docker_build_args, os.path.join(project_dir, 'build')
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-f', '--config_file')
    args = parser.parse_args()
    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})

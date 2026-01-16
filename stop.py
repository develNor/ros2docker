#!/usr/bin/env python3

import argparse
import os
import sys
import subprocess

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import *

def main(config_file=None, override=None):
    docker_command = [ 'docker', 'stop', get_container_name(config_file, override) ]
    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-o', '--override')
    parser.add_argument('-f', '--config_file')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})

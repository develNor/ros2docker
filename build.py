#!/usr/bin/env python3

import os
import sys
import subprocess

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_image_name, get_docker_build_args

def main():
    docker_build_args = [ '-t', get_image_name() ]
    docker_build_args += get_docker_build_args()

    docker_command = [
        'docker', 'build', *docker_build_args, os.path.join(project_dir, 'build')
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

if __name__ == "__main__":
    main()

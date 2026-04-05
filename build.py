#!/usr/bin/env python3

import argparse
import os
import sys
import shutil
import subprocess

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import get_image_name, get_docker_build_args, get_local_config, get_config_dir

def _stage_bake_packages(config_file=None, override=None):
    """Copy packages listed in config "bake_ros_packages" into build/bake_packages/."""
    bake_dir = os.path.join(project_dir, 'build', 'bake_packages')
    os.makedirs(bake_dir, exist_ok=True)

    config = get_local_config(config_file, override)
    pkg_paths = config.get("bake_ros_packages", [])
    config_dir = get_config_dir(config_file)

    for rel_path in pkg_paths:
        src = os.path.normpath(os.path.join(config_dir, rel_path))
        pkg_name = os.path.basename(src)
        dst = os.path.join(bake_dir, pkg_name)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        print(f"Staged bake package: {src} -> {dst}")

def _cleanup_bake_packages():
    bake_dir = os.path.join(project_dir, 'build', 'bake_packages')
    if os.path.exists(bake_dir):
        shutil.rmtree(bake_dir)

def main(config_file=None, override=None):
    _stage_bake_packages(config_file, override)
    try:
        docker_build_args = [ '-t', get_image_name(config_file, override) ]
        docker_build_args += get_docker_build_args(config_file, override)

        docker_command = [
            'docker', 'build', *docker_build_args, os.path.join(project_dir, 'build')
        ]

        print("Executing Docker command:", ' '.join(docker_command))
        subprocess.run(docker_command, check=True)
    finally:
        _cleanup_bake_packages()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-o', '--override')
    parser.add_argument('-f', '--config_file')

    args = parser.parse_args()
    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})

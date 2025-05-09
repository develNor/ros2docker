#!/usr/bin/env python3

import os
import sys

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.files import *
from utils.getters import *


def main():
    # Get the directory of the project relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)

    # Save the contents of the vsc-template-files
    devcontainer_template_file = os.path.join(project_dir, "config/devcontainer.json.template")
    devcontainer_file = os.path.join(project_dir, ".devcontainer/devcontainer.json")

    substitutions = { 
        "#docker_run_args": str(get_docker_run_args()).replace("'", '"')[1:-1], 
        "#image_name": get_image_name(),
    }
    generate_file_from_template(devcontainer_template_file, devcontainer_file, substitutions)

if __name__ == "__main__":
    main()
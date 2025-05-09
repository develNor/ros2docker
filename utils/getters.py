import os
import sys
import json
import json
import os
import shlex
from typing import List, Sequence


script_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.append(project_dir)

from utils.files import *

def get_local_config():
    return json_to_dict(f"{project_dir}/config/local.json")

def get_core_docker_run_args():
    local_config = get_local_config()

    template_file_path = f"{project_dir}/config/core_docker_run_args.json.template"

    ros2ws_src = local_config['ros2ws_src']
    if not os.path.isabs(ros2ws_src):
        ros2ws_src = os.path.abspath(os.path.join(project_dir, ros2ws_src))

    substitutions = {
        "#uid": str(os.getuid()),
        "#gid": str(os.getgid()),
        "#container_name": local_config['container_name'],
        "#ros2ws_src": ros2ws_src
    }

    processed_content = process_template(template_file_path, substitutions)
    content_dict = json.loads(processed_content)
    core_docker_run_args = content_dict["core_docker_run_args"]
    return core_docker_run_args

def get_local_docker_run_args():
    local_docker_run_args = []

    if get_local_config().get("enable_gui_forwarding"):
        local_docker_run_args.extend([
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",
        ])

    if get_local_config().get("forward_ssh_agent"):
        ssh_auth_sock = os.getenv('SSH_AUTH_SOCK')
        local_docker_run_args.extend([
            "-e", "SSH_AUTH_SOCK",
            "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"
        ])

    local_docker_run_args += get_local_config().get("run_args", [])
    
    return local_docker_run_args

def get_docker_run_args():
    return get_core_docker_run_args() + get_local_docker_run_args()

def get_image_name():
    return get_local_config()["image_name"]

def get_container_name():
    return get_local_config()["container_name"]

def get_docker_build_args():
    build_args = []
    for key, value in get_local_config().get("build_args", {}).items():
        build_args.append("--build-arg")
        build_args.append(f"{key}={value}")
    return build_args

def get_cli_args() -> Sequence[str]:
    """
    Extra `docker run` flags depending on run-type.
    """
    local_config = get_local_config()
    run_type = local_config.get("run_type", "bash")

    if run_type == "catmux":
        catmux_file = local_config["catmux_file"]
        if not os.path.isabs(catmux_file):
            catmux_file = os.path.abspath(os.path.join(project_dir, catmux_file))

        return [
            "-it",
            "-v",
            f"{catmux_file}:/shared/catmux.yaml",
        ]

    if run_type == "bash":
        return ["-it"]

    if run_type == "up":
        return ["-d"]

    raise ValueError(f"Unsupported run_type: {run_type!r}")

def _build_catmux_run_cmd() -> List[str]:
    """
    Create the long `/bin/bash -c "catmux_create_session …"` sequence.
    """
    # inner command as list → join at the end so quoting stays trivial
    inner_cmd = ["catmux_create_session", "/shared/catmux.yaml", "--session_name", get_container_name()]

    # append any --overwrite k=v pairs
    for k, v in (get_local_config().get("catmux_params") or {}).items():
        inner_cmd.extend(["--overwrite", f"{k}={v}"])

    return ["/bin/bash", "-c", " ".join(inner_cmd)]


def get_run_cmd() -> Sequence[str]:
    """
    Return the container’s *entry command* as list, driven by run_type.
    """
    run_type = get_local_config().get("run_type", "bash")

    if run_type == "catmux":
        return _build_catmux_run_cmd()
    if run_type == "bash":
        return ["bash"]
    if run_type == "up":
        return ["tail", "-f", "/dev/null"]

    raise ValueError(f"Unsupported run_type: {run_type!r}")

import os
import sys
import json
import json
import os
import warnings
from pathlib import Path
from typing import List, Sequence

script_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.append(project_dir)

from utils.files import *

def get_config_dir(manual_config=None):
    config_file = get_local_config_file(manual_config)
    return os.path.dirname(os.path.abspath(config_file))

def get_local_config(manual_config=None):
    return json_to_dict(get_local_config_file(manual_config))

def get_local_config_file(manual_config=None):
    project_dir = Path(__file__).resolve().parent.parent
    fallback_config = project_dir / "example" / "config.json"
    local_config = project_dir.parent / "config.json"

    if manual_config:
        return manual_config
    elif local_config.is_file():
        return local_config
    else:
        warnings.warn(f"Local config not found at {local_config}, using fallback: {fallback_config}")
        return fallback_config

def get_core_docker_run_args(manual_config=None):
    local_config = get_local_config(manual_config)
    template_file_path = f"{project_dir}/utils/core_docker_run_args.json.template"
    substitutions = {
        "#uid": str(os.getuid()),
        "#gid": str(os.getgid()),
        "#container_name": local_config['container_name']
    }

    processed_content = process_template(template_file_path, substitutions)
    content_dict = json.loads(processed_content)
    core_docker_run_args = content_dict["core_docker_run_args"]
    return core_docker_run_args

def get_local_docker_run_args(manual_config=None):
    local_docker_run_args = []

    if get_local_config(manual_config).get("enable_gui_forwarding"):
        local_docker_run_args.extend([
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",
        ])

    if get_local_config(manual_config).get("forward_ssh_agent"):
        ssh_auth_sock = os.getenv('SSH_AUTH_SOCK')
        local_docker_run_args.extend([
            "-e", "SSH_AUTH_SOCK",
            "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"
        ])

    local_docker_run_args += get_local_config(manual_config).get("run_args", [])
    
    return local_docker_run_args

def get_docker_run_args(manual_config=None):
    return get_core_docker_run_args(manual_config) + get_local_docker_run_args(manual_config) + get_workspace_mount_run_args(manual_config)

def get_image_name(manual_config=None):
    return get_local_config(manual_config).get("image_name", "ros2docker")

def get_container_name(manual_config=None):
    return get_local_config(manual_config)["container_name"]

def get_docker_build_args(manual_config=None):
    build_args = []
    for key, value in get_local_config(manual_config).get("build_args", {}).items():
        build_args.append("--build-arg")
        build_args.append(f"{key}={value}")
    return build_args

def get_cli_args(manual_config=None) -> Sequence[str]:
    """
    Extra `docker run` flags depending on run-type.
    """
    local_config = get_local_config(manual_config)
    run_type = local_config.get("run_type", "bash")

    if run_type == "bash" or run_type == "catmux" or run_type == "command":
        return ["-it"]

    if run_type == "up":
        return ["-d"]

    raise ValueError(f"Unsupported run_type: {run_type!r}")

def _build_catmux_run_cmd(manual_config=None) -> List[str]:
    """
    Create the long `/bin/bash -c "catmux_create_session …"` sequence.
    """
    local_config = get_local_config(manual_config)
    catmux_file = local_config["catmux_file"]
    # inner command as list → join at the end so quoting stays trivial
    inner_cmd = ["catmux_create_session", catmux_file, "--session_name", get_container_name()]

    # append any --overwrite k=v pairs
    for k, v in (get_local_config(manual_config).get("catmux_params") or {}).items():
        inner_cmd.extend(["--overwrite", f"{k}={v}"])

    return [" ".join(inner_cmd)]

def get_run_cmd(manual_config=None) -> Sequence[str]:
    """
    Return the container’s *entry command* as list, driven by run_type.
    """
    run_type = get_local_config(manual_config).get("run_type", "bash")

    if run_type == "catmux":
        return _build_catmux_run_cmd(manual_config)
    if run_type == "bash":
        return ["bash"]
    if run_type == "up":
        return ["tail", "-f", "/dev/null"]
    if run_type == "command":
        local_config = get_local_config(manual_config)
        command = local_config["command"]
        if isinstance(command, str):
            return command.split()
        elif isinstance(command, list):
            return command
        else:
            raise ValueError(f"Invalid command type: {type(command)}. Expected str or list.")
    raise ValueError(f"Unsupported run_type: {run_type!r}")

def get_workspace_mount_run_args(manual_config=None) -> List[str]:
    """
    Get the workspace mount arguments for the Docker run command.
    """
    mount_ws = get_local_config(manual_config).get("mount_ws", False)
    if mount_ws:
        ws_host = os.path.abspath(os.path.join(project_dir, "../ws"))
        if not os.path.exists(ws_host):
            warnings.warn(f"Workspace directory not found at {ws_host}, using example directory at {project_dir}/example/ws")
            ws_host = os.path.abspath(os.path.join(project_dir, "./example/ws"))
        return ["-v", ws_host + ":/ws", "-w", "/ws"]
    else:
        return []
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
    if config_file is None:
        # No config file - use current working directory
        return os.getcwd()
    return os.path.dirname(os.path.abspath(config_file))

def get_default_config():
    """
    Returns sensible defaults for running a basic ROS2 bash container.
    """
    return {
        "container_name": "ros2docker",
        "image_name": "ros2docker",
        "run_type": "bash",
        "mount_ws": False,
        "enable_gui_forwarding": False,
        "run_args": [],
        "build_args": {},
    }

def get_local_config(manual_config=None, override=None):
    config_file = get_local_config_file(manual_config)
    
    if config_file is None:
        # No config file - use defaults
        local_config_dict = get_default_config()
    else:
        local_config_dict = json_to_dict(config_file)
    
    if override:
        if isinstance(override, str):  # make sure override is a dict
            override = json.loads(override)
        local_config_dict.update(override)
    return local_config_dict

def get_local_config_file(manual_config=None):
    """
    Returns the config file path, or None if no config is provided/found.
    When None is returned, the system uses sensible defaults (basic ROS2 bash).
    """
    if manual_config:
        return manual_config
    
    project_dir = Path(__file__).resolve().parent.parent
    local_config = project_dir.parent / "config.json"
    
    if local_config.is_file():
        return local_config
    
    # No config found - return None to trigger default behavior
    return None

def get_image_name(manual_config=None, override=None) -> str:
    return get_local_config(manual_config, override).get("image_name", "ros2docker")

def get_container_name(manual_config=None, override=None) -> str:
    local_config = get_local_config(manual_config, override)
    # Default to image_name if container_name not specified
    return local_config.get("container_name", local_config.get("image_name", "ros2docker"))

def get_core_docker_run_args(manual_config=None, override=None) -> List[str]:
    template_file_path = f"{project_dir}/utils/core_docker_run_args.json.template"
    substitutions = {
        "#uid": str(os.getuid()),
        "#gid": str(os.getgid()),
        "#container_name": get_container_name(manual_config, override)
    }

    processed_content = process_template(template_file_path, substitutions)
    content_dict = json.loads(processed_content)
    core_docker_run_args = content_dict["core_docker_run_args"]
    return core_docker_run_args

def get_local_docker_run_args(manual_config=None, override=None) -> List[str]:
    local_docker_run_args = []

    if get_local_config(manual_config, override).get("enable_gui_forwarding"):
        local_docker_run_args.extend([
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",
        ])

    if get_local_config(manual_config, override).get("forward_ssh_agent"):
        ssh_auth_sock = os.getenv('SSH_AUTH_SOCK')
        local_docker_run_args.extend([
            "-e", "SSH_AUTH_SOCK",
            "-v", f"{ssh_auth_sock}:{ssh_auth_sock}"
        ])

    local_config = get_local_config(manual_config, override)
    raw_run_args = local_config.get("run_args", []) + local_config.get("extra_run_args", [])
    project_dir = get_config_dir(manual_config)

    i = 0
    while i < len(raw_run_args):
        arg = raw_run_args[i]
        # check if current arg is '-v'
        if arg == "-v":
            volume = raw_run_args[i + 1]
            host_path, container_path = volume.split(":", 1)
            if host_path.startswith("../") or host_path.startswith("./"):
                abs_host_path = os.path.realpath(os.path.join(project_dir, host_path))
                volume = f"{abs_host_path}:{container_path}"
            local_docker_run_args.extend(["-v", volume])
            i += 2
        else:
            local_docker_run_args.append(arg)
            i += 1

    return local_docker_run_args

def get_docker_run_args(manual_config=None, override=None, cli_mount=None, cli_extra_run_args=None) -> List[str]:
    return get_core_docker_run_args(manual_config, override) + get_local_docker_run_args(manual_config, override) + get_workspace_mount_run_args(manual_config, override, cli_mount) + (cli_extra_run_args or [])

def get_docker_build_args(manual_config=None, override=None) -> List[str]:
    build_args = ["--build-arg", "USER_UID="+str(os.getuid()), "--build-arg", "USER_GID="+str(os.getgid())]
    for key, value in get_local_config(manual_config, override).get("build_args", {}).items():
        build_args.append("--build-arg")
        build_args.append(f"{key}={value}")
    return build_args

def get_cli_args(manual_config=None, override=None) -> Sequence[str]:
    """
    Extra `docker run` flags depending on run-type.
    """
    local_config = get_local_config(manual_config, override)
    run_type = local_config.get("run_type", "bash")

    if run_type == "bash" or run_type == "catmux" or run_type == "command":
        return ["-it"]

    if run_type == "up":
        return ["-d"]

    raise ValueError(f"Unsupported run_type: {run_type!r}")

def _build_catmux_run_cmd(manual_config=None, override=None) -> List[str]:
    """
    Create the "catmux_create_session …" sequence.
    """
    local_config = get_local_config(manual_config, override)
    catmux_file = local_config["catmux_file"]
    # inner command as list → join at the end so quoting stays trivial
    command = ["catmux_create_session", catmux_file, "--session_name", get_container_name(manual_config, override)]

    # append any --overwrite k1=v1,k2=v2,... pairs
    if "catmux_params" in local_config:
        command.extend(["--overwrite"])
        params = ",".join(f"{k}={v}" for k, v in local_config["catmux_params"].items())
        command.append(params)

    return command

def get_run_cmd(manual_config=None, override=None) -> Sequence[str]:
    """
    Return the container’s *entry command* as list, driven by run_type.
    """
    run_type = get_local_config(manual_config, override).get("run_type", "bash")

    if run_type == "catmux":
        return _build_catmux_run_cmd(manual_config, override)
    if run_type == "bash":
        return ["bash"]
    if run_type == "up":
        return ["tail", "-f", "/dev/null"]
    if run_type == "command":
        local_config = get_local_config(manual_config, override)
        command = local_config["command"]
        if isinstance(command, str):
            return command.split()
        elif isinstance(command, list):
            return command
        else:
            raise ValueError(f"Invalid command type: {type(command)}. Expected str or list.")
    raise ValueError(f"Unsupported run_type: {run_type!r}")

def get_workspace_mount_run_args(manual_config=None, override=None, cli_mount=None) -> List[str]:
    """
    Get the workspace mount arguments for the Docker run command.
    
    Args:
        cli_mount: Optional path from CLI. Can be:
            - None: no CLI mount
            - ".": mount current working directory
            - absolute/relative path: mount that specific directory
    """
    # CLI mount takes precedence
    if cli_mount is not None:
        if cli_mount == ".":
            mount_path = os.getcwd()
        else:
            mount_path = os.path.abspath(cli_mount)
        
        if not os.path.exists(mount_path):
            raise ValueError(f"Mount path does not exist: {mount_path}")
        
        return ["-v", f"{mount_path}:/ws", "-w", "/ws"]
    
    # Fall back to config-based mount_ws
    mount_ws = get_local_config(manual_config, override).get("mount_ws", False)
    if mount_ws:
        config_dir = get_config_dir(manual_config)
        ws_host = os.path.abspath(os.path.join(config_dir, "ws"))
        if not os.path.exists(ws_host):
            warnings.warn(f"Workspace directory not found at {ws_host}, using example directory at {project_dir}/example/ws")
            ws_host = os.path.abspath(os.path.join(project_dir, "./example/ws"))
        return ["-v", ws_host + ":/ws", "-w", "/ws"]
    else:
        return []
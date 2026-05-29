# ros2docker

`ros2docker` is a versioned Python CLI and API for building and running ROS 2 Docker workspaces from JSON-with-comments config files.

## Install

Stable release:

```bash
pipx install --force git+ssh://git@github.com/develNor/ros2docker.git@v0.1.0
```

Latest development version:

```bash
pipx install --force git+ssh://git@github.com/develNor/ros2docker.git@main
```

Non-pipx fallback:

```bash
python3 -m pip install --user --force-reinstall git+ssh://git@github.com/develNor/ros2docker.git@v0.1.0
```

## CLI

```bash
ros2docker build -f ros2docker.json
ros2docker run -f ros2docker.json
ros2docker run -f ros2docker.json --no-build -- -v /host/data:/data
ros2docker stop -f ros2docker.json
ros2docker exec -f ros2docker.json -- bash -lc 'ros2 --help'
ros2docker --version
```

Every Docker action accepts `--dry-run`, which prints the Docker argv and exits without running Docker.

## Config

Config files are JSON with `//` and `/* ... */` comments. Supported keys include:

```json
{
  "container_name": "example_ros2container",
  "image_name": "ros2docker",
  "run_type": "bash",
  "mount_ws": true,
  "enable_gui_forwarding": false,
  "forward_ssh_agent": false,
  "run_args": [],
  "extra_run_args": [],
  "build_args": {},
  "bake_ros_packages": [],
  "catmux_file": "/ws/catmux.yaml",
  "catmux_params": {},
  "command": "ros2 topic list",
  "session_configs_dir": "/session/configs"
}
```

Host paths in `-v/--volume` and bind `--mount` args expand `~` and environment variables. Relative `./` and `../` host paths are resolved from the config file directory.

`bake_ros_packages` paths are also resolved from the config file directory and copied into a temporary Docker build context. The installed Python package directory is never mutated during builds.

## Python API

```python
from ros2docker.api import build, run, build_run, stop, exec_shell
from ros2docker.config import load_config, get_config_dir
from ros2docker.commands import make_build_command, make_run_command
```


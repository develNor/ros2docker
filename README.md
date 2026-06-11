# ros2docker

[![pr-lightweight](https://github.com/develNor/ros2docker/actions/workflows/pr-lightweight.yml/badge.svg)](https://github.com/develNor/ros2docker/actions/workflows/pr-lightweight.yml)
[![nightly-e2e](https://github.com/develNor/ros2docker/actions/workflows/nightly-e2e.yml/badge.svg)](https://github.com/develNor/ros2docker/actions/workflows/nightly-e2e.yml)

`ros2docker` is a versioned Python CLI and API for building and running ROS 2 Docker workspaces from JSON-with-comments config files.

## Install

Stable release:

```bash
pipx install ros2docker
```

Latest development version:

```bash
pipx install --force git+https://github.com/develNor/ros2docker.git@main
```

Non-pipx fallback:

```bash
python3 -m pip install --user ros2docker
```

## Development Install

When developing `ros2docker`, install this checkout in editable mode so the
host `ros2docker` command imports the local source:

```bash
pipx install --force --editable /path/to/fleet_mgmt/ros_communication_devcontainer/ros2docker
```

Normal Python source changes are picked up immediately by new `ros2docker`
commands. Reinstall after changing package metadata, dependencies, or console
entry points in `pyproject.toml`.

Verify the editable command with:

```bash
ros2docker run --no-build --dry-run -m .
```

The dry run should print a `docker run` command that mounts the current
directory as `/ws` and starts `bash`, without requiring `-f/--config`.

## CLI

```bash
ros2docker run -m /host/project
ros2docker run --no-build -m /host/project
ros2docker build -f ros2docker.json
ros2docker run -f ros2docker.json
ros2docker run -f ros2docker.json --no-build -- -v /host/data:/data
ros2docker stop -f ros2docker.json
ros2docker exec -f ros2docker.json -- bash -lc 'ros2 --help'
ros2docker --version
python -m ros2docker --version
```

Every Docker action accepts `--dry-run`, which prints the Docker argv and exits without running Docker.
The `-f`/`--config` option is optional; without it, `ros2docker` uses the default config, which starts an interactive Bash shell.

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
  "command": "ros2 topic list"
}
```

The machine-readable schema lives at `src/ros2docker/resources/schema/ros2docker.schema.json`.
See `docs/configuration.md` for the full configuration contract. Unknown top-level config keys are rejected.

Supported `run_type` values are:

- `bash`: start an interactive shell.
- `command`: run the configured `command`.
- `catmux`: start a catmux session from `catmux_file`.
- `up`: start a detached keepalive container.

Host paths in `-v/--volume` and bind `--mount` args expand `~` and environment variables. Relative `./` and `../` host paths are resolved from the config file directory.

`bake_ros_packages` paths are also resolved from the config file directory and copied into a temporary Docker build context. The installed Python package directory is never mutated during builds.

`enable_gui_forwarding` forwards the X11 socket at `/tmp/.X11-unix`.
`forward_ssh_agent` forwards the host `SSH_AUTH_SOCK` path when the variable is set and points to an existing socket or file.

## Testing

For the full contributor workflow, local check matrix, PR process, and merge
policy, see [CONTRIBUTING.md](CONTRIBUTING.md).

Run the basic local test suites:

```bash
just test-unit
just test-contract
```

Run all required local checks:

```bash
just check
```

Run fast Docker end-to-end fixtures when Docker/runtime behavior changed:

```bash
just test-e2e-fast
```

Run all Docker end-to-end fixtures, including slow image and ROS launch checks:

```bash
just test-e2e-slow
```

## Python API

```python
from ros2docker.api import build, run, build_run, stop, exec_shell
from ros2docker.config import load_config, get_config_dir
from ros2docker.commands import make_build_command, make_run_command
```

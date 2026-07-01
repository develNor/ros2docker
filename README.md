# ros2docker

[![CI](https://github.com/develNor/ros2docker/actions/workflows/pr-merge-gate.yml/badge.svg?branch=main&event=push)](https://github.com/develNor/ros2docker/actions/workflows/pr-merge-gate.yml)
[![Coverage](https://codecov.io/gh/develNor/ros2docker/branch/main/graph/badge.svg)](https://codecov.io/gh/develNor/ros2docker)
[![PyPI](https://img.shields.io/pypi/v/ros2docker.svg)](https://pypi.org/project/ros2docker/)
[![Python](https://img.shields.io/pypi/pyversions/ros2docker.svg)](https://pypi.org/project/ros2docker/)
[![License](https://img.shields.io/pypi/l/ros2docker.svg)](https://pypi.org/project/ros2docker/)
[![Nightly E2E](https://github.com/develNor/ros2docker/actions/workflows/nightly-e2e.yml/badge.svg?branch=main)](https://github.com/develNor/ros2docker/actions/workflows/nightly-e2e.yml)

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
pipx install --force --editable /path/to/ros2docker
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
ros2docker init
ros2docker init --profile desktop --devcontainer
ros2docker run -m /host/project
ros2docker run --no-build -m /host/project
ros2docker build -f ros2docker.json
ros2docker run -f ros2docker.json
ros2docker run -f ros2docker.json --no-build -- -v /host/data:/data
ros2docker stop -f ros2docker.json
ros2docker exec -f ros2docker.json -- bash -lc 'ros2 --help'
ros2docker validate -f ros2docker.json
ros2docker validate -f ros2docker.json --print-resolved
ros2docker doctor -f ros2docker.json
ros2docker --version
python -m ros2docker --version
```

`ros2docker init` scaffolds a starter workspace (`ros2docker.json`, `ws/ros2src/`,
`catmux.yaml`, and optionally `.devcontainer/devcontainer.json`). It will not
overwrite existing files unless `--overwrite` is passed. See
[Profiles](#profiles) for `--profile`/`--ros-distro`.

Every Docker action accepts `--dry-run`, which prints the Docker argv and exits without running Docker.
The `-f`/`--config` option is optional; without it, `ros2docker` uses the default config, which starts an interactive Bash shell.
Use `ros2docker exec` to run a command inside an already-running container, such as one started with `run_type: "up"`.
`validate` checks config syntax and schema without Docker side effects. `doctor` reports host readiness diagnostics before a build or run.

## Shell Completion

`ros2docker` ships tab completion for subcommands, options, and dynamic values
(profiles for `init --profile`, ROS distros for `init --ros-distro`, and file
paths for `-f`/`--config` and `-m`/`--mount`). Enable it with a one-liner.

Bash — add to `~/.bashrc`:

```bash
eval "$(ros2docker completion bash)"
```

Zsh — add to `~/.zshrc`:

```bash
eval "$(ros2docker completion zsh)"
```

Open a new shell (or re-source the rc file), then press `<TAB>` after
`ros2docker`. No extra dependency is required.

## Config

Config files are JSON with `//` and `/* ... */` comments. Supported keys include:

```json
{
  "container_name": "example_ros2container",
  "image_name": "ros2docker",
  "profile": null,
  "dockerfile": "Dockerfile.generic",
  "run_type": "bash",
  "tty": true,
  "stdin_open": true,
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
See [docs/configuration.md](docs/configuration.md) for the full configuration contract. Unknown top-level config keys are rejected.

Supported `run_type` values are:

- `bash`: start an interactive shell.
- `command`: run the configured `command` as a one-shot container.
- `catmux`: start a catmux session from `catmux_file`.
- `up`: start a detached, long-lived keepalive container for later `exec` commands.

`tty` and `stdin_open` control Docker run interactivity. When omitted, `bash` and `catmux` default both to `true`; `command` and `up` default both to `false`. Set them explicitly for an interactive command run.

`mount_ws` mounts the config-adjacent `ws` directory into `/ws`; normal project configs do not need to duplicate that mount in `run_args`.

Host paths in `-v/--volume` and bind `--mount` args expand `~` and environment variables. Relative `./` and `../` host paths are resolved from the config file directory.

`bake_ros_packages` paths are also resolved from the config file directory and copied into a temporary Docker build context. The installed Python package directory is never mutated during builds.

`enable_gui_forwarding` forwards the X11 socket at `/tmp/.X11-unix`.
`forward_ssh_agent` forwards the host `SSH_AUTH_SOCK` path when the variable is set and points to an existing socket or file.

## Profiles

The default image is intentionally boring: an official ROS base image plus
colcon, rosdep, a Python venv, and a small set of base apt packages. Everything
opinionated or project-specific lives in opt-in **profiles** that are merged
into your config via the `profile` key.

Two kinds of profiles ship with `ros2docker`:

- **Base profiles** pick a base image: `minimal` (`ros:*-ros-base`) and
  `desktop` (`osrf/ros:*-desktop-full`).
- **Add-on profiles** layer optional tooling onto a base, toggling
  `Dockerfile.generic` feature flags and adding apt/pip packages: `foxglove`
  (Foxglove bridge/messages, transports, ament linters), `zenoh` (Zenoh router +
  `zenoh-plugin-ros2dds` binaries and the Cyclone/Zenoh RMW packages), `mcap`
  (the `mcap` CLI), and `novatel` (`novatel_oem7_msgs` built from a pinned
  commit).

`profile` accepts a single name or an ordered list of add-ons applied left to
right. `build_args.APT_PACKAGES` and `build_args.PIP_PACKAGES` from profiles and
your own config are unioned, so you can extend a profile without losing its
packages:

```json
{
  "image_name": "my-robot",
  "profile": ["desktop", "foxglove", "zenoh", "mcap"],
  "build_args": { "APT_PACKAGES": "ros-lyrical-rviz2" }
}
```

`project-develnor` is a convenience profile that reproduces the historical full
image (every add-on enabled, Cyclone DDS as the default RMW) on
`Dockerfile.generic`.

Scaffold a workspace pre-wired to a profile and ROS distro:

```bash
ros2docker init --profile desktop
ros2docker init --profile minimal --ros-distro jazzy
```

The add-on apt package names currently target the `lyrical` distro.

## Security / Trust Boundary

`ros2docker` is intended for trusted developer workspaces, configs, and ROS packages. It can mount host paths, forward X11, forward the SSH agent, build arbitrary packages, and execute Docker containers. Treat project configs and workspace contents as code that can affect the host through those features, and do not run untrusted inputs with this tool.

## Development

For contributor setup, local checks, PR modes, merge policy, and release
workflow, see [CONTRIBUTING.md](CONTRIBUTING.md). CI behavior is summarized in
[docs/ci.md](docs/ci.md), release publishing is summarized in
[docs/release.md](docs/release.md), issue-driven work tracking is summarized in
[docs/work-items.md](docs/work-items.md), and reusable task recipes live in
[.github/ISSUE_TEMPLATE/](.github/ISSUE_TEMPLATE/). The quality model behind
those recipes — hard vs. soft checks, the task hierarchy, and how a release is
orchestrated — is in [docs/quality-model.md](docs/quality-model.md), and the
quality rules applied in audits and reviews are in
[DEVELOPMENT_PRINCIPLES.md](DEVELOPMENT_PRINCIPLES.md).

If you own this repository, [docs/owner-runbook.md](docs/owner-runbook.md)
collects the handful of prompts you actually run (quality pass, release) and the
three human-only gates — everything else is the agent layer you never invoke
directly.

## Python API

```python
from ros2docker.api import build, run, build_run, stop, exec_shell
from ros2docker.config import load_config, get_config_dir
from ros2docker.commands import make_build_command, make_run_command
```

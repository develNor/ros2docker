# Configuration

`ros2docker` config files are JSON objects with optional `//` and `/* ... */`
comments. The public config surface is described by the packaged JSON Schema at
`src/ros2docker/resources/schema/ros2docker.schema.json`.

Unknown top-level keys are rejected. This keeps the core `ros2docker` config
surface explicit and prevents project-specific metadata from becoming accidental
public API.

Validate a config without building or running Docker:

```bash
ros2docker validate -f ros2docker.json
ros2docker validate -f ros2docker.json --print-resolved
```

Inspect host readiness before a build or run:

```bash
ros2docker doctor -f ros2docker.json
```

## Example

```json
{
  "container_name": "example_ros2container",
  "image_name": "ros2docker",
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

## Keys

`container_name`: Docker container name.

`image_name`: Docker image tag used by build and run commands.

`run_type`: Container entry behavior. `bash` starts an interactive shell.
`command` runs the configured `command` as a one-shot container. `catmux`
starts a catmux session from `catmux_file`. `up` starts a detached, long-lived
keepalive container for later `ros2docker exec` commands.

`tty`: Allocate a Docker TTY for `docker run`. When omitted, this defaults to
`true` for `bash` and `catmux`, and `false` for `command` and `up`.

`stdin_open`: Keep STDIN open for `docker run`. When omitted, this defaults to
`true` for `bash` and `catmux`, and `false` for `command` and `up`.

`mount_ws`: Mount a config-adjacent `ws` directory into `/ws`. This is the
canonical way to mount a repo-local workspace; normal configs do not need to
duplicate that mount in `run_args`.

`enable_gui_forwarding`: Forward the host X11 socket at `/tmp/.X11-unix`. See
[Security / Trust Boundary](#security--trust-boundary) before enabling this for
a workspace.

`forward_ssh_agent`: Forward `SSH_AUTH_SOCK` when it is set and points to an
existing socket or file. See [Security / Trust Boundary](#security--trust-boundary)
before exposing host credentials to a container.

`run_args`: Docker run arguments. Host paths in `-v`, `--volume`, and bind
`--mount` specs expand `~` and environment variables. Relative `./` and `../`
host paths resolve from the config file directory. Treat host path mounts as
trusted inputs.

`extra_run_args`: Additional Docker run arguments appended after `run_args`.

`build_args`: Docker build arguments passed as `--build-arg` values. The default
Dockerfile pins external source and binary inputs with build args such as
`NOVATEL_OEM7_REF`, `ZENOH_SHA256`, `ZENOH_ROS2DDS_SHA256`, and
`MCAP_CLI_SHA256`; only override those values as an intentional maintenance
change.

`bake_ros_packages`: ROS package directories copied into the temporary Docker
build context. Relative paths resolve from the config file directory.

`catmux_file`: Catmux session file used when `run_type` is `catmux`.

`catmux_params`: Catmux overwrite parameters passed as `key=value` pairs.

`command`: Command string or argv list used when `run_type` is `command`.

## Security / Trust Boundary

Use `ros2docker` only with trusted workspaces, configs, and ROS packages. The
tool can mount host paths, forward X11, forward the SSH agent, build arbitrary
packages, and execute Docker containers. Those developer conveniences do not
make untrusted project inputs safe.

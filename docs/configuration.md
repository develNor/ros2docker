# Configuration

`ros2docker` config files are JSON objects with optional `//` and `/* ... */`
comments. The public config surface is described by the packaged JSON Schema at
`src/ros2docker/resources/schema/ros2docker.schema.json`.

Unknown top-level keys are rejected. This keeps the core `ros2docker` config
surface explicit and prevents project-specific metadata from becoming accidental
public API.

## Example

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

## Keys

`container_name`: Docker container name.

`image_name`: Docker image tag used by build and run commands.

`run_type`: Container entry behavior. Supported values are `bash`, `command`,
`catmux`, and `up`.

`mount_ws`: Mount a config-adjacent `ws` directory into `/ws`.

`enable_gui_forwarding`: Forward the host X11 socket at `/tmp/.X11-unix`.

`forward_ssh_agent`: Forward `SSH_AUTH_SOCK` when it is set and points to an
existing socket or file.

`run_args`: Docker run arguments. Host paths in `-v`, `--volume`, and bind
`--mount` specs expand `~` and environment variables. Relative `./` and `../`
host paths resolve from the config file directory.

`extra_run_args`: Additional Docker run arguments appended after `run_args`.

`build_args`: Docker build arguments passed as `--build-arg` values.

`bake_ros_packages`: ROS package directories copied into the temporary Docker
build context. Relative paths resolve from the config file directory.

`catmux_file`: Catmux session file used when `run_type` is `catmux`.

`catmux_params`: Catmux overwrite parameters passed as `key=value` pairs.

`command`: Command string or argv list used when `run_type` is `command`.

# ros2docker

Build and run ROS2 containers with flexible configuration options.

## Quick Start

### Basic ROS2 Bash (No Config)

Run a basic ROS2 bash container with no mounts:

```bash
python build_run.py
```

### Mount a Directory

Mount the current directory into `/ws`:

```bash
python build_run.py -m .
```

Mount a specific folder:

```bash
python build_run.py -m /path/to/your/workspace
```

### Using a Config File

For more advanced setups (catmux, GUI forwarding, custom run args), use a config file:

```bash
# Use the included example config
python build_run.py -f ./example/config.json

# Use your own config
python build_run.py -f /path/to/your/config.json
```

See `example/config.json` for all available options.

### Extra Docker Run Arguments

Pass additional `docker run` arguments after `--` without modifying the config:

```bash
# Add a volume mount and env var on top of the config's run_args
python build_run.py -f ./config.json -- -v /host/data:/data -e MY_VAR=1
```

You can also use the `extra_run_args` config key (or supply it via `-o` override)
to append arguments without replacing the base `run_args`:

```bash
python build_run.py -f ./config.json -o '{"extra_run_args": ["-v", "/host/data:/data"]}'
```

### Mounted ROS 2 Workspaces

When `/ws/ros2src` exists in the container, the entrypoint links it to
`/ros2ws/src`. This keeps arbitrary workspace assets under `/ws` while giving
ROS 2 tools a normal colcon workspace at `/ros2ws`.

Set these environment variables through `run_args` when you want automatic
workspace validation/builds:

```jsonc
"run_args": [
  "-e", "BUILD_ROS2WS=1",
  "-e", "CHECK_ROS2WS_DEPENDENCIES=1"
]
```

With `BUILD_ROS2WS=1`, ros2docker builds the mounted workspace with
`--symlink-install`, suppresses known unused CMake CLI variable warnings, and
sources `/ros2ws/install/setup.bash` before running the configured command. This
means generated message packages and installed Python entry points are visible
to `bash`, `catmux`, and `command` run types without per-command overlay setup.

Useful knobs:

| Variable | Default | Description |
|----------|---------|-------------|
| `BUILD_ROS2WS` | `0` | Build `/ros2ws` before running the command |
| `CHECK_ROS2WS_DEPENDENCIES` | `0` | Run `rosdep install --simulate` for `/ros2ws/src` |
| `ROS2WS_SYMLINK_INSTALL` | `1` | Use `colcon build --symlink-install` |
| `ROS2WS_SUPPRESS_UNUSED_CMAKE_WARNINGS` | `1` | Add `--cmake-args --no-warn-unused-cli` |
| `ROS2DOCKER_TRACE` | `0` | Enable shell tracing in the entrypoint |

## CLI Options

| Option | Description |
|--------|-------------|
| `-m, --mount PATH` | Mount a directory into `/ws`. Use `.` for current directory |
| `-f, --config_file PATH` | Path to a `config.json` file |
| `-o, --override JSON` | JSON string to override config values |
| `-- ARGS...` | Additional docker run arguments appended after config args |

## Config File Options

Place a `config.json` in the parent directory of `ros2docker/`, or specify one with `-f`.

```jsonc
{
  // Required
  "container_name": "my_ros2_container",

  // Run type: "bash" (default), "catmux", "up", or "command"
  "run_type": "bash",

  // Mount the ./ws directory into /ws
  "mount_ws": true,

  // Enable X11 GUI forwarding
  "enable_gui_forwarding": true,

  // Forward SSH agent into container
  "forward_ssh_agent": true,

  // Extra docker run arguments
  "run_args": [
    "-e", "ROS_DOMAIN_ID=42"
  ],

  // Additional docker run arguments (appended to run_args, useful with -o override)
  "extra_run_args": [
    "-v", "/host/path:/container/path"
  ],

  // Custom image name (default: "ros2docker")
  "image_name": "my_ros2_image",

  // Build arguments
  "build_args": {
    "BASE_IMAGE": "osrf/ros:jazzy-desktop-full-noble",
    "APT_PACKAGES": "git wget curl",
    "PIP_PACKAGES": "pytest"
  },

  // For run_type: "catmux"
  "catmux_file": "/ws/catmux.yaml",
  "catmux_params": {
    "param1": "value1"
  },

  // For run_type: "command"
  "command": "ros2 topic echo /chatter"
}
```

## Other Scripts

```bash
# Build only (no run)
python build.py [-f config.json]

# Execute bash in running container
python exec_bash.py [-f config.json]

# Stop container
python stop.py [-f config.json]
```

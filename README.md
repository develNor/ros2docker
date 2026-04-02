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


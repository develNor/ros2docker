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

## CLI Options

| Option | Description |
|--------|-------------|
| `-m, --mount PATH` | Mount a directory into `/ws`. Use `.` for current directory |
| `-f, --config_file PATH` | Path to a `config.json` file |
| `-o, --override JSON` | JSON string to override config values |

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


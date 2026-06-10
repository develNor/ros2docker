# Removed Config Surface Notes

This file records documentation surface that was removed because it did not
match ros2docker core behavior. These entries are not supported config keys or
run modes in the public ros2docker package.

## `run_type: "up"` as `docker-compose up`

Older example comments described `run_type: "up"` as `docker-compose up`.
ros2docker does not currently read compose files or call `docker compose`.
The implemented behavior is a detached Docker container running `tail -f
/dev/null`. Users that need compose should run compose outside ros2docker, or
add a real compose mode with tests before documenting it.

## X11/Wayland GUI Forwarding

The docs mentioned X11/Wayland forwarding together. ros2docker currently only
mounts `/tmp/.X11-unix` and forwards `DISPLAY`, so the documented feature is
X11 forwarding only.

## `session_configs_dir`

`session_configs_dir` is consumed by the sibling
`ros_communication_devcontainer` wrapper scripts. ros2docker core only
normalizes the value when present and does not otherwise use it. It is not part
of the public ros2docker config surface.

## `commmand`

The example config had a typo spelling `command` as `commmand`. The supported
key is `command`.

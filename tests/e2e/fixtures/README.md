# ros2docker E2E Fixtures

These fixtures are small public stand-ins for the private or large fleet
projects that consume ros2docker.

- `projects/command_mount`: command-style project with `mount_ws`, relative
  volume paths, config `extra_run_args`, and CLI extra args.
- `projects/catmux_center_like`: center/network-logger style catmux session
  with parameter overwrite and multiple windows.
- `workspaces/std`: mounted ROS 2 workspace with source code and only standard
  message dependencies.
- `workspaces/custom`: mounted ROS 2 workspace that imports a custom message
  package baked into the image.
- `workspaces/missing_dep`: dependency-check failure fixture with a missing
  rosdep key.
- `bake/e2e_msgs`: custom message package used by `bake_ros_packages`.

The tests also cover minimal CLI commands, detached `up` lifecycle, GUI/SSH
forwarding contracts, native two-container chatter, rosbag record/play,
foxglove launch smoke, and alternate base-image builds.

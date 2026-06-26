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
  "profile": ["desktop", "foxglove"],
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

`profile`: Predefined image profile(s) merged in as base configuration before
the config file and overrides. A single profile name or an ordered list of
add-on profiles applied left to right. See [Profiles](#profiles).

`dockerfile`: The packaged Dockerfile resource to build. Defaults to
`Dockerfile.generic`; profiles normally set this and you rarely need to.

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

`build_args`: Docker build arguments passed as `--build-arg` values.
`Dockerfile.generic` gates its optional add-ons behind `INSTALL_ZENOH`,
`INSTALL_MCAP`, and `INSTALL_NOVATEL` flags (set by the matching profiles) and
pins external source and binary inputs with build args such as
`NOVATEL_OEM7_REF`, `ZENOH_SHA256`, `ZENOH_ROS2DDS_SHA256`, and
`MCAP_CLI_SHA256`; only override those pins as an intentional maintenance
change. The `APT_PACKAGES` and `PIP_PACKAGES` values are unioned across profiles
and the config file rather than replaced, so a config can extend a profile's
package list.

`bake_ros_packages`: ROS package directories copied into the temporary Docker
build context. Relative paths resolve from the config file directory.

`catmux_file`: Catmux session file used when `run_type` is `catmux`.

`catmux_params`: Catmux overwrite parameters passed as `key=value` pairs.

`command`: Command string or argv list used when `run_type` is `command`.

## Profiles

Profiles are packaged config fragments under
`src/ros2docker/resources/profiles/`. The `profile` key merges one or more of
them in before the config file and overrides, so the default image stays generic
while opinionated tooling is opt-in.

Base profiles select a base image:

- `minimal`: `ros:lyrical-ros-base-resolute`.
- `desktop`: `osrf/ros:lyrical-desktop-full-resolute` (digest-pinned).

Add-on profiles layer optional tooling on top of a base by toggling
`Dockerfile.generic` feature flags and adding packages. They do not pin a base
image, so they compose:

- `foxglove`: Foxglove bridge/messages, common transports, ament linters.
- `zenoh`: Zenoh router + `zenoh-plugin-ros2dds` binaries (`INSTALL_ZENOH`) and
  the Cyclone/Zenoh RMW apt packages.
- `mcap`: the `mcap` CLI (`INSTALL_MCAP`).
- `novatel`: `novatel_oem7_msgs` built from a pinned commit (`INSTALL_NOVATEL`).

`project-develnor` is a convenience profile that reproduces the historical full
image (all add-ons enabled, Cyclone DDS default RMW) on `Dockerfile.generic`.

Compose add-ons with a list; `APT_PACKAGES`/`PIP_PACKAGES` are unioned:

```json
{
  "image_name": "my-robot",
  "profile": ["desktop", "foxglove", "zenoh"],
  "build_args": { "APT_PACKAGES": "ros-lyrical-rviz2" }
}
```

`ros2docker init --profile <name>` scaffolds a config wired to a profile, and
`--ros-distro <distro>` overrides the base image for non-lyrical distros. Add-on
apt package names currently target the `lyrical` distro.

## File Ownership And The Container User

A central promise of `ros2docker` is that mounted files behave like native
files: you can create and edit them on the host and inside the container
interchangeably, with no ownership or permission friction and **without sudo**.

How it works: the image is built with the host user's `USER_UID`/`USER_GID`, and
`docker run` uses `--user <uid>:<gid>` with the same values. Because the
container process runs as the host user's numeric IDs, files under any bind
mount (the `mount_ws` `/ws` mount and any `-v` mount) are owned by that user on
both sides. A file the host creates is writable in the container without sudo; a
file the container creates is owned by the invoking user on the host and editable
there without sudo.

This contract is locked by end-to-end tests so it cannot silently regress:

- `tests/e2e/test_fast_e2e.py::test_mounted_files_are_native_and_no_sudo_editable_both_directions`
  proves bidirectional, no-sudo ownership for the workspace mount and a generic
  `-v` mount.
- The clean, pre-sourced ROS 2 state on every entry path is covered by the
  workspace (`command`), `catmux`, and interactive `bash` E2E tests, each of
  which runs a workspace node directly.

**User-model decision.** The current build-time-UID + runtime `--user` model is
kept: the ownership contract above passes against it, so the alternatives
considered earlier (runtime `gosu` UID remap, build-time-only user, or a numeric
user without entrypoint sudo) are not needed for the supported single-user
workflow. The entrypoint's `sudo` use only prepares the in-container `/ros2ws`
build directory; it never touches mounted files. Images shared across users with
different host UIDs are out of scope for this contract and would require a
runtime-remap model if that becomes a goal.

## Security / Trust Boundary

Use `ros2docker` only with trusted workspaces, configs, and ROS packages. The
tool can mount host paths, forward X11, forward the SSH agent, build arbitrary
packages, and execute Docker containers. Those developer conveniences do not
make untrusted project inputs safe.

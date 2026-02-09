# Helios2 Docker + ROS 2 Jazzy (Arena SDK)

This repo provides a Docker-based setup for using Lucid Vision Labs Arena SDK with ROS 2 Jazzy.

## Credits
- `arena_camera_ros2` is included as a git submodule from Lucid Vision Labs:
  https://github.com/lucidvisionlabs/arena_camera_ros2

See `THIRD_PARTY_NOTICES.md` for details.

## Requirements
- Docker
- For ArenaViewMP GUI on NVIDIA: NVIDIA driver + NVIDIA Container Toolkit
- X11 running on the host

## Assets
Download the official SDK at this [link](https://thinklucid.com/downloads-hub) and place the `.tar.gz` in `assets/`:

- `assets/ArenaViewMP*_Linux_x64*.tar.gz`

The tarball is expected to contain:
- `ArenaSDK_Linux_x64/`
- `Lucid_Broadcom_Driver_Package_v*.tar.gz`

## Build the image
```bash
./docker/docker_build.sh
```

## Run the container
```bash
./docker/docker_run.sh
```

## Run ArenaViewMP (GUI)
Inside the container:
```bash
avmp
```
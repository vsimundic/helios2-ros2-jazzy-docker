# Helios2 Docker + ROS 2 Jazzy (Arena SDK)

Docker-based setup for using the Lucid Vision Labs Arena SDK with **ROS 2 Jazzy**, including:
- Helios2 HAL ROS 2 driver (point cloud + intensity + camera info)
- Tree capture UI node for synchronized dataset capture

## Credits
- A fork of `ROS2-HAL-LucidLabs-Helios2` from TsuruMasato is included as a git submodule: https://github.com/TsuruMasato/ROS2-HAL-LucidLabs-Helios2

The fork can be seen here: https://github.com/vsimundic/ROS2-HAL-LucidLabs-Helios2

## Requirements
- Docker
- X11 running on the host (for GUI windows)
- For ArenaViewMP GUI on NVIDIA: NVIDIA driver + NVIDIA Container Toolkit

---

# Clone (with submodules)
Clone with submodules enabled:
```bash
git clone --recurse-submodules https://github.com/vsimundic/helios2-ros2-jazzy-docker.git
```

## Installation

### 1) Assets
Download the official Arena SDK tarball (https://thinklucid.com/downloads-hub/) and place it into `assets/`:

- `assets/ArenaViewMP*_Linux_x64*.tar.gz`

The tarball is expected to contain:
- `ArenaSDK_Linux_x64/`
- `Lucid_Broadcom_Driver_Package_v*.tar.gz`

### 2) Build the image
```bash
./docker/docker_build.sh
```

### 3) Enable displaying windows from the container
```bash
xhost local:root
```
### 4) Run the container
```bash
./docker/docker_run.sh
```

### 5) ArenaViewMP (GUI) - Currently unavailable
Inside the container:
```bash
avmp
```

---

## ROS 2 node: Helios2 HAL (point cloud + intensity image + depth image + camera info)

This repository includes a ROS 2 component node that connects to a Helios2-family camera via Arena SDK and publishes:

- `/<frame_id>/points` (`sensor_msgs/msg/PointCloud2`)
- `/<frame_id>/intensity/image_raw` (`sensor_msgs/msg/Image`, `mono16`) - intensity image (Y channel)
- `/<frame_id>/depth/image_raw` (`sensor_msgs/msg/Image`, `mono16`) - depth image
- `/<frame_id>/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)

Default `frame_id` is `camera`, so the default topics are:
- `/camera/points`
- `/camera/intensity/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

### Parameters
Parameters can be found in `helios2_ws/src/helios2_hal/params/*.yaml`:

- `frame_id` (string, default: `camera`)  
  Used for message `header.frame_id` and to form topic names.

- `structured_cloud` (bool, default: `true`)  
  If `true`, publishes an organized cloud (`width x height`).  
  If `false`, publishes an unorganized cloud with invalid points dropped.

- `publish_intensity` (bool, default: `true`)  
  If `true`, configures the camera for `Coord3D_ABCY16` and publishes:
  - intensity in the point cloud (XYZI)
  - `/<frame_id>/image_raw` as mono16 intensity

- `exposure_level` (int, default: `0`)  
  Index into the internal exposure preset list (e.g., `Exp62_5Us`, `Exp250Us`, ...).

- `hdr_mode` (int, default: `0`)  
  HDR mode selector (availability depends on camera model).

- `accumulate_frames` (int, default: `1`)  
  Used for `Scan3dImageAccumulation` when HDR is off.

- `mode` (int, default: `0`)  
  Index into the internal `Scan3dOperatingMode` list. Some modes may not be available on all Helios models.

- `confidence_filter.enable` (bool, default: `true`)
- `confidence_filter.threshold` (int, default: `0`)
- `spatial_filter.enable` (bool, default: `true`)
- `flying_filter.enable` (bool, default: `true`)
- `flying_filter.threshold` (int, default: `0`)

#### PixelFormat note
Intensity publishing requires `Coord3D_ABCY16`. If you publish `image_raw`, the camera must be configured to an ABCY pixel format (4 channels).  
If you disable intensity, ensure the node does not attempt to publish `image_raw` while the camera is in `Coord3D_ABC16` (3 channels).

---

## Building and running (inside the container)

In the container, navigate to the workspace:
```bash
cd /home/user/helios2-ros2/helios2_ws
```

Build:
```bash
colcon build
source install/setup.bash
```

If warnings show up, just build again.

Launch the driver:
```bash
with_arena ros2 launch helios2_hal hal_lucidlabs_helios2.launch.py
```

---

## Tree capturing

Tree capturing is performed by a ROS 2 node named `tree_capture` (script: `capture_node`) inside the `tree_capture` package.

Workspace path: `helios2_ws/src/tree_capture`

### Run
```bash
ros2 launch tree_capture capture.launch.py
```

### Node parameters
The node declares the following ROS 2 parameters, which can be set in `helios2_ws/src/tree_capture/params/capture_params.yaml`. Remember to run `colcon build` after you change the `.yaml`.

#### Output / dataset labeling
- `base_dir` (string, default: `BRANCH_vZ`)  
  Root output directory for captured data.

- `camera_type` (string, default: `helios2`)  
  Subdirectory under `base_dir` indicating the camera name.

- `label` (string, default: `B`)  
  Dataset label folder under `<base_dir>/<camera_type>/`.  
  Typically: `B` (before) or `A` (after).

- `row` (string, default: `1`)  
  Orchard row identifier.

- `variety` (string, default: `unknown`)  
  Apple variety identifier (sanitized for folder naming: trimmed, spaces→`_`, lowercase, invalid path chars removed).

- `tree_id` (string, default: `0000`)  
  Tree identifier (zero-padded to 4 digits).

- `view_id` (string, default: `0`)  
  Capture index. Increments after each save (SPACE). Resets to `0` after `N` (next tree).

- `capture_dir` (string, default: `captures`)  
  Subdirectory inside each tree folder containing `depth/` and `pointcloud/`.

#### Preview window
- `preview_width` (int, default: `960`)  
- `preview_height` (int, default: `540`)

#### Topic names (synchronized capture)
- `depth_topic` (string, default: `/camera/depth/image_raw`)
- `intensity_topic` (string, default: `/camera/intensity/image_raw`)
- `points_topic` (string, default: `/camera/points`)
- `camera_info_topic` (string, default: `/camera/depth/camera_info`)

#### Synchronization tuning (message_filters)
- `sync_queue_size` (int, default: `10`)
- `sync_slop_sec` (double, default: `0.05`)

#### Saving options
- `rotate_ply_x180` (bool, default: `true`)  
  If `true`, the saved point cloud is rotated 180° around the X axis (negates Y and Z).

### Output layout
Each SPACE keypress saves a synchronized bundle of depth, intensity, point cloud, and camera info.

```
<base_dir>/<camera_type>/<label>/tree_<row>_<variety>_<tree_id%04d>/<capture_dir>/
  depth/
    <image_count>_depth.png
    <image_count>_heatmap.png
    <image_count>_intensity.png
    <image_count>_camera_params.yaml
  pointcloud/
    <image_count>.ply
```

### UI controls
- `SPACE` — save synchronized bundle and increment `view_id`
- `N` — increment `tree_id` and reset `view_id` to `0`
- `H` — toggle heatmap/grayscale preview
- `ESC` — exit

---

## End-to-end workflow (Docker → Helios2 HAL → Tree capture)

This section describes the full process of running everything: starting the container, launching the Helios2 HAL driver node, and then launching the tree capturing UI node (plus optional recording).

### 1) Build the Docker image (host)
```bash
./docker/docker_build.sh
```

### 2) Run the container (host Terminal 1)
```bash
./docker/docker_run.sh
```

### 3) Build the workspace (inside container, Terminal 1)
```bash
cd /home/user/helios2-ros2/helios2_ws
colcon build
source install/setup.bash
```

### 4) Launch the Helios2 HAL driver (inside container, Terminal 2)
Open a second terminal in the same container.

```bash
docker exec -it helios2-ros2-dev bash
cd /home/user/helios2-ros2/helios2_ws
source install/setup.bash
with_arena ros2 launch helios2_hal hal_lucidlabs_helios2.launch.py
```

Optional sanity checks:
```bash
ros2 topic list | grep camera
ros2 topic echo --once /camera/camera_info
```

### 5) Change the parameters in YAML file if necessary
Open `helios2_ws/src/tree_capture/params/capture_params.yaml` and change parameters as necessary. Run this to store the changes for ROS2:
```bash
docker exec -it helios2-ros2-dev bash
cd /home/user/helios2-ros2/helios2_ws
colcon build
source install/setup.bash
```

### 5) Launch the tree capture UI (inside container, Terminal 3)
Open a third terminal in the same container.

```bash
docker exec -it helios2-ros2-dev bash
cd /home/user/helios2-ros2/helios2_ws
source install/setup.bash
ros2 launch tree_capture capture.launch.py
```
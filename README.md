# kinectV2_ros2

> A ROS 2 wrapper for the `libk4w2` library that publishes RGB and depth image data — along with `camera_info` — into the ROS 2 ecosystem.

![Language](https://img.shields.io/badge/language-C%2B%2B%2FC-blue)
![ROS](https://img.shields.io/badge/ROS-2-brightgreen)
![License](https://img.shields.io/badge/license-GPL--2.0-orange)

---

## Overview

`kinectV2_ros2` bridges the open-source [libk4w2](https://github.com/yoshimoto/libk4w2) driver with a ROS 2 environment, enabling any ROS 2 node or tool (such as RViz2 or a point-cloud converter) to consume live Kinect V2 sensor streams with minimal setup.

**Published data:**
- RGB image stream (`sensor_msgs/Image`)
- Depth image stream (`sensor_msgs/Image`)
- Camera calibration metadata (`sensor_msgs/CameraInfo`)

---

## Repository Structure

```
kinectV2_ros2/
├── camera_publisher/   # ROS 2 package: node source code, CMakeLists, package.xml
├── LICENSE             # GPL-2.0
└── README.md
```

---

## Prerequisites

| Dependency | Notes |
|---|---|
| ROS 2 (any active distro) | Tested with standard colcon-based workspaces |
| [libk4w2](https://github.com/yoshimoto/libk4w2) | Must be installed with all its own dependencies before building |
| colcon | Standard ROS 2 build tool |

> Refer to the [libk4w2 repository](https://github.com/yoshimoto/libk4w2/tree/master) for its full installation guide and hardware requirements (USB 3.0, supported Kinect V2 firmware).

---

## Installation

```bash
# 1. Navigate into your ROS 2 workspace source directory
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/Abdallah-Hesham99/kinectV2_ros2.git

# 3. Move back to the workspace root
cd ..

# 4. Build only the camera_publisher package
colcon build --symlink-install --packages-select camera_publisher
```

---

## Usage

### Launch the publisher node

Open a terminal, source your workspace, then run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run camera_publisher pub
```

This node begins publishing on the following topics immediately after launch:

| Topic | Type | Description |
|---|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Live RGB frames |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Live depth frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Lens calibration data |

### Visualise in RViz2

```bash
rviz2
```

Add the relevant `Image` or `DepthCloud` displays and point them at the topics above.

### Generate a PointCloud2

Use the companion package [depthimage_to_pointcloud2](https://github.com/ricardodeazambuja/depthimage_to_pointcloud2) to convert the depth image into a 3D point cloud:

```bash
ros2 run depthimage_to_pointcloud2 depthimage_to_pointcloud2_node
```

---

## Architecture

```
[Kinect V2 Hardware]
        │  USB 3.0
        ▼
  [libk4w2 driver]
        │
        ▼
[camera_publisher node]
    ├──► /rgb/image_raw      (sensor_msgs/Image)
    ├──► /depth/image_raw    (sensor_msgs/Image)
    └──► /camera_info        (sensor_msgs/CameraInfo)
```

---

## Language Breakdown

| Language | Share |
|---|---|
| C++ | 59.6 % |
| C | 37.0 % |
| CMake | 3.4 % |

---

## License

Distributed under the **GNU General Public License v2.0**. See [`LICENSE`](LICENSE) for full terms.

---

## Related Projects

- [libk4w2](https://github.com/yoshimoto/libk4w2) — underlying driver
- [depthimage_to_pointcloud2](https://github.com/ricardodeazambuja/depthimage_to_pointcloud2) — depth → PointCloud2 converter

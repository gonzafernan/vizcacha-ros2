
# Vizcacha project

## Software
Environment setup following [Setup ROS 2 with VSCode and Docker](https://docs.ros.org/en/foxy/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)

### URDF robot description
URDF setup following [Setting Up The URDF](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html)

To visualize the robot with RViz the following service can be launched:

```bash
docker compose up display-robot
```

### Robot odometry in ROS 2
Odometry setup following [Setting Up Odometry](https://docs.nav2.org/setup_guides/odom/setup_odom.html)

### Development with Docker
The main resource used for the Docker image development was [An Updated Guide to Docker and ROS 2](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/).

Use of GUIs possible following [Using GUIs with Docker](https://wiki.ros.org/docker/Tutorials/GUI)

## Vision
### Intel RealSense D435

Build Intel RealSense tools following [Linux/Ubuntu - RealSense SDK 2.0 Build Guide](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide).

Version of the SDK used: [Intel RealSense SDK 2.0 beta (v2.56.2)](https://github.com/IntelRealSense/librealsense/releases/tag/v2.56.2)

Useful resources:
- [Intel RealSense SDK 2.0 documentation](https://www.intelrealsense.com/sdk-2/#)

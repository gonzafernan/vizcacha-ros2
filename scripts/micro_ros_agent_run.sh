#!/bin/bash

# Launch micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0
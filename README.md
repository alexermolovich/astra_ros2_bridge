# Astra ROS 2 Jazzy Bridge

This repository provides a ROS 2 Jazzy bridge for the Orbbec Astra camera.

The package works by adapting the Astra SDK and building a ROS 2 package around it to publish camera data into the ROS 2 ecosystem.

## Prerequisites

- ROS 2 Jazzy
- Orbbec Astra SDK

Download the Astra SDK from:
https://www.orbbec.com/developers/astra-sdk/

## Build Instructions

1. Download and extract the Astra SDK.

2. Navigate to the `CMakeLists.txt` file in this repository.

set(ASTRA_ROOT "your astra path")

4. Build the package 

## Running the Package

After building and sourcing your workspace, run:

ros2 run astra_ros2_bridge astra_ros2_bridge

## Planned Updates

- Add camera info publishing done
- Docker container support (in progress)

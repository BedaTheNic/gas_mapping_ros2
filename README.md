# Gas Mapping ROS 2 Project

This project maps gas sensor data (MQ2, MQ3, MQ9, MQ135) onto a 3D SLAM-generated environment using ROS 2 Humble.

## Packages
- `gas_mapping`: Visualization and fusion nodes
- `gas_mapping_msgs`: Custom message definitions

## Topics Published
- `/mq2_gas_markers`, `/mq3_gas_pointcloud`, ...

## Setup

```bash
git clone ...
colcon build
source install/setup.bash

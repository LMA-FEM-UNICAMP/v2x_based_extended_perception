# V2X Based Extended Perception

This repository contains ROS 2 packages designed to integrate Vehicle-to-Everything (V2X) communication as extended perception for Autoware Core/Universe.

By leveraging V2X Cooperative Awareness Messages (CAM) and ETSI standards, this stack extends the vehicle's perception capabilities beyond line-of-sight sensors, providing tracking and map-based prediction of connected entities (vehicles, pedestrians, infrastructure).

## Demo

| Obstacle avoidance | V2V-based perception dataset |
| :---: | :---: |
| [![V2X Map Prediction Demo](https://img.youtube.com/vi/ucnywTGE6lE/0.jpg)](https://www.youtube.com/watch?v=ucnywTGE6lE) | [![V2X Perception Demo](https://img.youtube.com/vi/2OaMEHSLu_8/0.jpg)](https://www.youtube.com/watch?v=2OaMEHSLu_8) | 

> **Note:** Click on the thumbnails above to watch the demonstration videos on YouTube.

## Repository Structure

This repository is organized into a modular architecture containing the following ROS 2 packages:

* **`v2x_extended_perception`** (Meta-Package & Bring-up): 
  The primary entry point of the repository. It manages the launch of UDP drivers (with automatic lifecycle transitions) to communicate with V2X hardware (such as Cohda Wireless MK6 OBUs). It also configures the ETSI ITS message parsing (handling specific BTP header offsets) to translate raw UDP payloads into ROS 2 messages.
* **`v2x_cam_extended_perception`** (Perception Node): 
  A C++ node that ingests the parsed V2X CAM data. Utilizing Autoware's `map_height_fitter` and `geography_utils`, it projects the GNSS coordinates of external entities onto the local pointcloud map, converting them into standard Autoware `DetectedObjects` (`autoware_perception_msgs`).

## Installation and Build

Ensure you have a working ROS 2 environment (e.g., Humble or Jazzy) and standard Autoware dependencies installed. [`etsi_its_messages`](https://github.com/ika-rwth-aachen/etsi_its_messages) repository is also a dependency.

```bash
# Clone the repository into your colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/LMA-FEM-UNICAMP/v2x_based_extended_perception
git clone https://github.com/ika-rwth-aachen/etsi_its_messages

# Build the packages
cd ~/ros2_ws
colcon build --packages-up-to v2x_extended_perception v2x_cam_extended_perception

# Source the workspace
source install/setup.bash
```

## Usage

The entire stack can be launched conveniently using the `v2x_extended_perception` meta-package. This single launch file spins up the UDP sender/receiver, the ETSI ITS parser, and the CAM perception node, while automatically handling the lifecycle transitions for the drivers.

```bash
ros2 launch v2x_extended_perception v2x_extended_perception.launch.py
```

### Launch Arguments

You can modularly disable parts of the stack if you are playing back bags or only testing perception:

* `launch_drivers` (default: `true`): Launches the UDP sender, UDP receiver, and the ETSI ITS parser.
* `launch_cam_perception` (default: `true`): Launches the CAM-to-Autoware object perception node.

Example: Running only the perception node (e.g., if replaying a rosbag containing raw ETSI messages):
```bash
ros2 launch v2x_extended_perception v2x_extended_perception.launch.py launch_drivers:=false
```

## Future Work

This package presents a modular structure designed to easily integrate further implementations of ETSI messages, such as Collective Perception Messages (CPM), MAPEM, and SPATEM, as extended perception inputs to the autonomous driving stack.
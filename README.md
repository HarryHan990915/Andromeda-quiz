# Andromeda Proximity Filter

## Build and Run Instructions

Make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```
Go to the work space and build the package

```bash
cd ~/ros_ws

colcon build --packages-select andromeda_proximity_filter

source install/setup.bash
```
A single launch file is created with binding filter pointcloud algorithm and also
a python script of pointcloud publisher

```bash
ros2 launch andromeda_proximity_filter proximity_filter_launch.py
```
A Google test is designed to test the pointcloud's filter performance, which is
separated from ROS. Without ROS, we can test that. The tested report is screenshoted in the file

```bash
colcon test --packages-select andromeda_proximity_filter
colcon test-result --verbose
```
![Proximity Filter Demo](test_result.png)

## Design Notes

The Andromeda Proximity Filter is structured into two main component from the main.cpp, which is proximity_filter_lib.cpp and proximity_filter_node.cpp for object oriented design.

### 1. proximity_filter_lib.cpp (Library)

Encapsulates the core filtering algorithm independent of ROS.

Responsibilities:

Filter points that are inside a configurable 2D polygon (representing the robot footprint).

Remove points beyond a configurable maximum range.

Apply an angular field-of-view (FOV) gate based on configurable center and width angles.

Design rationale:
By separating the core logic into a library, it becomes unit-testable without ROS. This allows validating the filtering algorithm with any pointcloud data in isolation.

Implementation details:

Polygon checking uses a ray-casting algorithm to determine if a point is inside the footprint polygon.

Distance filtering is done by comparing the squared distance of each point to the origin.

FOV filtering converts the point coordinates to polar angles and removes points outside the configured angular range.

### 2. proximity_filter_node.cpp (ROS Node)

Handles ROS I/O and wiring:

Subscribes to sensor_msgs/msg/PointCloud2 input topic.

Publishes filtered pointcloud and the polygon footprint (geometry_msgs/msg/PolygonStamped) for visualization.

Live parameter updates:
Uses add_on_set_parameters_callback to dynamically update polygon vertices, FOV, and range without restarting the node.

Reasoning:
Separating ROS I/O from the library allows:

Easy reuse of the library in other projects or simulations.

Focus on ROS-specific tasks (QoS, topic management, throttling).

Scenario assumptions:

The robot moves forward in a typical navigation scenario, so the polygon footprint represents the robot body.

Pointclouds are generated in a fixed "map" frame, while the robot’s footprint is considered relative to its local frame. This allows flexible placement of the FOV and polygon for testing different scanning angles.

Input pointclouds are simulated around the robot in 360° or partial coverage, so the FOV and angle parameters are configurable.

### 3. Launch and Testing

proximity_filter_launch.py binds the publisher and filter node for easy testing.

test_pointcloud_pub.py publishes a pointcloud in arbitrary positions around the robot to simulate real LiDAR scans.

The design allows testing both in ROS (live simulation) and independently via unit tests.

Summary of design philosophy:

Library / Node separation: ensures modularity and testability.

Configurable parameters and FOV: simulate different sensors or robot orientations.

Ray-casting polygon check: efficient and general solution for footprint-based filtering.

ROS I/O handling: ensures reliable QoS for output and latched visualization of the footprint.


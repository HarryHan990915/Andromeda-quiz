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
If you want to run it, you need to comment out the following codes in CmakeLists.txt, to avoid colon code sanity check

```bash
##find_package(ament_lint_auto REQUIRED)
##ament_lint_auto_find_test_dependencies()
```

```bash
colcon test --packages-select andromeda_proximity_filter
colcon test-result --verbose
```
<img src="test_result.png" alt="Test result" width="600"/>

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

### 4. Design trade offs

The main trade-off I made was simplifying coordinate frames and motion assumptions in order to focus on correctness, testability, and a clean separation between ROS integration and core filtering logic. The node operates in a fixed map frame and assumes the incoming point cloud is already expressed near the robot reference frame, without explicitly handling TF transforms or robot pose updates. This avoids introducing TF dependencies and time-synchronization complexity in a take-home setting, while keeping the filtering behavior deterministic and easy to reason about. In a full robot system, the point cloud would typically be transformed into base_link before applying the footprint and FOV filtering. I also chose conservative QoS settings: SensorDataQoS for the input point cloud to match typical LiDAR behavior, Reliable for the filtered output to ensure downstream consumers such as planners or visualizers do not miss critical data, and Transient Local for the polygon footprint to support latched visualization. The trade-off is that reliable delivery can increase latency at very high publish rates, but this was an intentional choice favoring robustness and safety over minimal delay.

### 5. What's next

I focused on completing Part A properly first, because it’s the core perception component and closest to real robotics work I’ve done before. Besides, I currently do not have a Linux system. With Windows system, I tried virtual machine and found its too laggy with Rviz, and made my debugging really slow and then I ended up with using WSL in Windows system. I spent about 5 hours on environment, so that I may only focus on Part A. I made sure the filtering logic was clean, testable outside ROS, supported live parameter updates, and followed reasonable ROS 2 QoS and node design patterns.

I didn’t get to Part B due to time, but the next thing I’d do is implement it as a small ROS-agnostic C++ utility with a simple state machine. I’d track how long the current stays over the limit using the provided timestep, enter a degraded state after a configurable duration, and reduce the effective limit. Once the load clears, I’d add a cooldown and gradually restore the limit step by step to avoid oscillation. I’d keep it deterministic and cover it with Catch2 tests for overload, recovery, and edge cases.

I chose this order because Part A touches more on system integration and runtime behavior, while Part B is self-contained and can be added cleanly afterward without affecting the rest of the system. But if I have linux system, I would finish Part B and improve part A's defects with more depth and potentially made this application more realistic which can be used in our life.


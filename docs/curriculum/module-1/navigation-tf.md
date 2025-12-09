# Navigation & Coordinate Frames

**Week 3: tf2, URDF, and Navigation2**

## Overview

Mobile robots operate in 3D space with multiple coordinate frames: robot base, sensors, world frame, map frame. Understanding how to manage these transforms and build autonomous navigation systems is fundamental to robotics. This chapter covers tf2 transforms, URDF robot descriptions, and the Navigation2 stack.

## Learning Objectives

By the end of this chapter, you will:
- Understand coordinate frames and transformations in robotics
- Use tf2 to broadcast and listen to transforms
- Build robot models using URDF and xacro
- Configure Navigation2 for autonomous navigation
- Implement waypoint-following behaviors
- Tune navigation parameters for optimal performance

## Coordinate Frames in Robotics

### Why Multiple Frames?

Robots need different reference frames for:
- **World Frame**: Fixed reference (map origin, GPS coordinates)
- **Map Frame**: Localization reference (SLAM map origin)
- **Odom Frame**: Odometry reference (continuous, drifts over time)
- **Base Frame**: Robot's center (moves with robot)
- **Sensor Frames**: Camera, LIDAR, IMU (mounted on robot)

### REP-103: Standard Frames Convention

ROS follows REP-103 for coordinate frame conventions:
- **x**: Forward
- **y**: Left
- **z**: Up
- **Right-handed coordinate system**

```
         z (up)
         │
         │
         │
         └────── x (forward)
        ╱
       ╱
      y (left)
```

## tf2: Transform Library

### What is tf2?

tf2 is ROS 2's transform library that:
- Maintains a transform tree between all coordinate frames
- Provides time-synchronized transform lookups
- Handles complex transform chains automatically
- Supports extrapolation and interpolation

### Transform Tree Example

```
map
 └─ odom
     └─ base_link
         ├─ camera_link
         │   └─ camera_optical_frame
         ├─ laser_link
         └─ imu_link
```

### Broadcasting Static Transforms

For fixed relationships (e.g., sensor mounted on robot):

```python
# static_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_frame_publisher')

        # Create static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Broadcast camera transform
        self.broadcast_camera_transform()

        # Broadcast laser transform
        self.broadcast_laser_transform()

    def broadcast_camera_transform(self):
        """Broadcast camera position relative to base_link"""
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'

        # Camera mounted 30cm forward, 20cm up from base
        transform.transform.translation.x = 0.3
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.2

        # Camera tilted down 15 degrees
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = math.sin(-15 * math.pi / 360)
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = math.cos(-15 * math.pi / 360)

        self.tf_static_broadcaster.sendTransform(transform)
        self.get_logger().info('Camera transform broadcasted')

    def broadcast_laser_transform(self):
        """Broadcast LIDAR position relative to base_link"""
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_link'

        # LIDAR at robot center, 15cm up
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.15

        # No rotation (aligned with base_link)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(transform)
        self.get_logger().info('Laser transform broadcasted')

def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Broadcasting Dynamic Transforms

For moving relationships (e.g., robot position in world):

```python
# dynamic_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import math

class DynamicFramePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_frame_publisher')

        # Create dynamic transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        """Broadcast odom → base_link transform from odometry"""
        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Translation from odometry
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Rotation from odometry
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = DynamicFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Listening to Transforms

```python
# tf_listener.py
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
import math

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically look up transforms
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Look up transform from camera to base_link"""
        try:
            # Look up transform (with timeout)
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                'camera_link',  # Source frame
                rclpy.time.Time(),  # Get latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Extract translation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            self.get_logger().info(
                f'Camera is at ({x:.2f}, {y:.2f}, {z:.2f}) '
                f'relative to base_link'
            )

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {str(e)}')

    def transform_point(self, point, from_frame, to_frame):
        """Transform a point from one frame to another"""
        try:
            # Create point stamped
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = point[0]
            point_stamped.point.y = point[1]
            point_stamped.point.z = point[2]

            # Transform point
            transformed_point = self.tf_buffer.transform(
                point_stamped,
                to_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            return [
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z
            ]

        except Exception as e:
            self.get_logger().error(f'Transform failed: {str(e)}')
            return None

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Command-Line TF Tools

```bash
# View current transform tree
ros2 run tf2_tools view_frames

# Echo transform between frames
ros2 run tf2_ros tf2_echo map base_link

# Print full TF tree
ros2 run tf2_ros tf2_monitor
```

## URDF: Robot Description

### What is URDF?

Unified Robot Description Format (URDF) defines:
- Robot structure (links and joints)
- Physical properties (mass, inertia)
- Visual appearance (meshes, colors)
- Collision geometry
- Sensor locations

### Basic URDF Structure

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Laser Link -->
  <link name="laser_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to laser -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
  </joint>

</robot>
```

### Using Xacro for Modularity

Xacro extends URDF with:
- Variables and properties
- Mathematical expressions
- Macros for reusable components
- File includes

```xml
<!-- robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.03"/>
  <xacro:property name="base_length" value="0.4"/>

  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder length="${wheel_width}" radius="${wheel_radius}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      </visual>

      <collision>
        <geometry>
          <cylinder length="${wheel_width}" radius="${wheel_radius}"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
      </collision>

      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                 iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${base_length/2} ${reflect*0.15} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} 0.3 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

### Visualizing URDF in RViz

```bash
# Check URDF validity
check_urdf my_robot.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf

# With xacro
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf.xacro
```

## Navigation2: Autonomous Navigation

### What is Navigation2?

Navigation2 (Nav2) is the ROS 2 navigation stack that provides:
- **Path Planning**: Find collision-free paths to goals
- **Local Control**: Follow paths while avoiding dynamic obstacles
- **Recovery Behaviors**: Handle navigation failures
- **Lifecycle Management**: Robust state management

### Nav2 Architecture

```
Goal → Planner Server → Path → Controller Server → cmd_vel
         ↑                         ↑
         │                         │
     Costmap 2D ←────── Sensor Data (laser, camera)
     (Global & Local)
```

### Key Components

1. **Planner Server**: Computes global path from start to goal
2. **Controller Server**: Follows path with local obstacle avoidance
3. **Costmap**: Represents environment obstacles
4. **BT Navigator**: Behavior tree for navigation logic
5. **Recovery Server**: Handles stuck situations

### Setting Up Navigation2

#### 1. Install Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

#### 2. Basic Nav2 Configuration

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # DWB Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

#### 3. Launch Navigation2

```python
# nav2_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_nav')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Localization (AMCL)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file]
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params_file]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]
            }]
        )
    ])
```

### Sending Navigation Goals Programmatically

```python
# nav_goal_sender.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

class NavigationGoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')

        # Create Basic Navigator
        self.navigator = BasicNavigator()

        # Wait for Nav2 to be active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')

    def send_goal(self, x, y, yaw):
        """Send navigation goal to Nav2"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal_pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(f'Sending goal to ({x}, {y}, {yaw})')
        self.navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m')

        # Check result
        result = self.navigator.getResult()
        if result == 3:  # SUCCEEDED
            self.get_logger().info('Goal reached!')
        else:
            self.get_logger().warn('Navigation failed')

    def send_waypoints(self, waypoints):
        """Navigate through a sequence of waypoints"""
        waypoint_poses = []

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            waypoint_poses.append(pose)

        self.navigator.followWaypoints(waypoint_poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Waypoint {feedback.current_waypoint} of {len(waypoints)}')

        self.get_logger().info('All waypoints completed!')

def main():
    rclpy.init()
    node = NavigationGoalSender()

    # Send single goal
    node.send_goal(2.0, 3.0, 0.0)

    # Send waypoint sequence
    waypoints = [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 1.57),
        (1.0, 2.0, 3.14),
        (0.0, 1.0, -1.57),
        (0.0, 0.0, 0.0)
    ]
    node.send_waypoints(waypoints)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 3.1: URDF Robot Model

### Objective
Create a complete URDF model for a differential-drive robot with sensors.

### Requirements
1. Chassis with proper dimensions and inertia
2. Two driven wheels (continuous joints)
3. Caster wheel (fixed or spherical joint)
4. Laser scanner on top
5. Camera mounted at front
6. Use xacro for modularity

### Validation
```bash
check_urdf my_robot.urdf
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## Lab 3.2: TF Broadcasting

### Objective
Create a node that publishes complete TF tree for your robot.

### Requirements
1. Broadcast static transforms for all sensors
2. Broadcast dynamic transforms from odometry
3. Verify transforms with `view_frames`
4. Write a listener node to transform points between frames

## Lab 3.3: Nav2 Configuration

### Objective
Set up Navigation2 for autonomous waypoint navigation.

### Requirements
1. Configure Nav2 with appropriate parameters
2. Tune costmap settings for your robot
3. Create launch file for complete navigation stack
4. Test with 5-waypoint navigation sequence
5. Achieve >90% success rate

### Testing
```bash
# Launch navigation
ros2 launch my_robot_nav navigation.launch.py

# Send goal via CLI
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0}}}"
```

## Best Practices

### TF Management
- Always use `lookup_transform` with timeout
- Handle transform exceptions gracefully
- Keep TF tree simple and well-organized
- Use static transforms for fixed relationships

### URDF Design
- Validate URDF before testing
- Use realistic inertial properties
- Keep coordinate frame conventions consistent (REP-103)
- Use xacro for reusable robot components

### Navigation Tuning
- Start with conservative velocity limits
- Tune global planner first, then local controller
- Adjust costmap inflation based on robot size
- Test recovery behaviors thoroughly

## Summary

This chapter covered:
- **tf2**: Managing coordinate frame transforms
- **URDF**: Describing robot structure and properties
- **Xacro**: Creating modular robot descriptions
- **Navigation2**: Autonomous navigation framework
- **Path Planning**: Finding collision-free routes
- **Local Control**: Following paths with obstacle avoidance

With these tools, you can build robots that navigate autonomously in complex environments!

---

**Next**: [Module 2: Digital Twin & Gazebo Simulation →](../module-2/index)
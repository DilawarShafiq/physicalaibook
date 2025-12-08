---
title: Gazebo Simulation Fundamentals
sidebar_label: Gazebo Fundamentals
description: Learn the fundamentals of Gazebo simulation for robotics applications
keywords: [gazebo, simulation, robotics, physics engine, robot modeling]
---

# Gazebo Simulation Fundamentals

Gazebo is the leading robotics simulation environment, providing physics-accurate virtual worlds to test and validate Physical AI systems before real-world deployment.

## Introduction to Gazebo

**Gazebo** is a 3D simulation environment that provides:
- **Physics Engine**: Accurate modeling of rigid body dynamics
- **Sensor Simulation**: Realistic camera, lidar, IMU, and other sensors
- **Scene Rendering**: Photorealistic graphics for computer vision
- **Robot Models**: Standard URDF/XACRO robot definitions
- **Plugin System**: Extensible robot and sensor behaviors

### Two Gazebo Versions

In 2020, Open Robotics released a major overhaul:
- **Gazebo Classic** (formerly gazebo): Traditional C++11 physics engine
- **Gazebo Sim** (formerly Ignition): Modern modular architecture

**For this course we recommend Gazebo Garden/Classic** as it has matured significantly for robotics applications.

## Core Concepts

### 1. Worlds & Environments

**Worlds** define the simulated environment:
- **Static geometry**: Walls, floors, furniture
- **Lighting**: Sun, spotlights, ambient lighting 
- **Physics properties**: Gravity, real-time factor, solver parameters
- **Models**: Robots, objects, props

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="maze_world">
    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

:::tip
Create worlds that closely match your target real-world environment to minimize the sim-to-real transfer gap.
:::

### 2. Physics Configuration

Gazebo's physics engine requires careful tuning:

```xml
<physics type="ode">
  <!-- Real-time update rate -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor (1.0 = real-time) -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Solver parameters -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Physics Tuning Guidelines**:
- **Small step size**: Higher accuracy but slower simulation (0.001-0.01)
- **Real-time factor**: 1.0 for real-time, higher for faster simulation
- **Solver iterations**: More iterations = higher accuracy but lower performance

### 3. Robot Modeling with URDF

Gazebo integrates seamlessly with ROS 2's URDF (Unified Robot Description Format):

```xml
<?xml version="1.0"?>
<robot name="turtlebot3_burger"
  xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.010" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <origin xyz="-0.032 0 0.060" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="-0.032 0 0.060" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="8.2573504e-01"/>
      <inertia ixx="2.2124626e-03" ixy="0.0" ixz="0.0"
               iyy="2.1125753e-03" iyz="0.0"
               izz="2.0064271e-03"/>
    </inertial>
  </link>

  <!-- Sensors -->
  <gazebo reference="base_scan">
    <sensor name="laser_scanner" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
      </ray>
      <plugin name="gazebo_ros_laser_scan" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/tb3_0</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Hands-On Lab: Create Your First Gazebo Environment

Create a simple maze world to test navigation algorithms:

### Step 1: Create World File
Create `maze.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="maze">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Maze walls -->
    <model name="maze_wall_1">
      <pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>6 0.2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.2 1</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>
    
    <!-- Add more walls to form a maze -->
  </world>
</sdf>
```

### Step 2: Launch Script
Create `launch/maze_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze.world',
        description='Choose one of the world files from `/maze_simulation/worlds`'
    )

    gzserver_cmd = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[world],
        on_exit='ignore'
    )

    gzclient_cmd = Node(
        package='gazebo_ros',
        executable='gzclient',
        respawn=False,
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
```

:::lab-container
<div class="lab-title">
    <span class="lab-title-icon">🎯</span>
    <strong>Lab Exercise: Navigation in Gazebo</strong>
</div>

**Objective**: Implement autonomous navigation in your maze environment

**Requirements**:
1. Create a custom maze world in Gazebo
2. Spawn a differential drive robot 
3. Implement navigation stack to reach waypoints
4. Validate performance in simulation before considering real-world deployment

**Steps**:
1. Design maze layout with walls and obstacles
2. Create URDF for your robot with differential drive and sensors
3. Implement costmap-based navigation
4. Benchmark success rate in simulation
5. Document sim-to-real transfer considerations

<details>
<summary>💡 Hint</summary>

Use the Navigation2 stack for ROS 2 navigation. Consider domain randomization to improve sim-to-real transfer.
</details>
:::

## Advanced Topics

### Domain Randomization

To improve sim-to-real transfer, implement domain randomization:

```python
# Randomize lighting conditions
def randomize_lighting(world_model):
    lights = world_model.get_lights()
    for light in lights:
        intensity = random.uniform(0.5, 1.0)
        direction_noise = np.random.normal(0, 0.1, 3)
        light.set_intensity(intensity)
        light.set_direction(light.direction + direction_noise)
```

### Multi-Robot Simulation

Gazebo excels at multi-robot simulation:

```xml
<!-- In your world file -->
<model name="robot_1">
  <pose>0 0 0 0 0 0</pose>
  <!-- Robot 1 definition -->
</model>

<model name="robot_2">
  <pose>1 1 0 0 0 0</pose>
  <!-- Robot 2 definition with different namespace -->
</model>
```

## Best Practices

### ✅ Do:
- Use realistic friction and damping coefficients
- Include sensor noise models that match real hardware
- Validate physics parameters against real-world robot behavior
- Implement proper joint limits and safety controllers
- Use simulation-specific launch files with remappings

### ❌ Don't:
- Use perfect sensing (no noise in simulation)
- Ignore computational delays in control loops
- Skip sim-to-real validation before hardware deployment
- Neglect to tune physics parameters to match real robot
- Use different URDF versions between sim and real

## Troubleshooting Tips

### Simulation Performance
```bash
# Monitor real-time factor
gz stats

# Reduce physics complexity for better performance
# - Lower update rates for non-critical sensors
# - Simplify collision geometries
# - Use less complex physics meshes
```

### Sensor Issues
```bash
# Check sensor topics
ros2 topic list | grep /sensor

# Echo sensor data
ros2 topic echo /robot1/lidar_scan

# Verify sensor plugin configuration
gz model -m robot1 -i  # Get model info
```

## Summary

Gazebo simulation provides the essential "Digital Twin" capability for Physical AI development. By mastering Gazebo fundamentals, you'll be able to:
- Test algorithms safely before hardware deployment
- Perform rapid iteration cycles with fast simulation
- Validate sim-to-real transfer strategies
- Benchmark different approaches in controlled environments

Next, we'll explore advanced Gazebo features and integration with NVIDIA Isaac for photorealistic simulation.

---

**Next**: [Module 2: Physics Tuning & Sensor Integration →](./physics-tuning)
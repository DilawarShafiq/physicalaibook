# Multi-Robot Systems & Advanced Simulation

**Week 6: Coordination, Optimization, and CI/CD**

## Overview

As robot systems scale from single agents to fleets, new challenges emerge: coordination, communication, resource allocation, and testing at scale. This chapter covers multi-robot simulation, coordination strategies, performance optimization, and integrating simulation into continuous integration pipelines.

## Learning Objectives

By the end of this chapter, you will:
- Deploy and coordinate multiple robots in simulation
- Implement inter-robot communication and task allocation
- Optimize simulation performance for large-scale scenarios
- Set up headless simulation for automated testing
- Integrate robot simulation into CI/CD pipelines
- Design and test swarm behaviors

## Multi-Robot Coordination

### Namespace Management in ROS 2

When running multiple robots, each needs unique topic names and node names:

```python
# multi_robot_spawner.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os

class MultiRobotSpawner(Node):
    def __init__(self):
        super().__init__('multi_robot_spawner')

        # Create spawn service client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait for Gazebo
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')

    def spawn_robot(self, robot_name: str, namespace: str, x: float, y: float):
        """Spawn robot with unique namespace"""
        # Load robot description
        pkg_dir = get_package_share_directory('my_robot_description')
        urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')

        with open(urdf_path, 'r') as f:
            robot_description = f.read()

        # Create spawn request
        request = SpawnEntity.Request()
        request.name = robot_name
        request.xml = robot_description
        request.robot_namespace = namespace
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = 0.1

        # Spawn robot
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Spawned {robot_name} in namespace {namespace}')
        else:
            self.get_logger().error(f'Failed to spawn {robot_name}')

    def spawn_fleet(self, num_robots: int, formation: str = 'line'):
        """Spawn fleet of robots in formation"""
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            namespace = f'/robot_{i}'

            # Calculate position based on formation
            if formation == 'line':
                x, y = i * 2.0, 0.0
            elif formation == 'grid':
                x = (i % 5) * 2.0
                y = (i // 5) * 2.0
            elif formation == 'circle':
                import math
                angle = 2 * math.pi * i / num_robots
                radius = 5.0
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
            else:
                x, y = 0.0, 0.0

            self.spawn_robot(robot_name, namespace, x, y)

def main():
    rclpy.init()
    spawner = MultiRobotSpawner()

    # Spawn 10 robots in grid formation
    spawner.spawn_fleet(10, formation='grid')

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File for Multi-Robot System

```python
# multi_robot_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_robot_launch(robot_id: int):
    """Generate launch configuration for single robot"""
    namespace = f'robot_{robot_id}'

    return GroupAction([
        PushRosNamespace(namespace),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': '$(find my_robot)/urdf/robot.urdf',
                'frame_prefix': f'{namespace}/'
            }]
        ),

        # Navigation
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='navigation',
            parameters=[{
                'use_namespace': True,
                'namespace': namespace
            }]
        ),

        # Robot-specific controller
        Node(
            package='my_robot_control',
            executable='robot_controller',
            name='controller',
            parameters=[{
                'robot_id': robot_id,
                'max_speed': 1.0
            }]
        )
    ])

def generate_launch_description():
    # Declare arguments
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='5',
        description='Number of robots to spawn'
    )

    num_robots = LaunchConfiguration('num_robots')

    # Generate robot launches
    robot_launches = []
    for i in range(5):  # Default 5 robots
        robot_launches.append(generate_robot_launch(i))

    return LaunchDescription([
        num_robots_arg,
        *robot_launches,

        # Fleet coordinator
        Node(
            package='my_fleet_manager',
            executable='fleet_coordinator',
            name='fleet_coordinator',
            parameters=[{
                'num_robots': num_robots
            }]
        )
    ])
```

## Inter-Robot Communication

### Centralized Coordination

```python
# fleet_coordinator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from typing import Dict, List
import json

class FleetCoordinator(Node):
    """Centralized coordinator for multi-robot system"""

    def __init__(self):
        super().__init__('fleet_coordinator')

        self.declare_parameter('num_robots', 5)
        self.num_robots = self.get_parameter('num_robots').value

        # Track robot states
        self.robot_poses: Dict[int, PoseStamped] = {}
        self.robot_status: Dict[int, str] = {}
        self.task_queue: List[dict] = []
        self.assigned_tasks: Dict[int, dict] = {}

        # Subscribe to robot poses
        self.pose_subs = []
        for i in range(self.num_robots):
            sub = self.create_subscription(
                PoseStamped,
                f'/robot_{i}/pose',
                lambda msg, robot_id=i: self.pose_callback(msg, robot_id),
                10
            )
            self.pose_subs.append(sub)

        # Publish task assignments
        self.task_pubs = []
        for i in range(self.num_robots):
            pub = self.create_publisher(
                String,
                f'/robot_{i}/task',
                10
            )
            self.task_pubs.append(pub)

        # Coordination timer
        self.create_timer(1.0, self.coordinate_fleet)

        self.get_logger().info(f'Fleet coordinator managing {self.num_robots} robots')

    def pose_callback(self, msg: PoseStamped, robot_id: int):
        """Update robot pose"""
        self.robot_poses[robot_id] = msg

    def add_task(self, task: dict):
        """Add task to queue"""
        self.task_queue.append(task)
        self.get_logger().info(f'Task added: {task}')

    def assign_task_to_robot(self, robot_id: int, task: dict):
        """Assign task to specific robot"""
        self.assigned_tasks[robot_id] = task

        # Publish task
        task_msg = String()
        task_msg.data = json.dumps(task)
        self.task_pubs[robot_id].publish(task_msg)

        self.get_logger().info(f'Assigned task to robot_{robot_id}: {task}')

    def find_nearest_robot(self, target_x: float, target_y: float) -> int:
        """Find nearest available robot to target location"""
        min_dist = float('inf')
        nearest_robot = -1

        for robot_id, pose in self.robot_poses.items():
            # Skip if robot is busy
            if robot_id in self.assigned_tasks:
                continue

            # Calculate distance
            dx = pose.pose.position.x - target_x
            dy = pose.pose.position.y - target_y
            dist = (dx**2 + dy**2) ** 0.5

            if dist < min_dist:
                min_dist = dist
                nearest_robot = robot_id

        return nearest_robot

    def coordinate_fleet(self):
        """Main coordination logic"""
        # Assign tasks to available robots
        while self.task_queue and len(self.assigned_tasks) < self.num_robots:
            task = self.task_queue.pop(0)

            # Find best robot for task
            robot_id = self.find_nearest_robot(
                task['target_x'],
                task['target_y']
            )

            if robot_id >= 0:
                self.assign_task_to_robot(robot_id, task)
            else:
                # No available robot, put task back
                self.task_queue.insert(0, task)
                break

        # Check for completed tasks
        self.check_task_completion()

    def check_task_completion(self):
        """Check if robots have completed their tasks"""
        completed = []

        for robot_id, task in self.assigned_tasks.items():
            if robot_id in self.robot_poses:
                pose = self.robot_poses[robot_id]

                # Check if robot reached target
                dx = pose.pose.position.x - task['target_x']
                dy = pose.pose.position.y - task['target_y']
                dist = (dx**2 + dy**2) ** 0.5

                if dist < 0.5:  # Within 50cm
                    completed.append(robot_id)
                    self.get_logger().info(f'robot_{robot_id} completed task')

        # Remove completed tasks
        for robot_id in completed:
            del self.assigned_tasks[robot_id]

def main():
    rclpy.init()
    coordinator = FleetCoordinator()

    # Add some example tasks
    coordinator.add_task({'target_x': 5.0, 'target_y': 3.0, 'action': 'deliver'})
    coordinator.add_task({'target_x': -2.0, 'target_y': 4.0, 'action': 'pickup'})
    coordinator.add_task({'target_x': 0.0, 'target_y': -5.0, 'action': 'inspect'})

    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Decentralized Coordination

```python
# decentralized_agent.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json

class DecentralizedAgent(Node):
    """Robot agent with decentralized coordination"""

    def __init__(self, robot_id: int):
        super().__init__(f'agent_{robot_id}')

        self.robot_id = robot_id
        self.my_pose = None
        self.nearby_robots = {}
        self.current_task = None

        # Broadcast own pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/robot_{robot_id}/pose',
            10
        )

        # Subscribe to other robots' poses
        self.pose_subs = []
        for i in range(10):  # Assume max 10 robots
            if i != robot_id:
                sub = self.create_subscription(
                    PoseStamped,
                    f'/robot_{i}/pose',
                    lambda msg, rid=i: self.other_pose_callback(msg, rid),
                    10
                )
                self.pose_subs.append(sub)

        # Task negotiation
        self.task_announce_pub = self.create_publisher(
            String, '/task_announcements', 10
        )
        self.task_announce_sub = self.create_subscription(
            String, '/task_announcements', self.task_announcement_callback, 10
        )

        self.timer = self.create_timer(0.5, self.decision_loop)

    def other_pose_callback(self, msg: PoseStamped, robot_id: int):
        """Track other robots' positions"""
        self.nearby_robots[robot_id] = msg

    def task_announcement_callback(self, msg: String):
        """Handle task announcements from other robots"""
        announcement = json.loads(msg.data)

        if announcement['type'] == 'bid':
            # Another robot is bidding on a task
            self.handle_task_bid(announcement)
        elif announcement['type'] == 'claim':
            # Another robot claimed a task
            self.handle_task_claim(announcement)

    def avoid_collisions(self):
        """Implement collision avoidance with nearby robots"""
        if not self.my_pose:
            return

        for robot_id, pose in self.nearby_robots.items():
            # Calculate distance to other robot
            dx = pose.pose.position.x - self.my_pose.pose.position.x
            dy = pose.pose.position.y - self.my_pose.pose.position.y
            dist = (dx**2 + dy**2) ** 0.5

            # If too close, adjust trajectory
            if dist < 1.0:  # 1 meter safety distance
                self.get_logger().warn(
                    f'Too close to robot_{robot_id}! Distance: {dist:.2f}m'
                )
                # Implement avoidance maneuver
                # (actual implementation would adjust velocity commands)

    def decision_loop(self):
        """Main decision loop"""
        # Update situational awareness
        self.avoid_collisions()

        # Task selection and execution
        if not self.current_task:
            # Look for new tasks
            self.find_and_bid_on_task()
        else:
            # Execute current task
            self.execute_task()

def main():
    import sys
    rclpy.init()

    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    agent = DecentralizedAgent(robot_id)

    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Performance Optimization

### Parallel Physics Simulation

```xml
<!-- optimized_world.sdf -->
<sdf version="1.7">
  <world name="optimized_world">

    <!-- Physics optimization -->
    <physics name="fast_physics" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>

      <!-- ODE solver optimization -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
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

    <!-- Use simplified collision geometry -->
    <model name="optimized_robot">
      <link name="base">
        <!-- Visual (detailed) -->
        <visual name="visual">
          <geometry>
            <mesh><uri>model://detailed_robot.dae</uri></mesh>
          </geometry>
        </visual>

        <!-- Collision (simplified) -->
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```

### Headless Simulation

```python
# headless_simulation.py
import rclpy
from rclpy.node import Node
import subprocess
import os

class HeadlessSimulation(Node):
    """Run Gazebo in headless mode for automated testing"""

    def __init__(self):
        super().__init__('headless_simulation')

        # Launch Gazebo headless (no GUI)
        self.gazebo_process = subprocess.Popen([
            'gzserver',  # Server only, no client
            '--verbose',
            '/path/to/world.sdf'
        ], env={
            **os.environ,
            'LIBGL_ALWAYS_SOFTWARE': '1',  # Software rendering
            'GAZEBO_MASTER_URI': 'http://localhost:11345'
        })

        self.get_logger().info('Headless simulation started')

    def shutdown(self):
        """Clean shutdown of simulation"""
        self.gazebo_process.terminate()
        self.gazebo_process.wait()
        self.get_logger().info('Simulation stopped')

def main():
    rclpy.init()
    sim = HeadlessSimulation()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        pass
    finally:
        sim.shutdown()
        sim.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## CI/CD Integration

### GitHub Actions Workflow

```yaml
# .github/workflows/robot_simulation_test.yml
name: Robot Simulation Tests

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  simulation_tests:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Setup ROS 2 Humble
      uses: ros-tooling/setup-ros@v0.6
      with:
        required-ros-distributions: humble

    - name: Install Dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y \
          ros-humble-gazebo-ros-pkgs \
          ros-humble-navigation2 \
          ros-humble-nav2-bringup \
          python3-colcon-common-extensions

    - name: Build Workspace
      run: |
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install

    - name: Run Simulation Tests
      run: |
        source /opt/ros/humble/setup.bash
        source install/setup.bash

        # Start headless Gazebo
        gzserver worlds/test_world.sdf &
        GAZEBO_PID=$!

        # Wait for Gazebo to start
        sleep 10

        # Run tests
        colcon test --packages-select my_robot_navigation
        colcon test-result --verbose

        # Cleanup
        kill $GAZEBO_PID

    - name: Upload Test Results
      if: always()
      uses: actions/upload-artifact@v3
      with:
        name: test-results
        path: build/*/test_results/
```

### Automated Performance Testing

```python
# performance_test.py
import rclpy
from rclpy.node import Node
import time
import psutil
import json

class PerformanceTest(Node):
    """Automated performance testing for multi-robot simulation"""

    def __init__(self):
        super().__init__('performance_test')

        self.metrics = {
            'real_time_factor': [],
            'cpu_usage': [],
            'memory_usage': [],
            'update_rate': []
        }

    def run_test(self, num_robots: int, duration: int = 60):
        """Run performance test with specified number of robots"""
        self.get_logger().info(f'Testing with {num_robots} robots')

        start_time = time.time()

        while time.time() - start_time < duration:
            # Measure CPU and memory
            self.metrics['cpu_usage'].append(psutil.cpu_percent())
            self.metrics['memory_usage'].append(
                psutil.virtual_memory().percent
            )

            # Query Gazebo real-time factor
            # (would require Gazebo API call)
            rtf = self.get_realtime_factor()
            self.metrics['real_time_factor'].append(rtf)

            time.sleep(1.0)

        # Compute statistics
        self.report_results(num_robots)

    def get_realtime_factor(self) -> float:
        """Get current real-time factor from Gazebo"""
        # Placeholder - actual implementation would query Gazebo
        return 1.0

    def report_results(self, num_robots: int):
        """Generate performance report"""
        import numpy as np

        report = {
            'num_robots': num_robots,
            'avg_rtf': np.mean(self.metrics['real_time_factor']),
            'min_rtf': np.min(self.metrics['real_time_factor']),
            'avg_cpu': np.mean(self.metrics['cpu_usage']),
            'max_cpu': np.max(self.metrics['cpu_usage']),
            'avg_memory': np.mean(self.metrics['memory_usage']),
            'max_memory': np.max(self.metrics['memory_usage'])
        }

        self.get_logger().info(f"\n=== Performance Report ===")
        self.get_logger().info(f"Robots: {report['num_robots']}")
        self.get_logger().info(f"Avg RTF: {report['avg_rtf']:.2f}")
        self.get_logger().info(f"CPU: {report['avg_cpu']:.1f}% (max: {report['max_cpu']:.1f}%)")
        self.get_logger().info(f"Memory: {report['avg_memory']:.1f}% (max: {report['max_memory']:.1f}%)")

        # Save to file
        with open(f'performance_{num_robots}_robots.json', 'w') as f:
            json.dump(report, f, indent=2)

def main():
    rclpy.init()
    tester = PerformanceTest()

    # Test with increasing number of robots
    for num_robots in [1, 5, 10, 20]:
        tester.run_test(num_robots, duration=60)
        time.sleep(10)  # Cool-down between tests

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 6.1: Multi-Robot Warehouse

### Objective
Deploy 5 robots in coordinated warehouse simulation.

### Requirements
1. Spawn 5 robots in warehouse environment
2. Implement centralized task coordinator
3. Robots navigate to pickup and delivery locations
4. Handle dynamic obstacle avoidance
5. Measure throughput (deliveries per hour)

## Lab 6.2: Headless Simulation for CI

### Objective
Set up headless Gazebo simulation for automated testing.

### Requirements
1. Configure headless Gazebo launch
2. Create automated test suite
3. Integrate with GitHub Actions
4. Generate test reports automatically

## Lab 6.3: Performance Benchmarking

### Objective
Benchmark simulation performance with varying robot counts.

### Requirements
1. Test with 1, 5, 10, 20 robots
2. Measure real-time factor, CPU, memory
3. Identify performance bottlenecks
4. Optimize and re-test

## Summary

This chapter covered:
- **Multi-Robot Coordination**: Centralized and decentralized approaches
- **Namespace Management**: Properly scoping multi-robot systems
- **Inter-Robot Communication**: Task allocation and conflict resolution
- **Performance Optimization**: Scaling simulation to many robots
- **Headless Simulation**: Running without GUI for automation
- **CI/CD Integration**: Automated testing in GitHub Actions

You now have the skills to build and test large-scale multi-robot systems!

---

**Next**: [Module 3: AI-Robot Brain & NVIDIA Isaac →](../module-3/index)
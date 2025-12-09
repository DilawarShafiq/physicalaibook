# Advanced ROS 2 Communication

**Week 2: Services, Actions, and Parameters**

## Overview

While publish/subscribe topics are great for continuous data streams, robotics systems often need more sophisticated communication patterns. This chapter covers ROS 2's advanced communication mechanisms for request/response interactions, long-running tasks, and runtime configuration.

## Learning Objectives

By the end of this chapter, you will:
- Implement ROS 2 services for synchronous request/response communication
- Create action servers and clients for long-running tasks with feedback
- Manage runtime parameters and dynamic reconfiguration
- Write launch files to orchestrate multi-node systems
- Record and replay data using ROS 2 bags

## Services: Request/Response Pattern

### What are Services?

Services provide synchronous, one-to-one communication for scenarios where you need a response:
- Triggering one-time operations (e.g., "take a picture now")
- Querying system state (e.g., "what's the current battery level?")
- Performing calculations (e.g., "compute inverse kinematics for this pose")

### Service Architecture

```
┌─────────────┐         Request          ┌─────────────┐
│   Client    │ ─────────────────────▶  │   Server    │
│             │                          │             │
│             │ ◀─────────────────────   │             │
└─────────────┘        Response          └─────────────┘
```

### Creating a Service Server

Let's create a simple calculator service:

```python
# calculator_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('Calculator service ready')

    def add_callback(self, request, response):
        """Handle incoming service requests"""
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )

        return response

def main():
    rclpy.init()
    server = CalculatorServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
# calculator_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.get_logger().info('Service available!')

    def send_request(self, a, b):
        """Send calculation request"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (synchronous)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Result: {result.sum}')
            return result.sum
        else:
            self.get_logger().error('Service call failed')
            return None

def main():
    rclpy.init()
    client = CalculatorClient()

    # Make some calculations
    client.send_request(5, 7)
    client.send_request(10, 20)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions: Long-Running Tasks

### What are Actions?

Actions are designed for tasks that:
- Take significant time to complete (seconds to minutes)
- Need to provide progress feedback
- May need to be cancelled mid-execution

Examples: navigation, pick-and-place, battery charging, mapping

### Action Architecture

```
┌─────────────┐                          ┌─────────────┐
│   Client    │  ──── Goal ────▶         │   Server    │
│             │                          │             │
│             │  ◀─── Feedback ────      │  (Executing │
│             │  ◀─── Feedback ────      │   action)   │
│             │  ◀─── Feedback ────      │             │
│             │                          │             │
│             │  ◀─── Result ────        │             │
└─────────────┘                          └─────────────┘
       │                                         ▲
       └──────────── Cancel ───────────────────┘
```

### Creating an Action Server

Let's create a navigation action that moves a robot to a goal position:

```python
# navigate_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,  # Using Fibonacci as example; replace with custom action
            'navigate_to_goal',
            self.execute_callback
        )

        self.get_logger().info('Navigation action server ready')

    def execute_callback(self, goal_handle):
        """Execute the navigation action"""
        self.get_logger().info('Executing navigation...')

        # Feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate navigation with progress updates
        for i in range(1, goal_handle.request.order):
            # Check if action was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation cancelled')
                return Fibonacci.Result()

            # Update progress
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Simulate work
            time.sleep(0.5)

        # Action completed successfully
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info('Navigation completed!')
        return result

def main():
    rclpy.init()
    server = NavigationActionServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

```python
# navigate_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('navigation_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'navigate_to_goal'
        )

    def send_goal(self, target_x, target_y):
        """Send navigation goal"""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10  # Example order

        # Wait for server
        self._action_client.wait_for_server()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.sequence}')

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main():
    rclpy.init()
    client = NavigationActionClient()
    client.send_goal(5.0, 3.0)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameters: Runtime Configuration

### What are Parameters?

Parameters allow you to configure node behavior at runtime without recompiling code:
- Tuning PID controllers
- Setting camera exposure
- Configuring thresholds and timeouts

### Using Parameters in a Node

```python
# configurable_node.py
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.get_logger().info(f'Max speed: {self.max_speed}')
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Debug mode: {self.debug_mode}')

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Handle parameter changes at runtime"""
        for param in params:
            if param.name == 'max_speed':
                self.max_speed = param.value
                self.get_logger().info(f'Max speed updated to: {self.max_speed}')
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(f'Debug mode: {self.debug_mode}')

        return rclpy.parameter.SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setting Parameters from Command Line

```bash
# Run node with custom parameters
ros2 run my_package configurable_node --ros-args \
  -p max_speed:=2.0 \
  -p robot_name:=turtlebot \
  -p debug_mode:=true

# Change parameters at runtime
ros2 param set /configurable_node max_speed 3.0
ros2 param get /configurable_node max_speed
ros2 param list
```

## Launch Files: System Orchestration

### What are Launch Files?

Launch files let you:
- Start multiple nodes with one command
- Set parameters for each node
- Configure remappings and namespaces
- Create complex system architectures

### Python Launch File

```python
# robot_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Name of the robot'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define nodes
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='camera',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_rate': 30.0
        }]
    )

    perception_node = Node(
        package='my_package',
        executable='perception',
        name='perception',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': robot_name
        }],
        remappings=[
            ('/image', '/camera/image_raw')
        ]
    )

    controller_node = Node(
        package='my_package',
        executable='controller',
        name='controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_speed': 1.0,
            'update_rate': 50.0
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        camera_node,
        perception_node,
        controller_node
    ])
```

### Running Launch Files

```bash
# Launch the system
ros2 launch my_package robot_system.launch.py

# With custom arguments
ros2 launch my_package robot_system.launch.py \
  robot_name:=turtlebot \
  use_sim_time:=true
```

## ROS 2 Bags: Data Recording

### What are ROS 2 Bags?

Bags record and replay ROS 2 topics for:
- Debugging robot behavior
- Training machine learning models
- Testing algorithms with real data
- Sharing datasets with collaborators

### Recording Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image_raw /scan /odom

# Record with custom name
ros2 bag record -o my_robot_data /camera/image_raw /scan

# Record for specific duration (seconds)
ros2 bag record -a --duration 60
```

### Playing Back Data

```bash
# Play recorded bag
ros2 bag play my_robot_data

# Play at different speed
ros2 bag play my_robot_data --rate 0.5  # Half speed
ros2 bag play my_robot_data --rate 2.0  # Double speed

# Play in loop
ros2 bag play my_robot_data --loop

# Get bag info
ros2 bag info my_robot_data
```

### Using Bags in Code

```python
import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

class BagReader(Node):
    def __init__(self):
        super().__init__('bag_reader')

        # Setup reader
        storage_options = StorageOptions(uri='my_robot_data', storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Read messages
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            self.get_logger().info(f'Topic: {topic}, Time: {timestamp}')
```

## Lab 2.1: Service Calculator

### Objective
Build a robot service that computes inverse kinematics for a robot arm.

### Requirements
1. Create a service server that takes end-effector position (x, y, z)
2. Compute joint angles (simplified 3-DOF calculation)
3. Return joint angles or error if position unreachable
4. Create client to test service with multiple positions

### Starter Code

```python
# ik_service.srv (create in srv/ folder)
# Request
float64 target_x
float64 target_y
float64 target_z
---
# Response
bool success
float64[] joint_angles
string message
```

## Lab 2.2: Navigation Action

### Objective
Create an action server that simulates robot navigation with progress feedback.

### Requirements
1. Action accepts goal position (x, y, theta)
2. Provides feedback every 0.5 seconds with distance remaining
3. Supports cancellation mid-navigation
4. Returns success/failure and final position

### Testing
```bash
# Use CLI to test action
ros2 action send_goal /navigate_to_goal <action_type> "{x: 5.0, y: 3.0, theta: 1.57}"
```

## Lab 2.3: Launch File Configuration

### Objective
Create a launch file for a multi-node robot system with configurable parameters.

### Requirements
1. Launch camera, perception, and control nodes
2. Support launch arguments for robot name and simulation mode
3. Load parameters from YAML file
4. Set up topic remappings for camera data
5. Include conditional node launching based on arguments

## Best Practices

### Service Design
- Keep service calls short (< 1 second if possible)
- Use actions for long-running tasks
- Provide meaningful error messages
- Validate inputs in service callback

### Action Design
- Provide regular feedback updates
- Handle cancellation gracefully
- Use timeouts to prevent hanging
- Return detailed result information

### Parameter Management
- Provide sensible defaults
- Validate parameter ranges
- Document parameters in code
- Use YAML files for complex configurations

### Launch Files
- Use descriptive argument names
- Provide default values
- Add comments explaining complex logic
- Organize nodes logically

## Common Pitfalls

1. **Blocking service calls**: Use `call_async()` instead of `call()` to avoid blocking
2. **Missing service checks**: Always wait for service availability
3. **Unhandled action cancellation**: Implement proper cancellation logic
4. **Parameter type mismatches**: Declare parameter types explicitly
5. **Launch file errors**: Test launch files incrementally

## Summary

This chapter covered ROS 2's advanced communication patterns:

- **Services**: Request/response for one-time operations
- **Actions**: Long-running tasks with feedback and cancellation
- **Parameters**: Runtime configuration and tuning
- **Launch Files**: System orchestration and deployment
- **ROS 2 Bags**: Data recording and playback

These tools enable you to build sophisticated, configurable robot systems with proper separation of concerns and robust communication patterns.

---

**Next**: [Navigation & Coordinate Frames →](./navigation-tf)
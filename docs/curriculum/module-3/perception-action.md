# End-to-End Perception-Action Pipelines

**Week 10: Integrating Vision with Control**

## Overview

Modern robotics combines perception, decision-making, and control into integrated pipelines. This chapter covers behavior trees, vision-guided manipulation, real-time optimization, and deploying complete systems to edge hardware.

## Learning Objectives

- Design behavior trees for robot decision-making
- Integrate computer vision with motion planning
- Implement visual servoing for precise manipulation
- Optimize pipelines for real-time performance
- Deploy to NVIDIA Jetson edge devices
- Handle failures and implement recovery behaviors

## Behavior Trees for Decision-Making

### BehaviorTree.CPP Integration

```python
# behavior_tree_node.py
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros

class PerceptionActionBT(Node):
    """Behavior tree for perception-action pipeline"""

    def __init__(self):
        super().__init__('perception_action_bt')

        # Build behavior tree
        self.tree = self.create_behavior_tree()

        # Setup ROS interfaces
        self.blackboard = py_trees.blackboard.Blackboard()

        # Timer for tree ticking
        self.create_timer(0.1, self.tick_tree)  # 10 Hz

    def create_behavior_tree(self):
        """Build behavior tree structure"""
        root = py_trees.composites.Sequence(
            name="Kitchen Assistant",
            memory=False
        )

        # Perception subtree
        perception = py_trees.composites.Selector(
            name="Perception",
            memory=False,
            children=[
                self.DetectObjectAction(name="Detect Target"),
                self.SearchForObjectAction(name="Search for Target")
            ]
        )

        # Approach subtree
        approach = py_trees.composites.Sequence(
            name="Approach",
            memory=False,
            children=[
                self.CheckReachableCondition(name="Is Reachable?"),
                self.NavigateToObjectAction(name="Navigate"),
                self.AlignWithObjectAction(name="Align")
            ]
        )

        # Manipulation subtree
        manipulation = py_trees.composites.Selector(
            name="Manipulation",
            memory=False,
            children=[
                self.PickObjectAction(name="Pick Object"),
                self.RecoverFromFailureAction(name="Recover")
            ]
        )

        # Complete sequence
        root.add_children([perception, approach, manipulation])

        return py_trees.trees.BehaviourTree(root)

    # Behavior implementations
    class DetectObjectAction(py_trees.behaviour.Behaviour):
        """Run object detection"""
        def update(self):
            # Query detection service
            detections = self.get_detections()  # ROS service call
            if detections:
                self.blackboard.set("target_object", detections[0])
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

    class NavigateToObjectAction(py_trees.behaviour.Behaviour):
        """Navigate to detected object"""
        def update(self):
            target = self.blackboard.get("target_object")
            # Send navigation goal
            success = self.navigate_to(target.pose)
            if success:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

    class PickObjectAction(py_trees.behaviour.Behaviour):
        """Grasp detected object"""
        def update(self):
            target = self.blackboard.get("target_object")
            # Execute grasp
            success = self.grasp_object(target)
            if success:
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

    def tick_tree(self):
        """Tick behavior tree"""
        self.tree.tick()

def main():
    rclpy.init()
    bt_node = PerceptionActionBT()
    rclpy.spin(bt_node)
    bt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Visual Servoing

### Image-Based Visual Servoing (IBVS)

```python
# visual_servo.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualServo(Node):
    """Image-based visual servoing for precise manipulation"""

    def __init__(self):
        super().__init__('visual_servo')

        self.bridge = CvBridge()

        # Camera parameters
        self.focal_length = 525.0  # pixels
        self.lambda_gain = 0.5  # Servo gain

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish velocity commands
        self.vel_pub = self.create_publisher(Twist, '/servo_vel', 10)

        # Target features (e.g., object corners in image)
        self.target_features = np.array([
            [320, 240],  # Image center
        ])

        self.get_logger().info('Visual servoing initialized')

    def image_callback(self, msg: Image):
        """Perform visual servoing"""
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Extract current features (e.g., detect object)
            current_features = self.extract_features(cv_image)

            if current_features is not None:
                # Compute feature error
                error = self.target_features - current_features

                # Compute image Jacobian
                L = self.compute_interaction_matrix(current_features)

                # Compute velocity command (v = -λ * L^+ * e)
                velocity = -self.lambda_gain * np.linalg.pinv(L) @ error.flatten()

                # Publish velocity
                vel_msg = Twist()
                vel_msg.linear.x = velocity[0]
                vel_msg.linear.y = velocity[1]
                vel_msg.linear.z = velocity[2]
                vel_msg.angular.x = velocity[3]
                vel_msg.angular.y = velocity[4]
                vel_msg.angular.z = velocity[5]

                self.vel_pub.publish(vel_msg)

                # Visualize
                for pt in current_features:
                    cv2.circle(cv_image, tuple(pt.astype(int)), 5, (0, 255, 0), -1)
                for pt in self.target_features:
                    cv2.circle(cv_image, tuple(pt.astype(int)), 5, (0, 0, 255), 2)

                cv2.imshow("Visual Servoing", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Visual servoing failed: {str(e)}')

    def extract_features(self, image):
        """Extract visual features from image"""
        # Example: detect ArUco marker
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            # Return marker center
            center = np.mean(corners[0][0], axis=0)
            return np.array([center])

        return None

    def compute_interaction_matrix(self, features):
        """Compute image Jacobian (interaction matrix)"""
        # Simplified for point features
        # Full implementation depends on feature type and camera model

        u, v = features[0]
        Z = 1.0  # Estimated depth (meters)
        f = self.focal_length

        # Interaction matrix for single point feature
        L = np.array([
            [-f/Z,    0,     u/Z,  u*v/f,   -(f + u**2/f),  v],
            [   0, -f/Z,     v/Z,  f + v**2/f,   -u*v/f,   -u]
        ])

        return L

def main():
    rclpy.init()
    servo = VisualServo()
    rclpy.spin(servo)
    servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Perception-Action Pipeline

### Integrated System

```python
# perception_action_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction
from action_msgs.msg import GoalStatus
import torch
from threading import Lock

class PerceptionActionPipeline(Node):
    """Complete pipeline from perception to action"""

    def __init__(self):
        super().__init__('perception_action_pipeline')

        self.lock = Lock()

        # Load perception model
        self.detection_model = torch.jit.load('/path/to/detection_model.pt')
        self.detection_model.eval()

        # State machine
        self.state = "DETECTING"
        self.target_object = None

        # ROS interfaces
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )

        self.grasp_client = self.create_client(
            MoveGroupAction, '/grasp_action'
        )

        # Timer for state machine
        self.create_timer(0.1, self.state_machine_update)

        self.get_logger().info('Perception-Action Pipeline initialized')

    def image_callback(self, msg: Image):
        """Process camera images"""
        if self.state == "DETECTING":
            with self.lock:
                # Run detection
                detections = self.run_detection(msg)

                if detections:
                    self.target_object = detections[0]
                    self.state = "PLANNING"

    def run_detection(self, image_msg):
        """Run object detection"""
        # Preprocess image
        image = self.preprocess_image(image_msg)

        # Inference
        with torch.no_grad():
            detections = self.detection_model(image)

        return self.postprocess_detections(detections)

    def state_machine_update(self):
        """Main state machine"""

        if self.state == "DETECTING":
            self.get_logger().info("Detecting objects...", throttle_duration_sec=1.0)

        elif self.state == "PLANNING":
            self.get_logger().info("Planning grasp...")

            # Compute grasp pose
            grasp_pose = self.compute_grasp_pose(self.target_object)

            # Send to motion planner
            self.execute_grasp(grasp_pose)
            self.state = "EXECUTING"

        elif self.state == "EXECUTING":
            # Check if grasp completed
            if self.is_grasp_complete():
                self.get_logger().info("Grasp completed successfully!")
                self.state = "DONE"

        elif self.state == "DONE":
            pass  # Task complete

    def compute_grasp_pose(self, detection):
        """Compute grasp pose from detection"""
        # Use grasp planning algorithm
        # (e.g., GraspNet, 6-DOF pose estimation)
        pass

    def execute_grasp(self, grasp_pose):
        """Execute grasp using motion planning"""
        # Call MoveIt! or similar motion planner
        pass

    def is_grasp_complete(self):
        """Check if grasp execution finished"""
        # Query grasp action status
        return False  # Placeholder

def main():
    rclpy.init()
    pipeline = PerceptionActionPipeline()
    rclpy.spin(pipeline)
    pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-Time Optimization

### Model Optimization with TensorRT

```python
# tensorrt_optimization.py
import torch
import torch_tensorrt

def optimize_model_for_jetson(model_path, input_shape):
    """Optimize PyTorch model with TensorRT"""

    # Load model
    model = torch.jit.load(model_path)
    model.eval()

    # Example input
    example_input = torch.randn(1, 3, *input_shape).cuda()

    # Compile with TensorRT
    trt_model = torch_tensorrt.compile(
        model,
        inputs=[torch_tensorrt.Input(
            shape=[1, 3, *input_shape],
            dtype=torch.float32
        )],
        enabled_precisions={torch.float16},  # FP16 for speed
        workspace_size=1 << 30,  # 1GB
    )

    # Save optimized model
    torch.jit.save(trt_model, model_path.replace('.pt', '_trt.pt'))

    print(f"Model optimized and saved")

    return trt_model

# Benchmark
def benchmark_model(model, input_shape, num_iterations=100):
    """Benchmark model inference time"""
    import time

    model = model.cuda()
    input_tensor = torch.randn(1, 3, *input_shape).cuda()

    # Warmup
    for _ in range(10):
        _ = model(input_tensor)

    # Benchmark
    torch.cuda.synchronize()
    start = time.time()

    for _ in range(num_iterations):
        _ = model(input_tensor)

    torch.cuda.synchronize()
    end = time.time()

    avg_time = (end - start) / num_iterations
    fps = 1.0 / avg_time

    print(f"Average inference time: {avg_time*1000:.2f} ms")
    print(f"Throughput: {fps:.2f} FPS")
```

## Deployment to Jetson

### Jetson Deployment Script

```bash
#!/bin/bash
# deploy_to_jetson.sh

# Jetson device IP
JETSON_IP="192.168.1.100"
JETSON_USER="nvidia"

echo "Deploying to NVIDIA Jetson at $JETSON_IP"

# Copy models
echo "Copying models..."
scp models/detection_trt.pt ${JETSON_USER}@${JETSON_IP}:~/robot/models/

# Copy ROS 2 packages
echo "Copying ROS 2 workspace..."
rsync -avz --exclude='build' --exclude='install' \
    ~/robot_ws ${JETSON_USER}@${JETSON_IP}:~/

# SSH and build
echo "Building on Jetson..."
ssh ${JETSON_USER}@${JETSON_IP} << 'EOF'
    cd ~/robot_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select perception_action_pipeline
    echo "Deployment complete!"
EOF

echo "Done! SSH to Jetson and run:"
echo "  ros2 launch perception_action_pipeline pipeline.launch.py"
```

## Lab 10.1: Behavior Tree Design

### Objective
Implement behavior tree for kitchen assistant robot.

### Requirements
1. Design tree with perception, navigation, manipulation
2. Implement recovery behaviors
3. Test with various task sequences
4. Measure success rate
5. Visualize tree execution

## Lab 10.2: Visual Servoing

### Objective
Implement visual servoing for precise alignment.

### Requirements
1. Detect target using ArUco markers or features
2. Compute image Jacobian
3. Servo to target pose
4. Achieve less than 5mm position accuracy
5. Test with moving targets

## Lab 10.3: Edge Deployment

### Objective
Deploy complete pipeline to NVIDIA Jetson.

### Requirements
1. Optimize models with TensorRT
2. Deploy to Jetson AGX Orin
3. Achieve >10 FPS perception
4. Integrate with robot hardware
5. Demonstrate pick-and-place task

## Summary

This chapter covered:
- **Behavior Trees**: Structured decision-making
- **Visual Servoing**: Vision-guided control
- **Complete Pipelines**: Integration of perception and action
- **Optimization**: TensorRT for real-time inference
- **Edge Deployment**: Running on NVIDIA Jetson

You can now build complete AI-powered robotic systems!

---

**Next**: [Module 4: Vision-Language-Action & LLMs →](../module-4/index)
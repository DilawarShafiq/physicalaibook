# Sim-to-Real Transfer & Best Practices

**Week 5: Bridging the Reality Gap**

## Overview

The "reality gap" is the difference between simulated and real-world robot behavior. Algorithms that work perfectly in simulation often fail on physical hardware due to unmodeled dynamics, sensor noise, and environmental variations. This chapter covers techniques to minimize this gap and successfully transfer policies from simulation to reality.

## Learning Objectives

By the end of this chapter, you will:
- Understand the sources of the sim-to-real reality gap
- Implement domain randomization techniques
- Model realistic sensor noise and imperfections
- Calibrate simulation parameters to match real hardware
- Validate sim-to-real transfer systematically
- Apply best practices for robust simulation development

## The Reality Gap

### What Causes the Reality Gap?

**Physical Modeling**:
- Simplified friction models
- Unmodeled compliance and flexibility
- Approximated contact dynamics
- Idealized actuator responses

**Sensor Discrepancies**:
- Perfect vs. noisy measurements
- Different resolution and field of view
- Lighting conditions in simulation vs. reality
- Timing and synchronization issues

**Environmental Factors**:
- Uniform vs. variable surfaces
- Predictable vs. unpredictable objects
- Static vs. dynamic environments
- Calibration drift over time

### Measuring the Reality Gap

```python
# sim_real_comparison.py
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class PerformanceMetrics:
    """Metrics to compare sim and real performance"""
    success_rate: float
    completion_time: float
    path_deviation: float
    collision_count: int
    energy_consumption: float

class RealityGapAnalyzer:
    def __init__(self):
        self.sim_metrics = []
        self.real_metrics = []

    def compute_gap(self) -> dict:
        """Compute reality gap across metrics"""
        return {
            'success_rate_gap': np.mean([
                abs(s.success_rate - r.success_rate)
                for s, r in zip(self.sim_metrics, self.real_metrics)
            ]),
            'time_gap': np.mean([
                abs(s.completion_time - r.completion_time)
                for s, r in zip(self.sim_metrics, self.real_metrics)
            ]),
            'path_deviation_gap': np.mean([
                abs(s.path_deviation - r.path_deviation)
                for s, r in zip(self.sim_metrics, self.real_metrics)
            ])
        }

    def report(self):
        """Generate reality gap report"""
        gaps = self.compute_gap()

        print("=== Reality Gap Analysis ===")
        print(f"Success Rate Gap: {gaps['success_rate_gap']:.2%}")
        print(f"Completion Time Gap: {gaps['time_gap']:.2f}s")
        print(f"Path Deviation Gap: {gaps['path_deviation_gap']:.2f}m")

        # Recommend actions based on gaps
        if gaps['success_rate_gap'] > 0.2:
            print("\n⚠️  Large success rate gap detected!")
            print("→ Check collision geometry and sensor accuracy")

        if gaps['time_gap'] > 5.0:
            print("\n⚠️  Large timing gap detected!")
            print("→ Verify actuator dynamics and control frequencies")
```

## Domain Randomization

### What is Domain Randomization?

Domain randomization trains models on a wide variety of simulated conditions, making them robust to real-world variations.

### Parameters to Randomize

**Physics**:
- Friction coefficients (0.3 - 1.2)
- Object masses (±20%)
- Contact damping and stiffness
- Actuator gains and delays

**Visual**:
- Lighting conditions (intensity, angle, color temperature)
- Texture appearance
- Camera intrinsics (focal length, distortion)
- Object colors and materials

**Sensor Noise**:
- Gaussian noise on measurements
- Dropout/occlusion events
- Timing jitter
- Calibration errors

### Implementing Domain Randomization in Gazebo

```xml
<!-- randomized_world.sdf -->
<sdf version="1.7">
  <world name="randomized_world">

    <!-- Randomized Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <!-- Randomize intensity: 0.5 - 1.5 -->
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>

      <!-- Randomize direction -->
      <direction>-0.5 0.5 -0.9</direction>
    </light>

    <!-- Randomized Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <!-- Randomize friction: 0.5 - 1.2 -->
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```

### Python Domain Randomization Plugin

```python
# domain_randomization_plugin.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, SetModelState
from geometry_msgs.msg import Pose
import random
import numpy as np

class DomainRandomizer(Node):
    def __init__(self):
        super().__init__('domain_randomizer')

        # Service clients for Gazebo
        self.set_state_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        # Randomization parameters
        self.friction_range = (0.3, 1.2)
        self.mass_variation = 0.2  # ±20%
        self.light_intensity_range = (0.5, 1.5)

        self.get_logger().info('Domain Randomizer initialized')

    def randomize_friction(self, surface_name: str):
        """Randomize surface friction"""
        friction = random.uniform(*self.friction_range)

        # Apply friction (requires Gazebo plugin)
        self.get_logger().info(f'Randomizing friction to {friction:.2f}')
        # Implementation depends on Gazebo version

    def randomize_object_properties(self, object_name: str, base_mass: float):
        """Randomize object mass and inertia"""
        mass_multiplier = 1.0 + random.uniform(-self.mass_variation, self.mass_variation)
        new_mass = base_mass * mass_multiplier

        self.get_logger().info(f'Object {object_name} mass: {new_mass:.2f}kg')

    def randomize_lighting(self):
        """Randomize lighting conditions"""
        intensity = random.uniform(*self.light_intensity_range)

        # Random light direction
        azimuth = random.uniform(0, 2 * np.pi)
        elevation = random.uniform(np.pi/6, np.pi/3)

        self.get_logger().info(
            f'Light intensity: {intensity:.2f}, '
            f'direction: ({azimuth:.2f}, {elevation:.2f})'
        )

    def randomize_camera_parameters(self):
        """Randomize camera intrinsics"""
        # Focal length variation ±5%
        focal_length_mult = random.uniform(0.95, 1.05)

        # Add radial distortion
        k1 = random.uniform(-0.1, 0.1)
        k2 = random.uniform(-0.05, 0.05)

        self.get_logger().info(
            f'Camera randomization: focal_mult={focal_length_mult:.3f}, '
            f'k1={k1:.3f}, k2={k2:.3f}'
        )

    def randomize_episode(self):
        """Randomize all parameters for new episode"""
        self.randomize_friction('ground_plane')
        self.randomize_lighting()
        self.randomize_camera_parameters()

        # Randomize object placements
        for obj in ['cube', 'cylinder', 'sphere']:
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(-1.0, 1.0)
            yaw = random.uniform(0, 2 * np.pi)
            self.randomize_object_pose(obj, x, y, yaw)

    def randomize_object_pose(self, name: str, x: float, y: float, yaw: float):
        """Randomize object pose in world"""
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = 0.5
        request.state.pose.orientation.z = np.sin(yaw / 2)
        request.state.pose.orientation.w = np.cos(yaw / 2)

        future = self.set_state_client.call_async(request)

def main():
    rclpy.init()
    randomizer = DomainRandomizer()

    # Randomize environment every 30 seconds
    timer = randomizer.create_timer(30.0, randomizer.randomize_episode)

    rclpy.spin(randomizer)
    randomizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Modeling

### Realistic Camera Simulation

```xml
<!-- realistic_camera.urdf -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>

      <!-- Realistic noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- Typical for RGB cameras -->
      </noise>

      <!-- Lens distortion -->
      <distortion>
        <k1>-0.02</k1>
        <k2>0.01</k2>
        <k3>0.0</k3>
        <p1>0.001</p1>
        <p2>-0.001</p2>
        <center>0.5 0.5</center>
      </distortion>
    </camera>

    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=rgb/image_raw</remapping>
        <remapping>camera_info:=rgb/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Realistic LIDAR Simulation

```xml
<!-- realistic_lidar.urdf -->
<gazebo reference="laser_link">
  <sensor name="laser" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.015</resolution>
      </range>

      <!-- Realistic noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm stddev -->
      </noise>
    </ray>

    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/scan</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Calibration for Sim-to-Real

### System Identification

```python
# system_identification.py
import numpy as np
from scipy.optimize import minimize
from typing import Tuple

class RobotSystemIdentifier:
    """Identify robot dynamic parameters from real data"""

    def __init__(self):
        self.friction_coeff = 0.5  # Initial guess
        self.motor_constant = 1.0
        self.wheel_radius = 0.05

    def collect_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """Collect velocity commands and actual velocities"""
        # cmd_vel: commanded velocities
        # actual_vel: measured velocities from encoders

        # Simulated data (replace with real robot data)
        cmd_vel = np.linspace(0, 1.0, 100)
        actual_vel = cmd_vel * 0.9 + np.random.normal(0, 0.02, 100)

        return cmd_vel, actual_vel

    def cost_function(self, params, cmd_vel, actual_vel):
        """Cost function for parameter optimization"""
        friction, motor_const = params

        # Predicted velocity model
        predicted_vel = cmd_vel * motor_const * (1 - friction * np.sign(cmd_vel))

        # Mean squared error
        error = np.mean((predicted_vel - actual_vel) ** 2)
        return error

    def identify_parameters(self):
        """Run system identification"""
        cmd_vel, actual_vel = self.collect_data()

        # Optimize parameters
        initial_params = [self.friction_coeff, self.motor_constant]
        result = minimize(
            self.cost_function,
            initial_params,
            args=(cmd_vel, actual_vel),
            method='Nelder-Mead'
        )

        self.friction_coeff, self.motor_constant = result.x

        print(f"Identified Parameters:")
        print(f"  Friction coefficient: {self.friction_coeff:.4f}")
        print(f"  Motor constant: {self.motor_constant:.4f}")
        print(f"  Final error: {result.fun:.6f}")

        return result.x

    def update_simulation_params(self, sim_config_file: str):
        """Update simulation with identified parameters"""
        # Update Gazebo SDF with identified parameters
        print(f"\nUpdating {sim_config_file}...")
        print(f"  Set friction to: {self.friction_coeff:.4f}")
        print(f"  Set motor gain to: {self.motor_constant:.4f}")
```

### Camera Calibration

```python
# camera_calibration.py
import cv2
import numpy as np
from pathlib import Path

class CameraCalibrator:
    """Calibrate camera and transfer parameters to simulation"""

    def __init__(self, checkerboard_size=(9, 6), square_size=0.025):
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size  # meters

        self.objpoints = []  # 3D points in real world
        self.imgpoints = []  # 2D points in image plane

    def calibrate_from_images(self, image_folder: str):
        """Calibrate camera from checkerboard images"""
        # Prepare object points
        objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3),
                       np.float32)
        objp[:, :2] = np.mgr[grid[0:self.checkerboard_size[0],
                                  0:self.checkerboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        # Find checkerboard corners in images
        images = Path(image_folder).glob('*.jpg')

        for img_path in images:
            img = cv2.imread(str(img_path))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find checkerboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, self.checkerboard_size, None
            )

            if ret:
                self.objpoints.append(objp)
                self.imgpoints.append(corners)

        # Calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, gray.shape[::-1], None, None
        )

        print("=== Camera Calibration Results ===")
        print(f"Camera Matrix:\n{mtx}")
        print(f"\nDistortion Coefficients:\n{dist}")

        return mtx, dist

    def generate_gazebo_config(self, mtx, dist, width=640, height=480):
        """Generate Gazebo camera configuration"""
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]

        # Convert to Gazebo format
        horizontal_fov = 2 * np.arctan(width / (2 * fx))

        config = f"""
<camera name="calibrated_camera">
  <horizontal_fov>{horizontal_fov:.6f}</horizontal_fov>
  <image>
    <width>{width}</width>
    <height>{height}</height>
  </image>
  <clip>
    <near>0.02</near>
    <far>100</far>
  </clip>
  <distortion>
    <k1>{dist[0][0]:.6f}</k1>
    <k2>{dist[0][1]:.6f}</k2>
    <k3>{dist[0][4]:.6f}</k3>
    <p1>{dist[0][2]:.6f}</p1>
    <p2>{dist[0][3]:.6f}</p2>
    <center>{cx/width:.4f} {cy/height:.4f}</center>
  </distortion>
</camera>
"""
        print("\n=== Gazebo Camera Config ===")
        print(config)
        return config
```

## Validation Strategies

### Performance Parity Testing

```python
# validation_suite.py
from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class ValidationTest:
    name: str
    sim_result: float
    real_result: float
    tolerance: float

    def passed(self) -> bool:
        return abs(self.sim_result - self.real_result) <= self.tolerance

class SimToRealValidator:
    def __init__(self):
        self.tests: List[ValidationTest] = []

    def add_test(self, name: str, sim_result: float,
                 real_result: float, tolerance: float):
        """Add validation test"""
        test = ValidationTest(name, sim_result, real_result, tolerance)
        self.tests.append(test)

        status = "✓ PASS" if test.passed() else "✗ FAIL"
        print(f"{status} | {name}: sim={sim_result:.3f}, real={real_result:.3f}")

    def run_standard_suite(self):
        """Run standard sim-to-real validation tests"""

        # Test 1: Straight-line motion
        self.add_test(
            "1m straight motion time",
            sim_result=10.5,  # seconds
            real_result=11.2,
            tolerance=1.0
        )

        # Test 2: 90-degree turn
        self.add_test(
            "90-deg turn accuracy",
            sim_result=89.5,  # degrees
            real_result=88.3,
            tolerance=3.0
        )

        # Test 3: Obstacle detection distance
        self.add_test(
            "Obstacle detection range",
            sim_result=5.2,  # meters
            real_result=4.9,
            tolerance=0.5
        )

        # Test 4: Path following error
        self.add_test(
            "Path following RMS error",
            sim_result=0.05,  # meters
            real_result=0.08,
            tolerance=0.05
        )

    def summary(self):
        """Print validation summary"""
        passed = sum(1 for t in self.tests if t.passed())
        total = len(self.tests)

        print(f"\n=== Validation Summary ===")
        print(f"Passed: {passed}/{total} ({100*passed/total:.1f}%)")

        if passed == total:
            print("✓ All tests passed! Sim-to-real gap is acceptable.")
        else:
            print("✗ Some tests failed. Review parameters and models.")

            failed_tests = [t for t in self.tests if not t.passed()]
            print("\nFailed Tests:")
            for test in failed_tests:
                gap = abs(test.sim_result - test.real_result)
                print(f"  - {test.name}: gap={gap:.3f} (tolerance={test.tolerance})")
```

## Best Practices

### 1. Start Simple, Add Complexity
- Begin with basic models and gradually add details
- Validate each addition against real data
- Don't over-complicate without data to support it

### 2. Measure Everything
- Instrument both simulation and real robot
- Log all relevant metrics
- Compare distributions, not just means

### 3. Iterate Based on Data
- Identify largest discrepancies first
- Prioritize fixing high-impact gaps
- Re-validate after each change

### 4. Use Progressive Sim-to-Real
- Train in simulation with increasing realism
- Fine-tune on small amounts of real data
- Leverage domain randomization throughout

## Lab 5.1: Domain Randomization

### Objective
Implement domain randomization for a navigation task.

### Requirements
1. Randomize lighting, friction, and object placement
2. Train navigation policy with randomization
3. Test on real robot (or high-fidelity sim)
4. Compare performance with/without randomization

## Lab 5.2: Camera Calibration

### Objective
Calibrate real camera and update simulation parameters.

### Requirements
1. Collect checkerboard calibration images
2. Extract camera intrinsics and distortion
3. Update Gazebo camera configuration
4. Validate visual servo performance

## Lab 5.3: Validation Report

### Objective
Create comprehensive sim-to-real validation report.

### Requirements
1. Define at least 5 performance metrics
2. Measure metrics in both sim and real
3. Compute reality gaps
4. Document tuning process and results

## Summary

This chapter covered:
- **Reality Gap**: Understanding and measuring sim-to-real discrepancies
- **Domain Randomization**: Training robust policies through variation
- **Sensor Modeling**: Adding realistic noise and imperfections
- **Calibration**: Matching simulation to real hardware
- **Validation**: Systematic testing of sim-to-real transfer

With these techniques, you can develop algorithms in simulation that transfer successfully to physical robots!

---

**Next**: [Multi-Robot Systems & Advanced Simulation →](./multi-robot)
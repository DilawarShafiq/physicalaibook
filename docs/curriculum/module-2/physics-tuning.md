---
title: Physics Tuning & Sensor Integration
sidebar_label: Physics & Sensor Integration
description: Learn to tune physics parameters and integrate sensors for realistic simulation
keywords: [physics tuning, sensor integration, gazebo, simulation, robotics, urdf]
---

# Physics Tuning & Sensor Integration

Accurate physics simulation and realistic sensor modeling are crucial for effective sim-to-real transfer in Physical AI systems.

## Physics Tuning Fundamentals

The accuracy of your simulation depends heavily on how well you tune physics parameters to match reality.

### Key Physics Parameters

**Max Step Size**
```xml
<max_step_size>0.001</max_step_size>
```
- Smaller values = higher accuracy but slower simulation
- Typical range: 0.001s to 0.01s
- Start with 0.001s for precise control tasks, increase for performance

**Real-time Factor**
```xml
<real_time_factor>1.0</real_time_factor>
```
- 1.0 = real-time simulation
- > 1.0 = faster than real-time (good for training)
- < 1.0 = slower than real-time (more stability)

**Solver Iterations**
```xml
<solver>
  <type>quick</type>
  <iters>10</iters>  <!-- Increase for accuracy -->
  <sor>1.3</sor>     <!-- Relaxation parameter -->
</solver>
```

### Mass Properties

Accurate mass properties are essential for realistic robot dynamics:

```xml
<link name="wheel_link">
  <inertial>
    <mass value="0.1"/>  <!-- Measured mass -->
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0"
             izz="0.002"/> <!-- Higher moment of inertia around rotation axis -->
  </inertial>
</link>
```

:::tip
Always validate mass properties experimentally. Use CAD software to calculate moments of inertia, or measure physical robot properties directly.
:::

## Sensor Integration

### Camera Simulation

Realistic camera simulation with depth and RGB-D capabilities:

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    
    <!-- Add noise model -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
    
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/tb3_0</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Simulation

Modeling realistic LiDAR with appropriate noise characteristics:

```xml
<sensor name="lidar" type="ray">
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
      <min>0.12</min>
      <max>3.5</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  
  <!-- Add realistic noise -->
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev>
  </noise>
</sensor>
```

### IMU Simulation

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <!-- Noise models for realistic sensor behavior -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Sim-to-Real Transfer Techniques

### 1. Domain Randomization

Randomize simulation parameters to improve robustness:

```python
class DomainRandomization:
    def __init__(self):
        self.param_ranges = {
            'friction': (0.5, 1.5),  # Friction coefficient range
            'mass_variance': (0.95, 1.05),  # Mass variation ±5%
            'lighting': (0.7, 1.3),  # Brightness range
        }

    def randomize_environment(self, world_model):
        # Randomize physics parameters
        friction_coeff = random.uniform(*self.param_ranges['friction'])
        mass_variance = random.uniform(*self.param_ranges['mass_variance'])
        
        # Apply randomization to robot model
        for link in world_model.links:
            # Apply mass variance
            link.mass *= mass_variance
            
            # Apply friction variations
            for collision in link.collisions:
                collision.friction.mu *= friction_coeff

    def randomize_sensors(self, robot_model):
        # Randomize sensor noise characteristics
        for sensor in robot_model.sensors:
            if sensor.type == 'camera':
                sensor.noise.stddev *= random.uniform(0.5, 2.0)
```

### 2. System Identification

Match simulation behavior to real robot:

```python
def identify_robot_parameters(real_robot, sim_robot):
    """
    Parameter identification using system identification techniques
    """
    # Excite robot with known inputs
    input_signals = generate_excitation_signals()
    
    # Collect real and sim responses
    real_responses = collect_real_data(real_robot, input_signals)
    sim_responses = collect_sim_data(sim_robot, input_signals)
    
    # Optimize parameters to minimize sim-to-real error
    optimized_params = optimize_parameters(
        real_responses, 
        sim_responses, 
        initial_guess=get_physics_params()
    )
    
    # Apply optimized parameters to simulation
    update_physics_params(sim_robot, optimized_params)
```

## Hands-On Lab: Physics Validation and Sensor Calibration

:::lab-container
<div class="lab-title">
    <span class="lab-title-icon">⚙️</span>
    <strong>Lab: Sim-to-Real Physics Validation</strong>
</div>

**Objective**: Tune simulation parameters to match real robot behavior

**Setup**:
- Use a differential-drive robot with encoders, IMU, and LiDAR
- Create identical simulation and physical robot setups

**Tasks**:
1. **Step Response Validation**
   - Command robot to move 1m forward in simulation
   - Measure time to reach target vs. real robot
   - Adjust motor parameters until match within 10%

2. **Turning Radius Validation**
   - Command robot to rotate 360° in place
   - Compare encoder counts between sim and real
   - Tune wheel radius and track parameters

3. **Sensor Noise Characterization**
   - Record 30 seconds of stationary sensor data
   - Compare noise statistics between sim and real sensors
   - Calibrate noise models

**Success Criteria**:
- Position accuracy: ≤ 5cm (both x, y coordinates)
- Orientation accuracy: ≤ 5°
- Sensor noise: matching distributions
</details>

<details>
<summary>Implementation Hints</summary>

- Start with kinematic parameters (wheel radius, track width)
- Use system identification tools like MATLAB or SciPy
- Focus on one sensor type at a time
- Use visual tracking for ground truth (if available)
</details>
:::

## Advanced Physics Topics

### Contact Materials

Define realistic contact properties between surfaces:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
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

<!-- Materials definition -->
<world name="default">
  <material name="tarmac">
    <odometry>0.95 0.05 0.0</odometry>  <!-- Rolling friction -->
  </material>
  
  <material name="grass">
    <odometry>0.15 0.85 0.0</odometry>  <!-- Sliding friction -->
  </material>
</world>
```

### Joint Dynamics

Accurate joint modeling for manipulator simulation:

```xml
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="arm_base_link"/>
  <child link="shoulder_link"/>
  <axis>
    <xyz>0 0 1</xyz>
    <limit effort="30" velocity="1.0" lower="-2.6" upper="2.6"/>
    <dynamics damping="1.0" friction="0.1"/>
  </axis>
</joint>

<gazebo reference="shoulder_pan_joint">
  <provide_feedback>true</provide_feedback>
  <implicit_spring_damper>true</implicit_spring_damper>
</gazebo>
```

## Troubleshooting Physics Issues

### Common Problems:

**1. Robot Drifts/Wobbles**
- Solution: Increase solver iterations or reduce step size
- Check mass/inertia properties are physically realistic

**2. Robot Falls Through Floor**
- Solution: Verify collision geometry and physics parameters
- Increase ERP/CFM values slightly

**3. Unstable Contacts**
- Solution: Reduce contact surface layer values
- Check for intersecting geometries

**4. Slow Simulation**
- Solution: Increase step size, reduce solver iterations
- Use simplified collision geometries

```bash
# Debug physics performance
gz stats  # Check real-time factor (should be close to 1.0)
gz topic -e -t /gazebo/default/stats  # Monitor simulation stats

# Check for physics errors
ro2 topic echo /gazebo/default/logical_camera/contacts
```

## Summary & Next Steps

Physics tuning and sensor integration are critical for effective sim-to-real transfer. Properly calibrated simulators enable:
- Safe algorithm validation
- Rapid iteration without hardware wear
- Photorealistic training data generation

In the next section, we'll explore advanced techniques for domain randomization and photorealistic simulation with NVIDIA Isaac Sim.

---

**Next**: [Module 2: Sim-to-Real Transfer Techniques →](./sim-to-real-transfer)
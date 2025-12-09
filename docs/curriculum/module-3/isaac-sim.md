# NVIDIA Isaac Sim Fundamentals

**Week 8: Photorealistic Simulation and Synthetic Data**

## Overview

NVIDIA Isaac Sim leverages Omniverse for photorealistic robot simulation, enabling synthetic data generation, parallel training environments, and accurate physics simulation with RTX ray tracing.

## Learning Objectives

- Install and configure NVIDIA Isaac Sim
- Create photorealistic simulation environments
- Generate synthetic datasets with automatic labeling
- Use Isaac Gym for parallel reinforcement learning
- Import custom robots into Isaac Sim
- Deploy trained policies to physical hardware

## Getting Started with Isaac Sim

### Installation

```bash
# Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Launch and install Isaac Sim from Omniverse
./omniverse-launcher-linux.AppImage

# Install Isaac Sim Python API
pip install isaacsim
```

### First Isaac Sim Scene

```python
# hello_isaac.py
from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
import numpy as np

# Create world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0, 0, 1.0]),
        size=np.array([0.2, 0.2, 0.2]),
        color=np.array([0.5, 0.0, 0.0])
    )
)

# Reset world
world.reset()

# Run simulation
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

## Importing Custom Robots

### Convert URDF to USD

```python
# urdf_to_usd.py
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.importer.urdf")

from omni.isaac.urdf import _urdf
import omni

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()

# Convert URDF to USD
urdf_path = "/path/to/robot.urdf"
usd_path = "/path/to/output/robot.usd"

import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.create_physics_scene = True

success = urdf_interface.parse_urdf(urdf_path, usd_path, import_config)

if success:
    print(f"Successfully converted URDF to {usd_path}")
else:
    print("URDF conversion failed")

simulation_app.close()
```

## Synthetic Data Generation

### Automatic Annotation Pipeline

```python
# synthetic_data_gen.py
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.viewports import set_camera_view
from omni.replicator.core import Writer, AnnotatorRegistry, BackendDispatch
import omni.replicator.core as rep
import numpy as np

# Create world with camera
world = World()
world.scene.add_default_ground_plane()

# Setup camera
camera = rep.create.camera(position=(3, 3, 3), look_at=(0, 0, 0))

# Domain randomization
def randomize_scene():
    """Randomize lighting, materials, and object poses"""
    with rep.trigger.on_frame():
        # Randomize lighting
        with rep.create.light(
            light_type="Sphere",
            temperature=rep.distribution.uniform(3000, 6500),
            intensity=rep.distribution.uniform(20000, 40000)
        ):
            rep.modify.pose(
                position=rep.distribution.uniform((-5, -5, 5), (5, 5, 10)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

        # Randomize object positions
        with rep.get.prims(semantics=[("class", "object")]):
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 2)),
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
            )

# Setup writers for annotations
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/path/to/output",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
    distance_to_camera=True,
    normals=True
)

# Generate dataset
rep.orchestrator.run()

for i in range(1000):
    world.step(render=True)
    if i % 10 == 0:
        print(f"Generated {i} frames")

simulation_app.close()
```

## Isaac Gym for Parallel RL

### Parallel Environment Setup

```python
# isaac_gym_env.py
from isaacgym import gymapi
from isaacgym import gymutil
import numpy as np
import torch

class ParallelReachEnv:
    """Parallel reaching task with Isaac Gym"""

    def __init__(self, num_envs=1024):
        self.num_envs = num_envs

        # Initialize gym
        self.gym = gymapi.acquire_gym()

        # Create sim
        sim_params = gymapi.SimParams()
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
        sim_params.dt = 1.0 / 60.0
        sim_params.substeps = 2

        # GPU pipeline
        sim_params.use_gpu_pipeline = True
        sim_params.physx.use_gpu = True

        self.sim = self.gym.create_sim(
            0, 0, gymapi.SIM_PHYSX, sim_params
        )

        # Create environments
        self.envs = []
        self.actors = []

        env_spacing = 2.0
        envs_per_row = int(np.sqrt(num_envs))

        for i in range(num_envs):
            # Create env
            env = self.gym.create_env(
                self.sim,
                gymapi.Vec3(-env_spacing, -env_spacing, 0),
                gymapi.Vec3(env_spacing, env_spacing, env_spacing),
                envs_per_row
            )

            # Load robot asset
            asset_root = "/path/to/assets"
            asset_file = "robot.urdf"
            asset = self.gym.load_asset(self.sim, asset_root, asset_file)

            # Create actor
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0)
            actor = self.gym.create_actor(env, asset, pose, f"robot_{i}", i, 1)

            self.envs.append(env)
            self.actors.append(actor)

        # Create viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())

    def step(self, actions):
        """Step all environments in parallel"""
        # Apply actions to all envs
        for i, env in enumerate(self.envs):
            # Set joint targets based on actions
            pass  # Implementation specific to robot

        # Step simulation
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Render
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, True)

        # Get observations and rewards
        obs = self.get_observations()
        rewards = self.compute_rewards()

        return obs, rewards

    def get_observations(self):
        """Get observations from all environments"""
        # Gather joint states, end-effector poses, etc.
        return torch.zeros((self.num_envs, 10))  # Placeholder

    def compute_rewards(self):
        """Compute rewards for all environments"""
        # Distance to goal, velocity penalties, etc.
        return torch.zeros(self.num_envs)  # Placeholder

    def reset(self, env_ids=None):
        """Reset specified environments"""
        if env_ids is None:
            env_ids = range(self.num_envs)

        for env_id in env_ids:
            # Reset robot state
            pass  # Implementation specific

# Usage
env = ParallelReachEnv(num_envs=1024)

for epoch in range(100):
    obs = env.reset()
    for step in range(1000):
        actions = torch.randn(1024, 6)  # Random actions
        obs, rewards = env.step(actions)
```

## ROS 2 Integration

### Isaac Sim with ROS 2

```python
# isaac_ros2_bridge.py
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS2 bridge
enable_extension("omni.isaac.ros2_bridge")

from omni.isaac.ros2_bridge import ROS2Bridge
import rclpy

# Initialize ROS 2
rclpy.init()

# Create world
world = World()
world.scene.add_default_ground_plane()

# Create ROS 2 publishers/subscribers
ros_bridge = ROS2Bridge()

# Publish camera images to ROS 2
camera_pub = ros_bridge.create_publisher(
    "/camera/image_raw",
    "sensor_msgs/Image",
    10
)

# Subscribe to cmd_vel from ROS 2
def cmd_vel_callback(msg):
    print(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
    # Apply to robot in simulation

cmd_vel_sub = ros_bridge.create_subscription(
    "/cmd_vel",
    "geometry_msgs/Twist",
    cmd_vel_callback,
    10
)

# Simulation loop
world.reset()
for i in range(10000):
    world.step(render=True)

simulation_app.close()
```

## Lab 8.1: Isaac Sim Setup

### Objective
Install Isaac Sim and create first simulation.

### Requirements
1. Install NVIDIA Omniverse and Isaac Sim
2. Create scene with robot and objects
3. Import custom URDF robot
4. Verify physics simulation accuracy

## Lab 8.2: Synthetic Dataset Generation

### Objective
Generate annotated dataset for perception.

### Requirements
1. Setup domain randomization
2. Generate 10,000 images with annotations
3. Include bounding boxes and segmentation masks
4. Export in COCO format
5. Train detection model on synthetic data

## Lab 8.3: Parallel RL Training

### Objective
Use Isaac Gym for parallel robot training.

### Requirements
1. Create parallel reaching environment
2. Implement PPO algorithm
3. Train with 1024 parallel environments
4. Achieve target reaching success rate >90%
5. Deploy policy to Isaac Sim robot

## Summary

This chapter covered:
- **Isaac Sim**: Photorealistic robot simulation
- **Synthetic Data**: Automated annotation pipeline
- **Isaac Gym**: Massively parallel RL training
- **USD Assets**: Robot import and conversion
- **ROS 2 Integration**: Bridging Isaac Sim with ROS 2

NVIDIA Isaac provides powerful tools for AI-powered robotics development!

---

**Next**: [Reinforcement Learning for Manipulation →](./rl-manipulation)
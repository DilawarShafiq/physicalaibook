# 13-Week Course Schedule

This schedule provides a week-by-week breakdown of topics, labs, and deliverables for the Physical AI & Humanoid Robotics course.

## Module 1: ROS 2 Fundamentals (Weeks 1-3)

### Week 1: Introduction to ROS 2 & Environment Setup

**Topics**:
- ROS 2 architecture and design philosophy
- Nodes, topics, and the publish/subscribe pattern
- Workspace setup with colcon
- Python and C++ client libraries

**Labs**:
- Lab 1.1: Install ROS 2 Humble on Ubuntu 22.04
- Lab 1.2: Create your first publisher/subscriber nodes
- Lab 1.3: Visualize data with RViz2

**Deliverable**: Functional ROS 2 workspace with custom talker/listener nodes

---

### Week 2: Advanced ROS 2 Communication

**Topics**:
- Services and clients (synchronous communication)
- Actions (long-running tasks with feedback)
- Parameters and launch files
- ROS 2 bag for data recording/playback

**Labs**:
- Lab 2.1: Implement a service-based calculator node
- Lab 2.2: Create an action server for robot navigation
- Lab 2.3: Write launch files with parameter configuration

**Deliverable**: Multi-node system with services and actions

---

### Week 3: Navigation & Coordinate Frames

**Topics**:
- tf2 transforms and coordinate frames
- URDF robot descriptions
- Navigation2 stack overview
- Basic path planning with Nav2

**Labs**:
- Lab 3.1: Build a robot URDF with sensors
- Lab 3.2: Broadcast and listen to tf transforms
- Lab 3.3: Configure Nav2 for autonomous navigation

**Deliverable**: Simulated robot navigating to waypoints

---

## Module 2: Digital Twin & Gazebo Simulation (Weeks 4-6)

### Week 4: Introduction to Gazebo Classic & Gazebo Sim

**Topics**:
- Simulation fundamentals and physics engines
- Gazebo world files and model SDFs
- Sensor plugins (cameras, lidars, IMUs)
- ROS 2-Gazebo integration with ros_gz

**Labs**:
- Lab 4.1: Create a custom Gazebo world
- Lab 4.2: Add sensors to your robot model
- Lab 4.3: Spawn robots programmatically

**Deliverable**: Custom Gazebo environment with sensor-equipped robot

---

### Week 5: Sim-to-Real Transfer & Best Practices

**Topics**:
- Domain randomization techniques
- Reality gap and how to minimize it
- Camera calibration and sensor noise models
- Benchmarking simulation performance

**Labs**:
- Lab 5.1: Add domain randomization to lighting/textures
- Lab 5.2: Calibrate virtual camera to match real RealSense
- Lab 5.3: Profile simulation speed vs. accuracy trade-offs

**Deliverable**: Sim-to-real transfer validation report

---

### Week 6: Multi-Robot Systems & Advanced Simulation

**Topics**:
- Multi-robot coordination
- ROS 2 namespacing and remapping
- Large-scale simulation optimization
- Headless simulation for CI/CD

**Labs**:
- Lab 6.1: Deploy 5 robots in coordinated formation
- Lab 6.2: Set up headless Gazebo for automated testing
- Lab 6.3: Integrate simulation into GitHub Actions

**Deliverable**: Multi-robot warehouse simulation

---

## Module 3: AI-Robot Brain & NVIDIA Isaac (Weeks 7-10)

### Week 7: Computer Vision for Robotics

**Topics**:
- Object detection (YOLO, Faster R-CNN)
- Semantic segmentation
- Depth estimation
- ROS 2 vision pipelines

**Labs**:
- Lab 7.1: Deploy YOLOv8 on ROS 2 image topics
- Lab 7.2: Perform semantic segmentation with DeepLabV3
- Lab 7.3: Fuse RGB-D data for 3D perception

**Deliverable**: Real-time object detection on robot camera feed

---

### Week 8: NVIDIA Isaac Sim Fundamentals

**Topics**:
- Isaac Sim architecture and Omniverse
- Photorealistic rendering for synthetic data
- RTX ray tracing and physics simulation
- Isaac Gym for parallel training

**Labs**:
- Lab 8.1: Install and configure Isaac Sim
- Lab 8.2: Import custom URDF into Isaac
- Lab 8.3: Generate labeled synthetic datasets

**Deliverable**: Synthetic dataset with 10,000 annotated images

---

### Week 9: Reinforcement Learning for Manipulation

**Topics**:
- RL basics (Q-learning, policy gradients)
- Isaac Gym for massively parallel training
- Reward shaping for manipulation tasks
- Sim-to-real RL deployment

**Labs**:
- Lab 9.1: Train pick-and-place with PPO in Isaac Gym
- Lab 9.2: Tune reward functions for stable grasping
- Lab 9.3: Deploy trained policy to Jetson

**Deliverable**: RL-trained robot performing pick-and-place

---

### Week 10: End-to-End Perception-Action Pipelines

**Topics**:
- Behavior trees for decision-making
- Integrating vision models with motion planning
- Failure recovery and robustness
- Real-time performance optimization

**Labs**:
- Lab 10.1: Build behavior tree for kitchen assistant robot
- Lab 10.2: Combine object detection + grasping pipeline
- Lab 10.3: Optimize inference latency on Jetson

**Deliverable**: Autonomous manipulation pipeline

---

## Module 4: Vision-Language-Action (VLA) & LLMs (Weeks 11-13)

### Week 11: Large Language Models for Robotics

**Topics**:
- LLM fundamentals (GPT-4, Claude, LLaMA)
- Prompt engineering for robot commands
- Function calling and tool use
- Safety constraints for LLM control

**Labs**:
- Lab 11.1: Integrate GPT-4 with ROS 2 command interface
- Lab 11.2: Parse natural language to robot actions
- Lab 11.3: Implement safety checks for LLM outputs

**Deliverable**: Voice-controlled robot with LLM backend

---

### Week 12: Vision-Language-Action Models

**Topics**:
- VLA architectures (RT-2, PaLM-E, OpenVLA)
- End-to-end vision-to-action learning
- Generalization to new tasks
- Deployment on edge devices

**Labs**:
- Lab 12.1: Fine-tune OpenVLA on custom tasks
- Lab 12.2: Deploy VLA model to Jetson Orin
- Lab 12.3: Benchmark VLA vs. traditional pipelines

**Deliverable**: VLA-powered manipulation demo

---

### Week 13: Capstone Project & Final Presentations

**Topics**:
- Project planning and system integration
- Debugging complex robotics systems
- Effective technical presentations
- Career paths in Physical AI

**Capstone Requirements**:
- Integrate ROS 2, simulation, AI models, and hardware
- Solve a real-world problem (e.g., assistive robotics, warehouse automation)
- Present system architecture, results, and lessons learned

**Deliverable**: Final capstone project + 15-minute presentation

---

## Important Dates

| Week | Deliverable Due | Weight |
|------|----------------|--------|
| 3 | Module 1 Project | 15% |
| 6 | Module 2 Project | 15% |
| 10 | Module 3 Project | 20% |
| 13 | Final Capstone | 30% |
| Ongoing | Lab Assignments | 20% |

---

**Next**: [Learning Outcomes →](./learning-outcomes)

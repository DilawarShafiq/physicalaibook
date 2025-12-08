# Module 1: ROS 2 Fundamentals

**Duration**: Weeks 1-3
**Prerequisite**: Intermediate Python, Linux command line basics

## Module Overview

Master the Robot Operating System 2 (ROS 2), the industry-standard framework for building modular, distributed robotics systems. This module covers core communication patterns, navigation frameworks, and best practices for professional robot software development.

## Learning Objectives

By the end of this module, you will be able to:

- Create and manage ROS 2 nodes in Python and C++
- Implement publishers, subscribers, services, and actions
- Use tf2 for coordinate frame transformations
- Build robot models with URDF
- Configure the Navigation2 stack for autonomous navigation
- Write launch files for complex multi-node systems

## Topics Covered

### Week 1: Introduction to ROS 2 & Environment Setup
- ROS 2 architecture and design philosophy (vs. ROS 1)
- Installation and workspace management with colcon
- Nodes, topics, and publish/subscribe pattern
- Quality of Service (QoS) policies
- Visualization with RViz2

### Week 2: Advanced Communication Patterns
- Services and clients (request/response)
- Actions (long-running tasks with feedback and cancellation)
- Parameters and dynamic reconfiguration
- Recording and playback with ROS 2 bags
- Multi-node launch files with XML/YAML/Python

### Week 3: Navigation & Coordinate Frames
- tf2 transform library
- URDF robot descriptions and xacro macros
- Navigation2 architecture
- Costmaps and path planning algorithms
- Integration of localization (AMCL) and mapping (SLAM)

## Hands-On Labs

| Lab | Title | Key Skills |
|-----|-------|-----------|
| 1.1 | ROS 2 Installation | Workspace setup, colcon build |
| 1.2 | Publisher/Subscriber | Topic communication, message types |
| 1.3 | RViz Visualization | TF frames, sensor data display |
| 2.1 | Service Calculator | Synchronous request/response |
| 2.2 | Navigation Action | Action servers, goal feedback |
| 2.3 | Launch Files | Parameter loading, node configuration |
| 3.1 | URDF Robot Model | Joints, links, sensor frames |
| 3.2 | TF Broadcasting | Dynamic transforms, parent/child frames |
| 3.3 | Nav2 Configuration | Waypoint navigation, obstacle avoidance |

## Module Project: Autonomous Navigation System

Build a complete autonomous navigation system where a differential-drive robot navigates a warehouse environment while avoiding obstacles.

**Requirements**:
- Custom URDF robot with laser scanner and wheel odometry
- Navigate to 5 predefined waypoints in sequence
- Real-time obstacle avoidance using laser scan data
- Launch file to start entire system with one command
- Success rate ≥90% over 10 runs

**Deliverables**:
- Git repository with clean commit history
- README with architecture diagram and setup instructions
- 5-minute demo video showing successful navigation
- Brief reflection on challenges encountered

## Key Concepts

### Node Graph Communication
```
┌─────────────┐         ┌──────────────┐
│   Camera    │         │  Laser Scan  │
│   Publisher │         │   Publisher  │
└──────┬──────┘         └──────┬───────┘
       │ /image_raw            │ /scan
       ▼                       ▼
┌────────────────────────────────────┐
│      Perception Node               │
│  (Object Detection + Localization) │
└─────────────────┬──────────────────┘
                  │ /detected_objects
                  ▼
┌────────────────────────────────────┐
│       Planning Node                │
│  (Nav2 + Behavior Trees)           │
└─────────────────┬──────────────────┘
                  │ /cmd_vel
                  ▼
┌────────────────────────────────────┐
│       Motor Controller             │
└────────────────────────────────────┘
```

### ROS 2 vs. ROS 1 Key Differences

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Architecture** | Master-based (single point of failure) | DDS (distributed, no master) |
| **Real-Time** | No | Yes (with RTOS) |
| **Security** | None | DDS Security (encryption, authentication) |
| **Python Support** | Python 2 | Python 3 |
| **Windows Support** | Poor | Native |
| **Communication** | TCPROS/UDPROS | DDS (multiple vendors) |

## Resources

### Official Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [tf2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### Recommended Reading
- *A Gentle Introduction to ROS 2* by Jason M. O'Kane
- [ROS 2 Design Decisions](https://design.ros2.org/)
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/index.html)

### Community
- [ROS Discourse Forum](https://discourse.ros.org/)
- [Robotics Stack Exchange](https://robotics.stackexchange.com/)
- [ROS 2 GitHub Discussions](https://github.com/ros2/ros2/discussions)

## Common Pitfalls

1. **Forgetting to source workspace**: Always run `source install/setup.bash`
2. **QoS mismatch**: Publisher/subscriber QoS policies must be compatible
3. **Transform lookup timing**: Use `lookup_transform` with `timeout` for robustness
4. **Coordinate frame conventions**: Follow REP-103 (x-forward, z-up)
5. **URDF validation**: Always validate with `check_urdf` before running

## Next Steps

After completing this module, proceed to:

**[Module 2: Digital Twin & Gazebo Simulation →](../module-2/index)**

Learn to build high-fidelity simulations for testing robot algorithms before hardware deployment.

---

**Start Learning**: [ROS 2 Basics →](./ros2-basics)

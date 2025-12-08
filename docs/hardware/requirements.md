---
title: Hardware Requirements
sidebar_label: Hardware Requirements
description: Complete hardware specification for the Physical AI & Humanoid Robotics course
keywords: [hardware, robotics, embedded systems, gpu, jetson, sensors]
---

# Hardware Requirements

This page details the complete hardware requirements for the Physical AI & Humanoid Robotics course. We've designed two configurations: a Digital Twin Workstation for simulation and development, and a Physical AI Edge Kit for real-world deployment.

## Digital Twin Workstation

For simulation, development, and AI model training.

### Minimum Specifications
- **GPU**: NVIDIA RTX 4060 (8GB VRAM) or equivalent
- **CPU**: AMD Ryzen 7 / Intel i7 (8 cores, 16 threads)
- **RAM**: 32GB DDR4-3200 
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS or Windows 10/11 Pro
- **Network**: Gigabit Ethernet (1 Gbps)

### Recommended Specifications
- **GPU**: NVIDIA RTX 4080/4090 or RTX A4000/A5000 (For Isaac Sim)
- **CPU**: AMD Ryzen 9 7950X / Intel i9-13900K (16+ cores)
- **RAM**: 64GB DDR5-5600 
- **Storage**: 2TB+ NVMe SSD
- **OS**: Ubuntu 22.04 LTS (Recommended for ROS 2 compatibility)
- **Network**: WiFi 6E or 10GbE (for high-bandwidth sensor streaming)

### Why These Specs?

| Component | Requirement | Rationale |
|-----------|-------------|-----------|
| **VRAM** | 8GB+ | Isaac Sim requires substantial GPU memory for photorealistic rendering |
| **CUDA Cores** | High count | AI model training and inference acceleration |
| **RAM** | 32GB+ | Multiple simulation instances + development tools + ML frameworks |
| **CPU Cores** | 8+ | Parallel processing for sensor fusion and control algorithms |
| **Storage** | NVMe SSD | Fast asset loading for simulation environments |

## Physical AI Edge Kit

For hardware-in-the-loop testing and real-world deployment.

### Core Platform
- **Compute**: NVIDIA Jetson Orin AGX (64GB RAM version)
- **Performance**: 275 TOPS AI performance, 512 CUDA cores
- **Power**: 60W max (can be configured lower)
- **Interfaces**: Multiple CSI cameras, GPIO, CAN bus, ethernet

### Vision Sensors
- **Depth Camera**: Intel RealSense D455 or Stereolabs ZED 2
  - RealSense: 1280×720 depth at 90 FPS
  - ZED 2: Stereo depth with wider FOV
- **RGB Cameras**: 2-4x USB Vision cameras for 360° coverage
  - Resolution: 720p/1080p at 30+ FPS
  - Interface: USB 3.0/3.1 for low latency

### Actuation & Mobility
- **Platform**: Clearpath Jackal UGV or custom 4WD differential drive
  - Payload capacity: 15kg+
  - Runtime: 2+ hours with AI compute
  - ROS 2 support: Native integration
- **Manipulator** (optional): WidowX 250S or equivalent 5-7 DOF arm
  - Reach: 300mm+
  - Gripper: Adaptive or parallel jaw
  - Perception: wrist-mounted camera

### Peripherals
- **Audio**: ReSpeaker 6-Mic Array for voice commands
- **IMU**: Bosch BNO055 or equivalent for state estimation
- **LiDAR**: SICK TIM571 or similar for navigation (if not using vision-only SLAM)
- **Power**: 11.1V 10,000mAh LiPo battery with voltage monitoring

## Alternative Hardware Configurations

### Budget Option (Reduced Capability)
- **Workstation**: RTX 3060/3070 (for lighter simulation)
- **Edge**: Jetson Nano (for basic inference, limited model complexity)
- **Trade-offs**: Reduced simulation fidelity, limited AI model size

### High-Performance Option (Research Grade)
- **Workstation**: Dual RTX 6000 Ada or H100 for advanced research
- **Edge**: Custom carrier board with RTX 4060 Mobile for enhanced inference
- **Extras**: Multiple depth cameras, thermal imaging, IMU arrays

## Hardware Integration Guide

### Setting Up Jetson for Physical AI

```bash
# Enable Jetson power mode for maximum performance
sudo nvpmodel -m 0  # Maximum performance mode

# Set Jetson clock speeds to maximum
sudo jetson_clocks  # Lock clocks to maximum boost

# Install Isaac ROS dependencies
sudo apt update
sudo apt install ros-humble-isaac-ros-*  # Install all Isaac ROS packages

# Install vision packages
sudo apt install ros-humble-vision-opencv ros-humble-image-transport
```

### Sensor Calibration Process

1. **Camera Intrinsics**: Use ROS 2 calibration tools
```bash
# For monocular calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

2. **Extrinsics (between sensors)**: Hand-eye calibration
```bash
# Between depth camera and robot frame
ros2 run robot_calibration calibrate --robot-config robot_cal_config.yaml
```

3. **Timing Synchronization**: For temporal accuracy
```bash
# Use hardware sync if available, else software timestamp alignment
# Configure camera driver for accurate timestamps
```

## Procurement Strategy

### University/Classroom Deployment
- **Bulk purchase**: Get education discounts from NVIDIA, Intel, Clearpath
- **Leasing options**: Consider 3-year lease-to-own for latest hardware
- **Centralized management**: Use NVIDIA Fleet Command for device management

### Individual Student Purchase
- **Starter kit**: Focus on workstation first, add physical kit later
- **Used market**: Good condition high-end GPUs retain value well
- **Cloud alternatives**: AWS G4dn, LambdaLabs for training, local dev for testing

## Maintenance & Upkeep

### Cleaning Schedule
- **Monthly**: Dust GPU heatsinks and case fans
- **Quarterly**: Check thermal paste condition
- **Annually**: Verify sensor calibration and mechanical wear

### Backup Strategy
- **Images**: Maintain master disk images for rapid recovery
- **Calibrations**: Backup sensor calibration files regularly
- **Models**: Sync trained models to cloud storage

## Troubleshooting Common Hardware Issues

### GPU Memory Issues
```bash
# Check current GPU memory usage
nvidia-smi

# Restart NVIDIA drivers (if stuck)
sudo rmmod nvidia_uvm nvidia_drm nvidia_modeset nvidia
sudo modprobe nvidia
```

### Camera Connection Problems
```bash
# Check USB device enumeration
lsusb

# Reset USB subsystem
echo 0 | sudo tee /sys/bus/pci/devices/0000:xx:00.0/remove
echo 1 | sudo tee /sys/bus/pci/devices/0000:xx:00.0/rescan
```

### Jetson Power Management
```bash
# Check power mode
sudo nvpmodel -q

# Ensure consistent performance
sudo systemctl disable nvpowermgmt
```

## Recommended Vendors

### For Universities
- **NVIDIA**: Education program for Isaac ecosystem
- **Clearpath Robotics**: Academic discounts and support
- **Intel**: Education pricing on RealSense cameras
- **Synaptics (formerly Pozyx)**: Ultra-wideband localization systems

### Retail
- **Micro Center**: High-end GPUs with competitive pricing
- **Ple Computer**: Specialized robotics hardware
- **SparkFun/Adafruit**: Peripherals and accessories

<details>
<summary>Sample Bill of Materials (Budget Estimate)</summary>

**Digital Twin Workstation (Recommended)**:
- CPU: AMD Ryzen 9 7950X ($700)
- GPU: RTX 4090 ($1600)
- RAM: 64GB DDR5 ($400)
- Motherboard: AM5 Compatible ($250)
- PSU: 1000W 80+ Gold ($150)
- Storage: 2TB NVMe SSD ($150)
- Case: Mid-tower with cooling ($100)
- **Total**: ~$3350

**Physical AI Edge Kit**:
- Jetson Orin AGX 64GB: $2200
- Clearpath Jackal UGV: $8000
- Intel RealSense D455: $200
- ReSpeaker Array: $150
- Custom manipulator: $3000
- **Total**: ~$13,550 (shared among multiple students)

**Alternative Edge Kit (Student Budget)**:
- Jetson Orin NX Carrier: $600
- TurtleBot 4: $2500
- Realsense D435: $150
- Basic gripper: $400
- **Total**: ~$3650
</details>

---

**Next**: [System Architecture →](../architecture/system-overview)
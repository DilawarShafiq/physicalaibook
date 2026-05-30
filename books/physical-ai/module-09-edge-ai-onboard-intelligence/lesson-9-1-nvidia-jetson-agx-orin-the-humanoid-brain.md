---
title: "NVIDIA Jetson AGX Orin: The Humanoid Brain"
sidebar_label: "9.1 NVIDIA Jetson AGX Orin: The Humanoid Brain"
sidebar_position: 2
description: "NVIDIA Jetson AGX Orin (2022) is the de facto standard compute platform for robotics AI"
tags: ["Orin", "compute", "NVIDIA", "hardware"]
---

# NVIDIA Jetson AGX Orin: The Humanoid Brain

**Duration:** 50 min · **Level:** Advanced · **Module:** 9. Edge AI & On-Board Intelligence · **Tags:** `Orin`, `compute`, `NVIDIA`, `hardware`

## Overview

NVIDIA Jetson AGX Orin (2022) is the de facto standard compute platform for robotics AI. Its combination of 275 TOPS of INT8 performance, 64GB LPDDR5 memory, and NVMe storage — in a 100W power envelope — makes it the only currently available platform that can run VLA inference on a battery-powered robot.

## Key Insights

- AGX Orin specs: 12-core Arm Cortex-A78AE CPU, 2048-core Ampere GPU with 64 Tensor Cores, 64GB LPDDR5; 275 TOPS INT8 / 135 TOPS FP16
- Power modes: 15W (max efficiency) to 60W (max performance); thermal design targets 100W peak; must budget power vs compute tradeoff carefully
- Model performance on AGX Orin: LLaMA-7B INT4 at ~5 tokens/s; OpenVLA 7B quantized at ~1.5 Hz; DepthAnything at 30 FPS; Grounded DINO at 15 FPS
- Multi-module setup: Figure 02 uses 2× Orin NX (10W each) modules — one for perception pipeline, one for motor control; avoids thermal throttling of single module
- Memory bandwidth: 204 GB/s on AGX Orin; limits large model inference more than TOPS; quantization reduces bandwidth requirement proportionally
- NVIDIA Isaac ROS: hardware-accelerated ROS 2 packages optimized for Orin; SLAM, perception, and NN inference run at 2-3× CPU speed using CUDA-accelerated nodes


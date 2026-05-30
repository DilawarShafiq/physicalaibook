---
title: "NVIDIA Jetson AGX Orin: The Humanoid Brain"
sidebar_label: "9.1 NVIDIA Jetson AGX Orin: The Humanoid Brain"
sidebar_position: 2
description: "NVIDIA Jetson AGX Orin (2022) is the de facto standard compute platform for robotics AI"
tags: ["Orin", "compute", "NVIDIA", "hardware"]
---

# NVIDIA Jetson AGX Orin: The Humanoid Brain

**Duration:** 50 min · **Level:** Advanced · **Module:** 9. Edge AI & On-Board Intelligence · **Focus:** `Orin`, `compute`, `NVIDIA`, `hardware`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- AGX Orin specs
- Power modes
- Model performance on AGX Orin
- Multi-module setup
- Memory bandwidth
:::

## Why this matters

NVIDIA Jetson AGX Orin (2022) is the de facto standard compute platform for robotics AI.

## Overview

NVIDIA Jetson AGX Orin (2022) is the de facto standard compute platform for robotics AI. Its combination of 275 TOPS of INT8 performance, 64GB LPDDR5 memory, and NVMe storage — in a 100W power envelope — makes it the only currently available platform that can run VLA inference on a battery-powered robot.

## Key concepts

:::note Key idea

AGX Orin specs: 12-core Arm Cortex-A78AE CPU, 2048-core Ampere GPU with 64 Tensor Cores, 64GB LPDDR5; 275 TOPS INT8 / 135 TOPS FP16

:::

- Power modes: 15W (max efficiency) to 60W (max performance); thermal design targets 100W peak; must budget power vs compute tradeoff carefully
- Model performance on AGX Orin: LLaMA-7B INT4 at ~5 tokens/s; OpenVLA 7B quantized at ~1.5 Hz; DepthAnything at 30 FPS; Grounded DINO at 15 FPS
- Multi-module setup: Figure 02 uses 2× Orin NX (10W each) modules — one for perception pipeline, one for motor control; avoids thermal throttling of single module
- Memory bandwidth: 204 GB/s on AGX Orin; limits large model inference more than TOPS; quantization reduces bandwidth requirement proportionally
- NVIDIA Isaac ROS: hardware-accelerated ROS 2 packages optimized for Orin; SLAM, perception, and NN inference run at 2-3× CPU speed using CUDA-accelerated nodes

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about AGX Orin specs?</summary>

12-core Arm Cortex-A78AE CPU, 2048-core Ampere GPU with 64 Tensor Cores, 64GB LPDDR5; 275 TOPS INT8 / 135 TOPS FP16

</details>

<details>
<summary>Q2. What do you know about Power modes?</summary>

15W (max efficiency) to 60W (max performance); thermal design targets 100W peak; must budget power vs compute tradeoff carefully

</details>

<details>
<summary>Q3. What do you know about Model performance on AGX Orin?</summary>

LLaMA-7B INT4 at ~5 tokens/s; OpenVLA 7B quantized at ~1.5 Hz; DepthAnything at 30 FPS; Grounded DINO at 15 FPS

</details>

<details>
<summary>Q4. What do you know about Multi-module setup?</summary>

Figure 02 uses 2× Orin NX (10W each) modules — one for perception pipeline, one for motor control; avoids thermal throttling of single module

</details>

<details>
<summary>Q5. What do you know about Memory bandwidth?</summary>

204 GB/s on AGX Orin; limits large model inference more than TOPS; quantization reduces bandwidth requirement proportionally

</details>

---

Next: **9.2 Model Compression for Edge Deployment** →

*Part of Module 9: Edge AI & On-Board Intelligence.*


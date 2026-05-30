---
title: "Building a Physics-Accurate Digital Twin of G1"
sidebar_label: "7.3 Building a Physics-Accurate Digital Twin of G1"
sidebar_position: 4
description: "A digital twin is a live, synchronized simulation of the real robot — updated in real-time from sensor data"
tags: ["digital-twin", "URDF", "Isaac-Sim", "modeling"]
---

# Building a Physics-Accurate Digital Twin of G1

**Duration:** 60 min · **Level:** Intermediate · **Module:** 7. Simulation & Digital Twins · **Focus:** `digital-twin`, `URDF`, `Isaac-Sim`, `modeling`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- URDF → USD pipeline
- Inertial parameters
- Actuator modeling
- Sensor simulation
- State estimation bridge
:::

## Why this matters

A digital twin is a live, synchronized simulation of the real robot — updated in real-time from sensor data.

## Overview

A digital twin is a live, synchronized simulation of the real robot — updated in real-time from sensor data. For G1 development, the digital twin serves three purposes: visualization, predictive maintenance, and policy pre-testing before hardware deployment.

## Key concepts

:::note Key idea

URDF → USD pipeline: convert G1's URDF (Unified Robot Description Format) to USD for Isaac Sim; NVIDIA provides isaac_ros_urdf for automated conversion

:::

- Inertial parameters: measured vs nominal inertia tensors for each link differ by 5-20%; use system identification (swing experiments) to measure actual parameters
- Actuator modeling: add motor dynamics to simulation (current limiting, back-EMF, thermal derating model) for accurate torque prediction
- Sensor simulation: add realistic noise models for each sensor based on manufacturer datasheets; IMU noise, camera calibration errors, encoder quantization
- State estimation bridge: real-time pose estimate from onboard SLAM → update digital twin state; latency &lt;50ms for useful synchronization
- Use cases: predict joint wear patterns, test new locomotion policies in digital twin before hardware deployment, train maintenance technicians on correct repair procedures

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about URDF → USD pipeline?</summary>

convert G1's URDF (Unified Robot Description Format) to USD for Isaac Sim; NVIDIA provides isaac_ros_urdf for automated conversion

</details>

<details>
<summary>Q2. What do you know about Inertial parameters?</summary>

measured vs nominal inertia tensors for each link differ by 5-20%; use system identification (swing experiments) to measure actual parameters

</details>

<details>
<summary>Q3. What do you know about Actuator modeling?</summary>

add motor dynamics to simulation (current limiting, back-EMF, thermal derating model) for accurate torque prediction

</details>

<details>
<summary>Q4. What do you know about Sensor simulation?</summary>

add realistic noise models for each sensor based on manufacturer datasheets; IMU noise, camera calibration errors, encoder quantization

</details>

<details>
<summary>Q5. What do you know about State estimation bridge?</summary>

real-time pose estimate from onboard SLAM → update digital twin state; latency &lt;50ms for useful synchronization

</details>

---

← Previous: **7.2 Domain Randomization: The Bridge from Sim to Real**

*Part of Module 7: Simulation & Digital Twins.*


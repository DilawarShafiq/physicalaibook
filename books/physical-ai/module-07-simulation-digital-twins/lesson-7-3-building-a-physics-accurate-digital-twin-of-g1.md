---
title: "Building a Physics-Accurate Digital Twin of G1"
sidebar_label: "7.3 Building a Physics-Accurate Digital Twin of G1"
sidebar_position: 4
description: "A digital twin is a live, synchronized simulation of the real robot — updated in real-time from sensor data"
tags: ["digital-twin", "URDF", "Isaac-Sim", "modeling"]
---

# Building a Physics-Accurate Digital Twin of G1

**Duration:** 60 min · **Level:** Intermediate · **Module:** 7. Simulation & Digital Twins · **Tags:** `digital-twin`, `URDF`, `Isaac-Sim`, `modeling`

## Overview

A digital twin is a live, synchronized simulation of the real robot — updated in real-time from sensor data. For G1 development, the digital twin serves three purposes: visualization, predictive maintenance, and policy pre-testing before hardware deployment.

## Key Insights

- URDF → USD pipeline: convert G1's URDF (Unified Robot Description Format) to USD for Isaac Sim; NVIDIA provides isaac_ros_urdf for automated conversion
- Inertial parameters: measured vs nominal inertia tensors for each link differ by 5-20%; use system identification (swing experiments) to measure actual parameters
- Actuator modeling: add motor dynamics to simulation (current limiting, back-EMF, thermal derating model) for accurate torque prediction
- Sensor simulation: add realistic noise models for each sensor based on manufacturer datasheets; IMU noise, camera calibration errors, encoder quantization
- State estimation bridge: real-time pose estimate from onboard SLAM → update digital twin state; latency &lt;50ms for useful synchronization
- Use cases: predict joint wear patterns, test new locomotion policies in digital twin before hardware deployment, train maintenance technicians on correct repair procedures


---
title: "Sensor Suite Design for Humanoids"
sidebar_label: "4.1 Sensor Suite Design for Humanoids"
sidebar_position: 2
description: "A humanoid's sensor suite must provide sufficient information for navigation, manipulation, and safe human interaction — while fitting within size, weight, and power constraints"
tags: ["sensors", "perception", "hardware", "design"]
---

# Sensor Suite Design for Humanoids

**Duration:** 45 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Focus:** `sensors`, `perception`, `hardware`, `design`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- RGB-D (depth cameras)
- Fisheye / wide-angle cameras
- IMU (Inertial Measurement Unit)
- Wrist F/T sensors
- Tactile fingertip sensors
:::

## Why this matters

A humanoid's sensor suite must provide sufficient information for navigation, manipulation, and safe human interaction — while fitting within size, weight, and power constraints.

## Overview

A humanoid's sensor suite must provide sufficient information for navigation, manipulation, and safe human interaction — while fitting within size, weight, and power constraints. The 2024 consensus configuration combines RGB-D cameras, wide-angle fisheye cameras, IMU, and force/torque sensors at the wrists.

## Key concepts

:::note Key idea

RGB-D (depth cameras): Intel RealSense D435i, Microsoft Azure Kinect, or Orbbec Astra — typically 2 forward-facing for stereo + depth, 1 downward for foot placement

:::

- Fisheye / wide-angle cameras: provide 180°+ field of view for peripheral awareness; critical for detecting humans approaching from the side
- IMU (Inertial Measurement Unit): at minimum 1 high-quality IMU at the pelvis; 2 IMUs (pelvis + head) preferred for improved balance estimation
- Wrist F/T sensors: 6-DOF force/torque at each wrist provides manipulation force feedback; ATI Mini45 or custom piezoelectric designs
- Tactile fingertip sensors: GelSight-style or DIGIT sensors on each fingertip for contact detection; essential for dexterous manipulation in healthcare
- Microphone array: 4-8 mics in circular array on head for sound localization and noise-robust speech recognition in clinical environments

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about RGB-D (depth cameras)?</summary>

Intel RealSense D435i, Microsoft Azure Kinect, or Orbbec Astra — typically 2 forward-facing for stereo + depth, 1 downward for foot placement

</details>

<details>
<summary>Q2. What do you know about Fisheye / wide-angle cameras?</summary>

provide 180°+ field of view for peripheral awareness; critical for detecting humans approaching from the side

</details>

<details>
<summary>Q3. What do you know about IMU (Inertial Measurement Unit)?</summary>

at minimum 1 high-quality IMU at the pelvis; 2 IMUs (pelvis + head) preferred for improved balance estimation

</details>

<details>
<summary>Q4. What do you know about Wrist F/T sensors?</summary>

6-DOF force/torque at each wrist provides manipulation force feedback; ATI Mini45 or custom piezoelectric designs

</details>

<details>
<summary>Q5. What do you know about Tactile fingertip sensors?</summary>

GelSight-style or DIGIT sensors on each fingertip for contact detection; essential for dexterous manipulation in healthcare

</details>

---

Next: **4.2 Real-Time SLAM for Indoor Navigation** →

*Part of Module 4: Perception & Spatial Intelligence.*


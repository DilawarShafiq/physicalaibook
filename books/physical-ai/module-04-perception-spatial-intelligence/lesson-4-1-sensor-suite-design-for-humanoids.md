---
title: "Sensor Suite Design for Humanoids"
sidebar_label: "4.1 Sensor Suite Design for Humanoids"
sidebar_position: 2
description: "A humanoid's sensor suite must provide sufficient information for navigation, manipulation, and safe human interaction — while fitting within size, weight, and power constraints"
tags: ["sensors", "perception", "hardware", "design"]
---

# Sensor Suite Design for Humanoids

**Duration:** 45 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Tags:** `sensors`, `perception`, `hardware`, `design`

## Overview

A humanoid's sensor suite must provide sufficient information for navigation, manipulation, and safe human interaction — while fitting within size, weight, and power constraints. The 2024 consensus configuration combines RGB-D cameras, wide-angle fisheye cameras, IMU, and force/torque sensors at the wrists.

## Key Insights

- RGB-D (depth cameras): Intel RealSense D435i, Microsoft Azure Kinect, or Orbbec Astra — typically 2 forward-facing for stereo + depth, 1 downward for foot placement
- Fisheye / wide-angle cameras: provide 180°+ field of view for peripheral awareness; critical for detecting humans approaching from the side
- IMU (Inertial Measurement Unit): at minimum 1 high-quality IMU at the pelvis; 2 IMUs (pelvis + head) preferred for improved balance estimation
- Wrist F/T sensors: 6-DOF force/torque at each wrist provides manipulation force feedback; ATI Mini45 or custom piezoelectric designs
- Tactile fingertip sensors: GelSight-style or DIGIT sensors on each fingertip for contact detection; essential for dexterous manipulation in healthcare
- Microphone array: 4-8 mics in circular array on head for sound localization and noise-robust speech recognition in clinical environments


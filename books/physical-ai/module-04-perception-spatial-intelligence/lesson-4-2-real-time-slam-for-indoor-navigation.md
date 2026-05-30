---
title: "Real-Time SLAM for Indoor Navigation"
sidebar_label: "4.2 Real-Time SLAM for Indoor Navigation"
sidebar_position: 3
description: "Simultaneous Localization and Mapping (SLAM) allows G1 to build a map of an environment while tracking its own position within it — without GPS"
tags: ["SLAM", "localization", "mapping", "navigation"]
---

# Real-Time SLAM for Indoor Navigation

**Duration:** 55 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Tags:** `SLAM`, `localization`, `mapping`, `navigation`

## Overview

Simultaneous Localization and Mapping (SLAM) allows G1 to build a map of an environment while tracking its own position within it — without GPS. For indoor hospital or home environments, LiDAR SLAM or visual SLAM provides centimeter-level localization accuracy.

## Key Insights

- ORB-SLAM3 (2021): state-of-the-art visual SLAM; runs in real-time on CPU; supports monocular, stereo, and RGB-D; robust to dynamic scenes with people moving
- LiDAR SLAM (LIO-SAM, FAST-LIO2): centimeter accuracy in large environments; too heavy and expensive for lightweight humanoids but used in industrial deployments
- Semantic SLAM: extends metric SLAM with object labels; "I am 3m from the kitchen counter and 1.5m from the chair" — enables more natural task planning
- Loop closure: detect when robot returns to a previously visited location; correct accumulated drift in pose estimate; critical for long-duration operation
- Dynamic objects: humans, doors, moving furniture are obstacles to SLAM; modern approaches use semantic segmentation to mask dynamic elements from map updates
- Map types: occupancy grid (navigation), point cloud (3D geometry), semantic map (object labels + positions); G1 needs all three updated in real-time

## References

- **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM** — Campos et al. (2021). *IEEE T-RO 2021*


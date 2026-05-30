---
title: "3D Gaussian Splatting for Robot Scene Understanding"
sidebar_label: "4.3 3D Gaussian Splatting for Robot Scene Understanding"
sidebar_position: 4
description: "3D Gaussian Splatting (3DGS), introduced at SIGGRAPH 2023, represents scenes as millions of 3D Gaussians and achieves real-time photorealistic rendering"
tags: ["3DGS", "NeRF", "scene-understanding", "representation"]
---

# 3D Gaussian Splatting for Robot Scene Understanding

**Duration:** 50 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Focus:** `3DGS`, `NeRF`, `scene-understanding`, `representation`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- 3DGS (Kerbl et al., SIGGRAPH 2023)
- LangSplat (2024)
- Feature3DGS (2024)
- Robotic application
- Online update
:::

## Why this matters

3D Gaussian Splatting (3DGS), introduced at SIGGRAPH 2023, represents scenes as millions of 3D Gaussians and achieves real-time photorealistic rendering.

## Overview

3D Gaussian Splatting (3DGS), introduced at SIGGRAPH 2023, represents scenes as millions of 3D Gaussians and achieves real-time photorealistic rendering. For robotics, it enables rapid scene reconstruction from RGB images and, with semantic extensions, provides a queryable 3D understanding of the environment.

## Key concepts

:::note Key idea

3DGS (Kerbl et al., SIGGRAPH 2023): 100× faster rendering than NeRF; 30+ FPS at 1080p; scene reconstruction from 100-200 photos in ~10 minutes on a single GPU

:::

- LangSplat (2024): adds CLIP language features to each Gaussian; enables queries like "find the cup" without any additional training — returns 3D location of matching objects
- Feature3DGS (2024): distills features from 2D foundation models (SAM, DINO) into 3D Gaussians; enables segmentation and part-level scene understanding in 3D
- Robotic application: scan a new room in 60 seconds using onboard cameras → build 3DGS scene → query with natural language for object locations → plan manipulation
- Online update: incremental 3DGS allows adding new Gaussians as robot explores, handling dynamic environments without full reconstruction
- Limitation: requires good initial camera poses (from SLAM) and fails in textureless regions (plain white walls); combine with LiDAR for robust reconstruction

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about 3DGS (Kerbl et al., SIGGRAPH 2023)?</summary>

100× faster rendering than NeRF; 30+ FPS at 1080p; scene reconstruction from 100-200 photos in ~10 minutes on a single GPU

</details>

<details>
<summary>Q2. What do you know about LangSplat (2024)?</summary>

adds CLIP language features to each Gaussian; enables queries like "find the cup" without any additional training — returns 3D location of matching objects

</details>

<details>
<summary>Q3. What do you know about Feature3DGS (2024)?</summary>

distills features from 2D foundation models (SAM, DINO) into 3D Gaussians; enables segmentation and part-level scene understanding in 3D

</details>

<details>
<summary>Q4. What do you know about Robotic application?</summary>

scan a new room in 60 seconds using onboard cameras → build 3DGS scene → query with natural language for object locations → plan manipulation

</details>

<details>
<summary>Q5. What do you know about Online update?</summary>

incremental 3DGS allows adding new Gaussians as robot explores, handling dynamic environments without full reconstruction

</details>

## References

- **3D Gaussian Splatting for Real-Time Radiance Field Rendering** — Kerbl et al. (2023). *SIGGRAPH 2023*
- **LangSplat: 3D Language Gaussian Splatting** — Qin et al. (2024). *CVPR 2024*

---

← Previous: **4.2 Real-Time SLAM for Indoor Navigation** · Next: **4.4 Foundation Models for Open-Vocabulary Perception** →

*Part of Module 4: Perception & Spatial Intelligence.*


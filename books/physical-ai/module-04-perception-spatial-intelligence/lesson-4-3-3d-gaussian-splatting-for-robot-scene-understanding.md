---
title: "3D Gaussian Splatting for Robot Scene Understanding"
sidebar_label: "4.3 3D Gaussian Splatting for Robot Scene Understanding"
sidebar_position: 4
description: "3D Gaussian Splatting (3DGS), introduced at SIGGRAPH 2023, represents scenes as millions of 3D Gaussians and achieves real-time photorealistic rendering"
tags: ["3DGS", "NeRF", "scene-understanding", "representation"]
---

# 3D Gaussian Splatting for Robot Scene Understanding

**Duration:** 50 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Tags:** `3DGS`, `NeRF`, `scene-understanding`, `representation`

## Overview

3D Gaussian Splatting (3DGS), introduced at SIGGRAPH 2023, represents scenes as millions of 3D Gaussians and achieves real-time photorealistic rendering. For robotics, it enables rapid scene reconstruction from RGB images and, with semantic extensions, provides a queryable 3D understanding of the environment.

## Key Insights

- 3DGS (Kerbl et al., SIGGRAPH 2023): 100× faster rendering than NeRF; 30+ FPS at 1080p; scene reconstruction from 100-200 photos in ~10 minutes on a single GPU
- LangSplat (2024): adds CLIP language features to each Gaussian; enables queries like "find the cup" without any additional training — returns 3D location of matching objects
- Feature3DGS (2024): distills features from 2D foundation models (SAM, DINO) into 3D Gaussians; enables segmentation and part-level scene understanding in 3D
- Robotic application: scan a new room in 60 seconds using onboard cameras → build 3DGS scene → query with natural language for object locations → plan manipulation
- Online update: incremental 3DGS allows adding new Gaussians as robot explores, handling dynamic environments without full reconstruction
- Limitation: requires good initial camera poses (from SLAM) and fails in textureless regions (plain white walls); combine with LiDAR for robust reconstruction

## References

- **3D Gaussian Splatting for Real-Time Radiance Field Rendering** — Kerbl et al. (2023). *SIGGRAPH 2023*
- **LangSplat: 3D Language Gaussian Splatting** — Qin et al. (2024). *CVPR 2024*


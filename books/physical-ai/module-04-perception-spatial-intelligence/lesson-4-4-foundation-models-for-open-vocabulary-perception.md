---
title: "Foundation Models for Open-Vocabulary Perception"
sidebar_label: "4.4 Foundation Models for Open-Vocabulary Perception"
sidebar_position: 5
description: "Traditional object detection required training a separate model for every object class"
tags: ["foundation-models", "detection", "segmentation", "open-vocabulary"]
---

# Foundation Models for Open-Vocabulary Perception

**Duration:** 55 min · **Level:** Intermediate · **Module:** 4. Perception & Spatial Intelligence · **Focus:** `foundation-models`, `detection`, `segmentation`, `open-vocabulary`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Grounded DINO (2023)
- SAM 2 (Meta, 2024)
- OWL-ViT (Google, 2022)
- Combined pipeline
- Real-time performance
:::

## Overview

Traditional object detection required training a separate model for every object class. Foundation models like Grounded DINO, SAM 2, and OWL-ViT provide open-vocabulary detection and segmentation: "find the orange pill bottle" works without any specific training on pill bottles.

## Key concepts

:::note Key idea

Grounded DINO (2023): combines DINO (vision transformer) with BERT text encoder; detects any object described in natural language with SOTA zero-shot performance

:::

- SAM 2 (Meta, 2024): Segment Anything Model 2 — zero-shot image and video segmentation; takes a point click or bounding box and produces pixel-perfect mask in real-time
- OWL-ViT (Google, 2022): open-vocabulary detection using CLIP-style image-text alignment; works on novel objects not seen during training
- Combined pipeline: Grounded DINO detects + bounding boxes → SAM 2 generates precise 3D mask → depth image gives 3D position → robot plans grasp
- Real-time performance: Grounded DINO ~15 FPS on NVIDIA Orin AGX; SAM 2 ~30 FPS; combined pipeline ~8-10 Hz — sufficient for manipulation at human hand speed
- Healthcare application: "bring me the blood pressure cuff" — robot identifies cuff in cluttered medical cart, segments it precisely, grasps without disturbing other equipment

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>Grounded DINO (2023)</summary>

combines DINO (vision transformer) with BERT text encoder; detects any object described in natural language with SOTA zero-shot performance

</details>

<details>
<summary>SAM 2 (Meta, 2024)</summary>

Segment Anything Model 2 — zero-shot image and video segmentation; takes a point click or bounding box and produces pixel-perfect mask in real-time

</details>

<details>
<summary>OWL-ViT (Google, 2022)</summary>

open-vocabulary detection using CLIP-style image-text alignment; works on novel objects not seen during training

</details>

<details>
<summary>Combined pipeline</summary>

Grounded DINO detects + bounding boxes → SAM 2 generates precise 3D mask → depth image gives 3D position → robot plans grasp

</details>

<details>
<summary>Real-time performance</summary>

Grounded DINO ~15 FPS on NVIDIA Orin AGX; SAM 2 ~30 FPS; combined pipeline ~8-10 Hz — sufficient for manipulation at human hand speed

</details>

## References

- **Grounded Language-Image Pre-Training** — Li et al. (2023). *CVPR 2022*
- **SAM 2: Segment Anything in Images and Videos** — Ravi et al. (2024). *Meta AI Research 2024*

---

← Previous: **4.3 3D Gaussian Splatting for Robot Scene Understanding**

*Part of Module 4: Perception & Spatial Intelligence.*


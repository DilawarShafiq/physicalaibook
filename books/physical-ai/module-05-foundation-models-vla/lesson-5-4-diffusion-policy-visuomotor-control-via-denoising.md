---
title: "Diffusion Policy — Visuomotor Control via Denoising"
sidebar_label: "5.4 Diffusion Policy — Visuomotor Control via Denoising"
sidebar_position: 5
description: "Diffusion Policy (Chi et al., RSS 2023) applies denoising diffusion probabilistic models to robot action generation"
tags: ["diffusion-policy", "visuomotor", "bimanual", "manipulation"]
---

# Diffusion Policy — Visuomotor Control via Denoising

**Duration:** 60 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Focus:** `diffusion-policy`, `visuomotor`, `bimanual`, `manipulation`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Key insight
- Two variants
- Architecture
- State-of-the-art on
- ACT (Action Chunking with Transformers, Zhao et al. 2023)
:::

## Why this matters

Diffusion Policy (Chi et al., RSS 2023) applies denoising diffusion probabilistic models to robot action generation.

## Overview

Diffusion Policy (Chi et al., RSS 2023) applies denoising diffusion probabilistic models to robot action generation. Unlike direct regression, it learns a score function that can represent multimodal action distributions — critical for tasks where multiple correct trajectories exist (e.g., "place the cup somewhere on the table").

## Key concepts

:::note Key idea

Key insight: direct regression predicts a single mean action; diffusion policy predicts a distribution; crucial for contact-rich tasks with multiple valid solutions

:::

- Two variants: DDPM-based (slower, 100-step denoising) and consistency policy (faster, 1-4 step denoising); for real-time use, consistency policy preferred
- Architecture: conditioning on RGB observations via ResNet/ViT encoder; denoising network is U-Net or transformer; action space is joint angles or end-effector deltas
- State-of-the-art on: RoboMimic, RLBench, BridgeV2; particularly strong on bimanual tasks and contact-rich manipulation
- ACT (Action Chunking with Transformers, Zhao et al. 2023): related approach using chunked action prediction; used in Stanford ALOHA bimanual robot system
- ALOHA 2 (Stanford, 2024): 14-DOF bimanual system trained with ACT on 50 demonstrations; achieves cooking, surgery simulation tasks with 90%+ success rate

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about Key insight?</summary>

direct regression predicts a single mean action; diffusion policy predicts a distribution; crucial for contact-rich tasks with multiple valid solutions

</details>

<details>
<summary>Q2. What do you know about Two variants?</summary>

DDPM-based (slower, 100-step denoising) and consistency policy (faster, 1-4 step denoising); for real-time use, consistency policy preferred

</details>

<details>
<summary>Q3. What do you know about Architecture?</summary>

conditioning on RGB observations via ResNet/ViT encoder; denoising network is U-Net or transformer; action space is joint angles or end-effector deltas

</details>

<details>
<summary>Q4. What do you know about State-of-the-art on?</summary>

RoboMimic, RLBench, BridgeV2; particularly strong on bimanual tasks and contact-rich manipulation

</details>

<details>
<summary>Q5. What do you know about ACT (Action Chunking with Transformers, Zhao et al. 2023)?</summary>

related approach using chunked action prediction; used in Stanford ALOHA bimanual robot system

</details>

## References

- **Diffusion Policy: Visuomotor Policy Learning via Action Diffusion** — Chi et al. (2023). *RSS 2023*
- **Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware** — Zhao et al. (2023). *RSS 2023*

---

← Previous: **5.3 OpenVLA — The Open-Source VLA Ecosystem** · Next: **5.5 Deploying VLAs on G1: Architecture & Integration** →

*Part of Module 5: Foundation Models & VLA Architecture.*


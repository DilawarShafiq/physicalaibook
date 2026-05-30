---
title: "Diffusion Policy — Visuomotor Control via Denoising"
sidebar_label: "5.4 Diffusion Policy — Visuomotor Control via Denoising"
sidebar_position: 5
description: "Diffusion Policy (Chi et al., RSS 2023) applies denoising diffusion probabilistic models to robot action generation"
tags: ["diffusion-policy", "visuomotor", "bimanual", "manipulation"]
---

# Diffusion Policy — Visuomotor Control via Denoising

**Duration:** 60 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Tags:** `diffusion-policy`, `visuomotor`, `bimanual`, `manipulation`

## Overview

Diffusion Policy (Chi et al., RSS 2023) applies denoising diffusion probabilistic models to robot action generation. Unlike direct regression, it learns a score function that can represent multimodal action distributions — critical for tasks where multiple correct trajectories exist (e.g., "place the cup somewhere on the table").

## Key Insights

- Key insight: direct regression predicts a single mean action; diffusion policy predicts a distribution; crucial for contact-rich tasks with multiple valid solutions
- Two variants: DDPM-based (slower, 100-step denoising) and consistency policy (faster, 1-4 step denoising); for real-time use, consistency policy preferred
- Architecture: conditioning on RGB observations via ResNet/ViT encoder; denoising network is U-Net or transformer; action space is joint angles or end-effector deltas
- State-of-the-art on: RoboMimic, RLBench, BridgeV2; particularly strong on bimanual tasks and contact-rich manipulation
- ACT (Action Chunking with Transformers, Zhao et al. 2023): related approach using chunked action prediction; used in Stanford ALOHA bimanual robot system
- ALOHA 2 (Stanford, 2024): 14-DOF bimanual system trained with ACT on 50 demonstrations; achieves cooking, surgery simulation tasks with 90%+ success rate

## References

- **Diffusion Policy: Visuomotor Policy Learning via Action Diffusion** — Chi et al. (2023). *RSS 2023*
- **Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware** — Zhao et al. (2023). *RSS 2023*


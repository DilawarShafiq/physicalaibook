---
title: "π0 — Diffusion-Based Whole-Body Control"
sidebar_label: "5.2 π0 — Diffusion-Based Whole-Body Control"
sidebar_position: 3
description: "Physical Intelligence (founded by ex-Google Robotics, Stanford, CMU researchers) published π0 in October 2024 — arguably the most important robotics paper since RT-2"
tags: ["pi0", "diffusion", "flow-matching", "physical-intelligence"]
---

# π0 — Diffusion-Based Whole-Body Control

**Duration:** 75 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Tags:** `pi0`, `diffusion`, `flow-matching`, `physical-intelligence`

## Overview

Physical Intelligence (founded by ex-Google Robotics, Stanford, CMU researchers) published π0 in October 2024 — arguably the most important robotics paper since RT-2. π0 uses flow matching (a continuous-time generalization of diffusion) to generate smooth, physically realistic action sequences for whole-body control tasks.

## Key Insights

- π0 architecture: PaliGemma 3B vision-language backbone + flow matching action expert; total ~3B parameters; runs at 50 Hz on robot
- Flow matching (vs diffusion): learns a velocity field that transforms noise → action; faster sampling (1-3 function evaluations vs 50+ for DDPM), smoother trajectories
- Training data: 10,000+ hours of teleoperation data across 7 robot platforms; tasks include laundry folding, box assembly, table clearing, egg packaging
- Whole-body result: π0 fine-tuned on 1-hour of data achieves 70%+ success on laundry folding (the hardest manipulation benchmark); previous SOTA was ~30%
- Fine-tuning efficiency: 1-10 hours of task-specific demonstrations fine-tunes the pretrained π0 to new tasks; dramatic reduction from previous 1000+ hours needed
- Physical Intelligence raised $400M Series B in 2024 at $2B valuation; pursuing the "foundation model for physical AI" market

:::tip Lab

Implement a simplified flow matching policy: define a 2D toy action space, implement the conditional flow matching loss, train on 100 demonstration trajectories, and visualize how the learned vector field transforms random noise into action samples.

:::

## References

- **π0: A Vision-Language-Action Flow Model for General Robot Control** — Black et al., Physical Intelligence (2024). *arXiv 2410.24164*


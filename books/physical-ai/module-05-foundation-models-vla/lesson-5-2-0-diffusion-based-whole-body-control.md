---
title: "π0 — Diffusion-Based Whole-Body Control"
sidebar_label: "5.2 π0 — Diffusion-Based Whole-Body Control"
sidebar_position: 3
description: "Physical Intelligence (founded by ex-Google Robotics, Stanford, CMU researchers) published π0 in October 2024 — arguably the most important robotics paper since RT-2"
tags: ["pi0", "diffusion", "flow-matching", "physical-intelligence"]
---

# π0 — Diffusion-Based Whole-Body Control

**Duration:** 75 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Focus:** `pi0`, `diffusion`, `flow-matching`, `physical-intelligence`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- π0 architecture
- Flow matching (vs diffusion)
- Training data
- Whole-body result
- Fine-tuning efficiency

You will then consolidate these ideas in the hands-on lab below.
:::

## Why this matters

Physical Intelligence (founded by ex-Google Robotics, Stanford, CMU researchers) published π0 in October 2024 — arguably the most important robotics paper since RT-2.

## Overview

Physical Intelligence (founded by ex-Google Robotics, Stanford, CMU researchers) published π0 in October 2024 — arguably the most important robotics paper since RT-2. π0 uses flow matching (a continuous-time generalization of diffusion) to generate smooth, physically realistic action sequences for whole-body control tasks.

## Key concepts

:::note Key idea

π0 architecture: PaliGemma 3B vision-language backbone + flow matching action expert; total ~3B parameters; runs at 50 Hz on robot

:::

- Flow matching (vs diffusion): learns a velocity field that transforms noise → action; faster sampling (1-3 function evaluations vs 50+ for DDPM), smoother trajectories
- Training data: 10,000+ hours of teleoperation data across 7 robot platforms; tasks include laundry folding, box assembly, table clearing, egg packaging
- Whole-body result: π0 fine-tuned on 1-hour of data achieves 70%+ success on laundry folding (the hardest manipulation benchmark); previous SOTA was ~30%
- Fine-tuning efficiency: 1-10 hours of task-specific demonstrations fine-tunes the pretrained π0 to new tasks; dramatic reduction from previous 1000+ hours needed
- Physical Intelligence raised $400M Series B in 2024 at $2B valuation; pursuing the "foundation model for physical AI" market

:::tip Hands-on lab

Implement a simplified flow matching policy: define a 2D toy action space, implement the conditional flow matching loss, train on 100 demonstration trajectories, and visualize how the learned vector field transforms random noise into action samples.

:::

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about π0 architecture?</summary>

PaliGemma 3B vision-language backbone + flow matching action expert; total ~3B parameters; runs at 50 Hz on robot

</details>

<details>
<summary>Q2. What do you know about Flow matching (vs diffusion)?</summary>

learns a velocity field that transforms noise → action; faster sampling (1-3 function evaluations vs 50+ for DDPM), smoother trajectories

</details>

<details>
<summary>Q3. What do you know about Training data?</summary>

10,000+ hours of teleoperation data across 7 robot platforms; tasks include laundry folding, box assembly, table clearing, egg packaging

</details>

<details>
<summary>Q4. What do you know about Whole-body result?</summary>

π0 fine-tuned on 1-hour of data achieves 70%+ success on laundry folding (the hardest manipulation benchmark); previous SOTA was ~30%

</details>

<details>
<summary>Q5. What do you know about Fine-tuning efficiency?</summary>

1-10 hours of task-specific demonstrations fine-tunes the pretrained π0 to new tasks; dramatic reduction from previous 1000+ hours needed

</details>

## References

- **π0: A Vision-Language-Action Flow Model for General Robot Control** — Black et al., Physical Intelligence (2024). *arXiv 2410.24164*

---

← Previous: **5.1 From Narrow Policies to General-Purpose Robot Brains** · Next: **5.3 OpenVLA — The Open-Source VLA Ecosystem** →

*Part of Module 5: Foundation Models & VLA Architecture.*


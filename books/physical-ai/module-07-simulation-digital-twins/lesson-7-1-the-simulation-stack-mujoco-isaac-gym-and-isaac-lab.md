---
title: "The Simulation Stack: MuJoCo, Isaac Gym, and Isaac Lab"
sidebar_label: "7.1 The Simulation Stack: MuJoCo, Isaac Gym, and Isaac Lab"
sidebar_position: 2
description: "Physics simulation is the training environment for G1's neural policies"
tags: ["Isaac", "MuJoCo", "simulation", "RL"]
---

# The Simulation Stack: MuJoCo, Isaac Gym, and Isaac Lab

**Duration:** 50 min · **Level:** Intermediate · **Module:** 7. Simulation & Digital Twins · **Focus:** `Isaac`, `MuJoCo`, `simulation`, `RL`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- MuJoCo (Multi-Joint dynamics with Contact)
- MuJoCo 3.0 (2023)
- Isaac Gym (2021, now deprecated)
- Isaac Lab (2024)
- Isaac Sim 4.0
:::

## Why this matters

Physics simulation is the training environment for G1's neural policies.

## Overview

Physics simulation is the training environment for G1's neural policies. The choice of simulator affects training speed, sim-to-real fidelity, and toolchain compatibility. The field has converged on two primary platforms: MuJoCo (DeepMind) for precision and Isaac Lab (NVIDIA) for scale.

## Key concepts

:::note Key idea

MuJoCo (Multi-Joint dynamics with Contact): acquired by DeepMind in 2021, made free; gold standard for contact physics accuracy; used in OpenAI Gym, all major RL benchmarks

:::

- MuJoCo 3.0 (2023): 10-100× faster than previous versions with parallel simulation; runs 10,000+ environments on a single workstation GPU
- Isaac Gym (2021, now deprecated): NVIDIA's GPU-parallel RL training environment; 4096+ parallel environments on a single A100; enabled ETH Zurich ANYmal locomotion results
- Isaac Lab (2024): replaces Isaac Gym; built on Isaac Sim 4.0/Omniverse; better asset management, more realistic contact model, ray-traced rendering for visual RL
- Isaac Sim 4.0: full USD (Universal Scene Description) workflow; Omniverse-based; photorealistic rendering for domain randomization of visual appearance
- G1 recommended stack: Isaac Lab for locomotion training (massively parallel RL) + MuJoCo for manipulation fine-tuning (contact precision) + ROS 2 bridge for hardware deployment

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about MuJoCo (Multi-Joint dynamics with Contact)?</summary>

acquired by DeepMind in 2021, made free; gold standard for contact physics accuracy; used in OpenAI Gym, all major RL benchmarks

</details>

<details>
<summary>Q2. What do you know about MuJoCo 3.0 (2023)?</summary>

10-100× faster than previous versions with parallel simulation; runs 10,000+ environments on a single workstation GPU

</details>

<details>
<summary>Q3. What do you know about Isaac Gym (2021, now deprecated)?</summary>

NVIDIA's GPU-parallel RL training environment; 4096+ parallel environments on a single A100; enabled ETH Zurich ANYmal locomotion results

</details>

<details>
<summary>Q4. What do you know about Isaac Lab (2024)?</summary>

replaces Isaac Gym; built on Isaac Sim 4.0/Omniverse; better asset management, more realistic contact model, ray-traced rendering for visual RL

</details>

<details>
<summary>Q5. What do you know about Isaac Sim 4.0?</summary>

full USD (Universal Scene Description) workflow; Omniverse-based; photorealistic rendering for domain randomization of visual appearance

</details>

## References

- **Isaac Lab: A Unified and Modular Reinforcement Learning Framework for Robot Learning** — Mittal et al. (2023). *arXiv 2301.04195*

---

Next: **7.2 Domain Randomization: The Bridge from Sim to Real** →

*Part of Module 7: Simulation & Digital Twins.*


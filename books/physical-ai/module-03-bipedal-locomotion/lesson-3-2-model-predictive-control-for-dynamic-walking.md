---
title: "Model Predictive Control for Dynamic Walking"
sidebar_label: "3.2 Model Predictive Control for Dynamic Walking"
sidebar_position: 3
description: "Model Predictive Control (MPC) replaced ZMP as the dominant approach for dynamic locomotion"
tags: ["MPC", "locomotion", "optimization", "control"]
---

# Model Predictive Control for Dynamic Walking

**Duration:** 65 min · **Level:** Advanced · **Module:** 3. Bipedal Locomotion & Whole-Body Control · **Focus:** `MPC`, `locomotion`, `optimization`, `control`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- MPC formulation
- Contact-implicit MPC
- MIT Cheetah 3 (2018)
- Humanoid MPC
- Centroidal MPC
:::

## Why this matters

Model Predictive Control (MPC) replaced ZMP as the dominant approach for dynamic locomotion.

## Overview

Model Predictive Control (MPC) replaced ZMP as the dominant approach for dynamic locomotion. MPC plans a finite horizon of future states, solves an optimization at every control step, and handles contact transitions explicitly. It enables running, stair climbing, and perturbation recovery that ZMP controllers cannot achieve.

## Key concepts

:::note Key idea

MPC formulation: minimize cost over horizon N while satisfying dynamics constraints, friction cone constraints, and joint limits; solve at 100-1000 Hz

:::

- Contact-implicit MPC: treats contact schedule (which foot contacts the ground when) as an optimization variable — can automatically discover new gaits
- MIT Cheetah 3 (2018): used convex MPC at 500 Hz to achieve stable 3D running at 6 m/s with real-time re-planning on rough terrain
- Humanoid MPC: more complex than quadruped — 40+ joints, whole-body inertial properties, upper-body dynamics couple to leg control
- Centroidal MPC: reduce humanoid to centroidal dynamics (6 DOF center of mass) for planning; send outputs to whole-body QP controller for full joint commands
- Key software: Drake (MIT), CROCODDYL (LAAS), PINOCCHIO (LAAS) — open-source libraries for humanoid MPC with sub-millisecond solve times

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about MPC formulation?</summary>

minimize cost over horizon N while satisfying dynamics constraints, friction cone constraints, and joint limits; solve at 100-1000 Hz

</details>

<details>
<summary>Q2. What do you know about Contact-implicit MPC?</summary>

treats contact schedule (which foot contacts the ground when) as an optimization variable — can automatically discover new gaits

</details>

<details>
<summary>Q3. What do you know about MIT Cheetah 3 (2018)?</summary>

used convex MPC at 500 Hz to achieve stable 3D running at 6 m/s with real-time re-planning on rough terrain

</details>

<details>
<summary>Q4. What do you know about Humanoid MPC?</summary>

more complex than quadruped — 40+ joints, whole-body inertial properties, upper-body dynamics couple to leg control

</details>

<details>
<summary>Q5. What do you know about Centroidal MPC?</summary>

reduce humanoid to centroidal dynamics (6 DOF center of mass) for planning; send outputs to whole-body QP controller for full joint commands

</details>

## References

- **Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control** — Di Carlo et al. (2018). *IROS 2018*
- **Centroidal Dynamics of a Humanoid Robot** — Orin et al. (2013). *Autonomous Robots*

---

← Previous: **3.1 Locomotion Foundations: ZMP, CoM, and Linear Inverted Pendulum** · Next: **3.3 Reinforcement Learning for Locomotion** →

*Part of Module 3: Bipedal Locomotion & Whole-Body Control.*


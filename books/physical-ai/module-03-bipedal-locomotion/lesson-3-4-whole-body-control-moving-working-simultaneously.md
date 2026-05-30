---
title: "Whole-Body Control: Moving & Working Simultaneously"
sidebar_label: "3.4 Whole-Body Control: Moving & Working Simultaneously"
sidebar_position: 5
description: "Whole-Body Control (WBC) is the control framework that allows a humanoid to simultaneously maintain balance, walk, and use its arms for manipulation — treating all joints as a unif"
tags: ["WBC", "loco-manipulation", "control", "QP"]
---

# Whole-Body Control: Moving & Working Simultaneously

**Duration:** 60 min · **Level:** Advanced · **Module:** 3. Bipedal Locomotion & Whole-Body Control · **Focus:** `WBC`, `loco-manipulation`, `control`, `QP`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- WBC formulation
- Task hierarchy example
- Null-space projection
- CMU loco-manipulation work (2023-2024)
- Contact-consistent WBC
:::

## Why this matters

Whole-Body Control (WBC) is the control framework that allows a humanoid to simultaneously maintain balance, walk, and use its arms for manipulation — treating all joints as a unified system.

## Overview

Whole-Body Control (WBC) is the control framework that allows a humanoid to simultaneously maintain balance, walk, and use its arms for manipulation — treating all joints as a unified system. It is the foundation of every useful humanoid task: you cannot fold laundry while standing if your arms and legs are controlled independently.

## Key concepts

:::note Key idea

WBC formulation: hierarchical QP that solves for all joint accelerations simultaneously; high-priority tasks (balance) override low-priority tasks (arm motion) in null space

:::

- Task hierarchy example: (1) stay balanced, (2) maintain desired CoM trajectory, (3) track arm end-effector goal, (4) minimize joint torques — solved as cascaded QP
- Null-space projection: arm movements that would disturb balance are projected into the null space of the balance task — arm can move without affecting CoM
- CMU loco-manipulation work (2023-2024): humanoid carrying objects while navigating; WBC enables stable walking with 10kg payload in extended arms
- Contact-consistent WBC: explicitly models foot contact forces as optimization variables; handles terrain with varying friction gracefully
- Computational cost: full WBC QP for 40-DOF humanoid solves in ~0.5ms on modern CPU — fast enough for 2kHz control loop

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about WBC formulation?</summary>

hierarchical QP that solves for all joint accelerations simultaneously; high-priority tasks (balance) override low-priority tasks (arm motion) in null space

</details>

<details>
<summary>Q2. What do you know about Task hierarchy example?</summary>

(1) stay balanced, (2) maintain desired CoM trajectory, (3) track arm end-effector goal, (4) minimize joint torques — solved as cascaded QP

</details>

<details>
<summary>Q3. What do you know about Null-space projection?</summary>

arm movements that would disturb balance are projected into the null space of the balance task — arm can move without affecting CoM

</details>

<details>
<summary>Q4. What do you know about CMU loco-manipulation work (2023-2024)?</summary>

humanoid carrying objects while navigating; WBC enables stable walking with 10kg payload in extended arms

</details>

<details>
<summary>Q5. What do you know about Contact-consistent WBC?</summary>

explicitly models foot contact forces as optimization variables; handles terrain with varying friction gracefully

</details>

## References

- **Whole-Body Control of a Mobile Manipulator Using Hierarchical Tasks** — Sentis and Khatib (2005). *ICRA 2005*
- **LOCO-MANI: Simultaneous Locomotion and Manipulation** — Sleiman et al. (2023). *ICRA 2023*

---

← Previous: **3.3 Reinforcement Learning for Locomotion** · Next: **3.5 Terrain Adaptation & Push Recovery** →

*Part of Module 3: Bipedal Locomotion & Whole-Body Control.*


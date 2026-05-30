---
title: "Reinforcement Learning for Locomotion"
sidebar_label: "3.3 Reinforcement Learning for Locomotion"
sidebar_position: 4
description: "Since 2019, RL-based locomotion policies have surpassed MPC controllers on almost every metric: robustness, terrain adaptation, energy efficiency, and max speed"
tags: ["RL", "locomotion", "simulation", "training"]
---

# Reinforcement Learning for Locomotion

**Duration:** 70 min · **Level:** Advanced · **Module:** 3. Bipedal Locomotion & Whole-Body Control · **Focus:** `RL`, `locomotion`, `simulation`, `training`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- ETH Zurich ANYmal (2022)
- Key insight
- Curriculum learning
- Unitree H1 world record (2024)
- Reward function design

You will then consolidate these ideas in the hands-on lab below.
:::

## Why this matters

Since 2019, RL-based locomotion policies have surpassed MPC controllers on almost every metric: robustness, terrain adaptation, energy efficiency, and max speed.

## Overview

Since 2019, RL-based locomotion policies have surpassed MPC controllers on almost every metric: robustness, terrain adaptation, energy efficiency, and max speed. ETH Zurich's ANYmal results, Berkeley's learning-to-walk research, and CMU's humanoid locomotion work define the current state of the art.

## Key concepts

:::note Key idea

ETH Zurich ANYmal (2022): RL policy trained entirely in simulation, deployed zero-shot to real robot; traverses rubble, mud, stairs — outperforms MPC on all metrics

:::

- Key insight: domain randomization during training (randomize mass, friction, terrain) creates policies robust to sim-to-real gap
- Curriculum learning: start with flat terrain, gradually increase difficulty; avoids policy collapse on hard terrain early in training
- Unitree H1 world record (2024): 3.3 m/s walking speed achieved with RL policy trained in Isaac Gym; no human-designed gait pattern
- Reward function design: forward velocity + alive bonus − energy consumption − joint torque limits; shaping matters enormously for behavior quality
- Humanoid RL challenge: 40+ action dimensions creates exploration problem; initialization with reference motion capture data dramatically accelerates learning

:::tip Hands-on lab

In Isaac Lab (Isaac Sim), set up a basic locomotion RL training loop for a 12-DOF biped: define observation space (joint angles, velocities, base orientation, command), action space (target joint angles), and reward function (forward velocity − energy). Run 1000 training iterations and analyze the learning curve.

:::

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about ETH Zurich ANYmal (2022)?</summary>

RL policy trained entirely in simulation, deployed zero-shot to real robot; traverses rubble, mud, stairs — outperforms MPC on all metrics

</details>

<details>
<summary>Q2. What do you know about Key insight?</summary>

domain randomization during training (randomize mass, friction, terrain) creates policies robust to sim-to-real gap

</details>

<details>
<summary>Q3. What do you know about Curriculum learning?</summary>

start with flat terrain, gradually increase difficulty; avoids policy collapse on hard terrain early in training

</details>

<details>
<summary>Q4. What do you know about Unitree H1 world record (2024)?</summary>

3.3 m/s walking speed achieved with RL policy trained in Isaac Gym; no human-designed gait pattern

</details>

<details>
<summary>Q5. What do you know about Reward function design?</summary>

forward velocity + alive bonus − energy consumption − joint torque limits; shaping matters enormously for behavior quality

</details>

## References

- **Learning to Walk in Minutes Using Massively Parallel Deep RL** — Rudin et al. (2022). *CoRL 2022*
- **Expressive Whole-Body Control for Humanoid Robots** — Cheng et al. (2024). *arXiv 2402.16796*
- **Humanoid Locomotion as Next Token Prediction** — Radosavovic et al. (2024). *arXiv 2402.18844*

---

← Previous: **3.2 Model Predictive Control for Dynamic Walking** · Next: **3.4 Whole-Body Control: Moving & Working Simultaneously** →

*Part of Module 3: Bipedal Locomotion & Whole-Body Control.*


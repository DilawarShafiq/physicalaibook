---
title: "Domain Randomization: The Bridge from Sim to Real"
sidebar_label: "7.2 Domain Randomization: The Bridge from Sim to Real"
sidebar_position: 3
description: "Every sim-to-real policy fails due to the \"reality gap\" — differences between simulation and the real world"
tags: ["domain-randomization", "sim-to-real", "training", "transfer"]
---

# Domain Randomization: The Bridge from Sim to Real

**Duration:** 55 min · **Level:** Intermediate · **Module:** 7. Simulation & Digital Twins · **Focus:** `domain-randomization`, `sim-to-real`, `training`, `transfer`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- OpenAI Dactyl (2019)
- What to randomize
- Curriculum + randomization
- Adaptive domain randomization (ADR)
- Visual domain randomization

You will then consolidate these ideas in the hands-on lab below.
:::

## Why this matters

Every sim-to-real policy fails due to the "reality gap" — differences between simulation and the real world.

## Overview

Every sim-to-real policy fails due to the "reality gap" — differences between simulation and the real world. Domain randomization, pioneered by OpenAI and ETH Zurich, systematically randomizes simulation parameters during training to produce policies robust to the distribution of reality.

## Key concepts

:::note Key idea

OpenAI Dactyl (2019): trained dexterous in-hand cube manipulation using domain randomization across 100+ parameters; zero-shot transfer to real Shadow Hand — defining result

:::

- What to randomize: mass (±20%), friction (uniform 0.1-1.5), damping, motor strength, sensor noise, camera exposure, object color/texture, terrain height maps
- Curriculum + randomization: start with narrow randomization range, expand as policy succeeds; prevents training collapse on hard distributions early in learning
- Adaptive domain randomization (ADR): automatically adjust randomization range based on policy success rate; parameters that cause failure get narrower range, those that don't get wider
- Visual domain randomization: randomize textures, lighting, camera parameters; essential for vision-based policies; Isaac Sim 4.0 provides photorealistic randomization via ray tracing
- Residual physics simulation: add a learned "residual" model that compensates for systematic sim-to-real errors; particularly effective for contact-rich manipulation

:::tip Hands-on lab

In Isaac Lab, set up G1 locomotion training with domain randomization: define at least 8 randomized parameters, implement curriculum that extends terrain difficulty as success rate exceeds 80%, and compare zero-shot transfer performance against a policy trained without randomization.

:::

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about OpenAI Dactyl (2019)?</summary>

trained dexterous in-hand cube manipulation using domain randomization across 100+ parameters; zero-shot transfer to real Shadow Hand — defining result

</details>

<details>
<summary>Q2. What do you know about What to randomize?</summary>

mass (±20%), friction (uniform 0.1-1.5), damping, motor strength, sensor noise, camera exposure, object color/texture, terrain height maps

</details>

<details>
<summary>Q3. What do you know about Curriculum + randomization?</summary>

start with narrow randomization range, expand as policy succeeds; prevents training collapse on hard distributions early in learning

</details>

<details>
<summary>Q4. What do you know about Adaptive domain randomization (ADR)?</summary>

automatically adjust randomization range based on policy success rate; parameters that cause failure get narrower range, those that don't get wider

</details>

<details>
<summary>Q5. What do you know about Visual domain randomization?</summary>

randomize textures, lighting, camera parameters; essential for vision-based policies; Isaac Sim 4.0 provides photorealistic randomization via ray tracing

</details>

## References

- **Solving Rubik's Cube with a Robot Hand** — OpenAI et al. (2019). *arXiv 1910.07113*
- **Sim-to-Real Transfer of Robotic Control with Dynamics Randomization** — Peng et al. (2018). *ICRA 2018*

---

← Previous: **7.1 The Simulation Stack: MuJoCo, Isaac Gym, and Isaac Lab** · Next: **7.3 Building a Physics-Accurate Digital Twin of G1** →

*Part of Module 7: Simulation & Digital Twins.*


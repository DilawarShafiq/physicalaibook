---
title: "Locomotion Foundations: ZMP, CoM, and Linear Inverted Pendulum"
sidebar_label: "3.1 Locomotion Foundations: ZMP, CoM, and Linear Inverted Pendulum"
sidebar_position: 2
description: "Zero Moment Point (ZMP) theory, developed by Vukobratović in 1969, was the theoretical foundation of humanoid locomotion for 30 years"
tags: ["ZMP", "locomotion", "balance", "theory"]
---

# Locomotion Foundations: ZMP, CoM, and Linear Inverted Pendulum

**Duration:** 60 min · **Level:** Advanced · **Module:** 3. Bipedal Locomotion & Whole-Body Control · **Focus:** `ZMP`, `locomotion`, `balance`, `theory`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Zero Moment Point
- Linear Inverted Pendulum Model (LIPM)
- ZMP limitation
- Center of Mass (CoM) trajectory planning
- ASIMO (Honda) used ZMP control
:::

## Overview

Zero Moment Point (ZMP) theory, developed by Vukobratović in 1969, was the theoretical foundation of humanoid locomotion for 30 years. ASIMO, HRP-2, and early ATLAS all used ZMP-based controllers. While modern robots have moved beyond it, understanding ZMP is prerequisite to understanding why we moved on.

## Key concepts

:::note Key idea

Zero Moment Point: the point on the ground where the net moment of inertial and gravitational forces has no component along horizontal axes; ZMP inside support polygon → stable

:::

- Linear Inverted Pendulum Model (LIPM): approximates the robot as a point mass on a massless leg; enables real-time trajectory planning with convex optimization
- ZMP limitation: requires slow, flat-footed gait to maintain ZMP inside support polygon; prevents dynamic maneuvers like running or jumping
- Center of Mass (CoM) trajectory planning: plan CoM trajectory such that ZMP constraint is satisfied; solved as QP (quadratic program) in real-time
- ASIMO (Honda) used ZMP control: could walk at 2.7 km/h on flat surfaces, completely failed on any unexpected perturbation or terrain variation
- Capture Point theory (Pratt 2006): extends ZMP to handle single-step recovery; still used as fallback in modern robots when RL policy fails

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>Zero Moment Point</summary>

the point on the ground where the net moment of inertial and gravitational forces has no component along horizontal axes; ZMP inside support polygon → stable

</details>

<details>
<summary>Linear Inverted Pendulum Model (LIPM)</summary>

approximates the robot as a point mass on a massless leg; enables real-time trajectory planning with convex optimization

</details>

<details>
<summary>ZMP limitation</summary>

requires slow, flat-footed gait to maintain ZMP inside support polygon; prevents dynamic maneuvers like running or jumping

</details>

<details>
<summary>Center of Mass (CoM) trajectory planning</summary>

plan CoM trajectory such that ZMP constraint is satisfied; solved as QP (quadratic program) in real-time

</details>

<details>
<summary>ASIMO (Honda) used ZMP control</summary>

could walk at 2.7 km/h on flat surfaces, completely failed on any unexpected perturbation or terrain variation

</details>

## References

- **Biped Stability Conditions with Ankle, Hip and Foot Rotation Indicators** — Vukobratović and Borovac (2004). *International Journal of Humanoid Robotics*
- **Capture Point: A Step toward Humanoid Push Recovery** — Pratt et al. (2006). *Humanoids 2006*

---

Next: **3.2 Model Predictive Control for Dynamic Walking** →

*Part of Module 3: Bipedal Locomotion & Whole-Body Control.*


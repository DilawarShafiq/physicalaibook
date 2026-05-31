---
title: "Hand Design: DOF, Actuation, and the 22-DOF Target"
sidebar_label: "6.1 Hand Design: DOF, Actuation, and the 22-DOF Target"
sidebar_position: 2
description: "The human hand has 27 bones, 29 joints, and over 30 muscles — producing 21 controllable DOF and extraordinary dexterity"
tags: ["hands", "dexterity", "DOF", "design"]
---

# Hand Design: DOF, Actuation, and the 22-DOF Target

**Duration:** 55 min · **Level:** Advanced · **Module:** 6. Dexterous Manipulation · **Focus:** `hands`, `dexterity`, `DOF`, `design`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Human hand DOF breakdown
- Tendon-driven hands (Shadow, Dexterous Hand)
- Direct-drive hands (LEAP Hand)
- LEAP Hand (Carnegie Mellon 2023)
- Inspire Hands (used in Unitree G1 upgrade kit)
:::

## Overview

The human hand has 27 bones, 29 joints, and over 30 muscles — producing 21 controllable DOF and extraordinary dexterity. Modern robot hands target 16-22 DOF as a practical compromise between capability and cost/weight. Figure 02 and Tesla Optimus Gen 2 both chose 22 DOF as the G1 target.

## Key concepts

:::note Key idea

Human hand DOF breakdown: 4 fingers × 4 DOF (MCP flexion/extension, MCP ab/adduction, PIP, DIP) + thumb × 5 DOF = 21 DOF total

:::

- Tendon-driven hands (Shadow, Dexterous Hand): motors proximal to hand, cables run through fingers; lightweight fingers but complex routing; Shadow Hand has 20 DOF
- Direct-drive hands (LEAP Hand): small actuators at each joint; heavier fingers but simpler control and more transparent force feedback; LEAP Hand: 16 DOF, $4k
- LEAP Hand (Carnegie Mellon 2023): open-source, 3D-printable, affordable; used extensively in research; matches human-scale manipulation in most tasks
- Inspire Hands (used in Unitree G1 upgrade kit): 12 DOF, good dexterity, ~$8k/pair; commercially available with ROS support
- G1 hand specification: 22 DOF minimum for healthcare; tendon-driven preferred for finger weight reduction; custom tactile sensing on all 10 fingertips required

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>Human hand DOF breakdown</summary>

4 fingers × 4 DOF (MCP flexion/extension, MCP ab/adduction, PIP, DIP) + thumb × 5 DOF = 21 DOF total

</details>

<details>
<summary>Tendon-driven hands (Shadow, Dexterous Hand)</summary>

motors proximal to hand, cables run through fingers; lightweight fingers but complex routing; Shadow Hand has 20 DOF

</details>

<details>
<summary>Direct-drive hands (LEAP Hand)</summary>

small actuators at each joint; heavier fingers but simpler control and more transparent force feedback; LEAP Hand: 16 DOF, $4k

</details>

<details>
<summary>LEAP Hand (Carnegie Mellon 2023)</summary>

open-source, 3D-printable, affordable; used extensively in research; matches human-scale manipulation in most tasks

</details>

<details>
<summary>Inspire Hands (used in Unitree G1 upgrade kit)</summary>

12 DOF, good dexterity, ~$8k/pair; commercially available with ROS support

</details>

## References

- **LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning** — Shaw et al. (2023). *RSS 2023*

---

Next: **6.2 Tactile Sensing: GelSight, DIGIT, and BioTac** →

*Part of Module 6: Dexterous Manipulation.*


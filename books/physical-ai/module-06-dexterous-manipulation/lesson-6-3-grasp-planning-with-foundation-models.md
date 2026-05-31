---
title: "Grasp Planning with Foundation Models"
sidebar_label: "6.3 Grasp Planning with Foundation Models"
sidebar_position: 4
description: "Grasp planning — deciding how to position the hand and which fingers contact an object to achieve a stable, task-appropriate grip — has been transformed by large-scale learning"
tags: ["grasping", "planning", "learned", "bimanual"]
---

# Grasp Planning with Foundation Models

**Duration:** 55 min · **Level:** Advanced · **Module:** 6. Dexterous Manipulation · **Focus:** `grasping`, `planning`, `learned`, `bimanual`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- GraspNet (2020)
- AnyGrasp (2022)
- Force closure grasps
- Task-oriented grasps
- Bimanual grasps
:::

## Overview

Grasp planning — deciding how to position the hand and which fingers contact an object to achieve a stable, task-appropriate grip — has been transformed by large-scale learning. Modern approaches use category-agnostic grasp prediction networks combined with open-vocabulary detection to grasp novel objects without task-specific training.

## Key concepts

:::note Key idea

GraspNet (2020): trained on 97,280 RGB-D images with 1.2B grasp annotations; predicts grasp poses for novel objects in cluttered scenes; works zero-shot on new object categories

:::

- AnyGrasp (2022): extends GraspNet with language-conditioned grasp selection; "grasp the top of the bottle" vs "grasp the handle" produce different hand configurations
- Force closure grasps: analytical criterion — does the set of contact forces span a space that can resist arbitrary external wrenches? Modern learned grasps approximate this
- Task-oriented grasps: "use" grasps vs "move" grasps differ; pick up a hammer by handle to use it, or anywhere to move it; VLA models learn task-appropriate grasp selection
- Bimanual grasps: many healthcare objects (pill trays, trays of supplies) require two hands; bimanual grasp planning coordinates approach trajectories and contact timing
- Regrasping: if initial grasp is suboptimal, robot can pass object hand-to-hand or use environment surface to improve grip; underexplored but critical for real-world use

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>GraspNet (2020)</summary>

trained on 97,280 RGB-D images with 1.2B grasp annotations; predicts grasp poses for novel objects in cluttered scenes; works zero-shot on new object categories

</details>

<details>
<summary>AnyGrasp (2022)</summary>

extends GraspNet with language-conditioned grasp selection; "grasp the top of the bottle" vs "grasp the handle" produce different hand configurations

</details>

<details>
<summary>Force closure grasps</summary>

analytical criterion — does the set of contact forces span a space that can resist arbitrary external wrenches? Modern learned grasps approximate this

</details>

<details>
<summary>Task-oriented grasps</summary>

"use" grasps vs "move" grasps differ; pick up a hammer by handle to use it, or anywhere to move it; VLA models learn task-appropriate grasp selection

</details>

<details>
<summary>Bimanual grasps</summary>

many healthcare objects (pill trays, trays of supplies) require two hands; bimanual grasp planning coordinates approach trajectories and contact timing

</details>

---

← Previous: **6.2 Tactile Sensing: GelSight, DIGIT, and BioTac**

*Part of Module 6: Dexterous Manipulation.*


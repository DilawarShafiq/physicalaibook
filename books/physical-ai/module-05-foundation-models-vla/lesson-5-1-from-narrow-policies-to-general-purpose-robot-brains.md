---
title: "From Narrow Policies to General-Purpose Robot Brains"
sidebar_label: "5.1 From Narrow Policies to General-Purpose Robot Brains"
sidebar_position: 2
description: "Before 2022, robotic manipulation required a separate hand-engineered controller for each task"
tags: ["VLA", "foundation-models", "generalization", "RT-2"]
---

# From Narrow Policies to General-Purpose Robot Brains

**Duration:** 50 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Focus:** `VLA`, `foundation-models`, `generalization`, `RT-2`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- RT-1 (Google, 2022)
- RT-2 (Google DeepMind, 2023)
- RT-2 example
- Scale law for robotics
- Open X-Embodiment (Google + 33 institutions, 2023)
:::

## Overview

Before 2022, robotic manipulation required a separate hand-engineered controller for each task. You could not transfer a grasp controller to a pouring controller. The insight from GPT-3 applied to robotics: if you scale data and model size enough, a single neural network can learn to do everything.

## Key concepts

:::note Key idea

RT-1 (Google, 2022): first demonstration that a single transformer policy trained on 130,000 robot demonstrations could generalize to new tasks and objects

:::

- RT-2 (Google DeepMind, 2023): co-trained on internet-scale vision-language data AND robot demonstrations; emergent capability: novel semantic reasoning in manipulation
- RT-2 example: "place the extinct animal in front of the green object" — robot correctly identifies dinosaur toy, places it appropriately — zero-shot from language only
- Scale law for robotics: RT-2 used 55B parameter PaLM-E backbone; larger models generalize better but need hardware to run; a key engineering challenge
- Open X-Embodiment (Google + 33 institutions, 2023): pooled 22 different robot platforms, 527 skills, 160,000 demonstrations; trained single policy that works across platforms
- Key shift: data collection via teleoperation + internet pre-training → general policies; the bottleneck is now data quality and quantity, not algorithm design

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>RT-1 (Google, 2022)</summary>

first demonstration that a single transformer policy trained on 130,000 robot demonstrations could generalize to new tasks and objects

</details>

<details>
<summary>RT-2 (Google DeepMind, 2023)</summary>

co-trained on internet-scale vision-language data AND robot demonstrations; emergent capability: novel semantic reasoning in manipulation

</details>

<details>
<summary>RT-2 example</summary>

"place the extinct animal in front of the green object" — robot correctly identifies dinosaur toy, places it appropriately — zero-shot from language only

</details>

<details>
<summary>Scale law for robotics</summary>

RT-2 used 55B parameter PaLM-E backbone; larger models generalize better but need hardware to run; a key engineering challenge

</details>

<details>
<summary>Open X-Embodiment (Google + 33 institutions, 2023)</summary>

pooled 22 different robot platforms, 527 skills, 160,000 demonstrations; trained single policy that works across platforms

</details>

## References

- **RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control** — Brohan et al. (2023). *CoRL 2023*
- **Open X-Embodiment: Robotic Learning Datasets and RT-X Models** — Open X-Embodiment Collaboration (2023). *arXiv 2310.08864*

---

Next: **5.2 π0 — Diffusion-Based Whole-Body Control** →

*Part of Module 5: Foundation Models & VLA Architecture.*


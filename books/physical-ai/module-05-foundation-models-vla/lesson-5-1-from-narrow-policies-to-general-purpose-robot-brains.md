---
title: "From Narrow Policies to General-Purpose Robot Brains"
sidebar_label: "5.1 From Narrow Policies to General-Purpose Robot Brains"
sidebar_position: 2
description: "Before 2022, robotic manipulation required a separate hand-engineered controller for each task"
tags: ["VLA", "foundation-models", "generalization", "RT-2"]
---

# From Narrow Policies to General-Purpose Robot Brains

**Duration:** 50 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Tags:** `VLA`, `foundation-models`, `generalization`, `RT-2`

## Overview

Before 2022, robotic manipulation required a separate hand-engineered controller for each task. You could not transfer a grasp controller to a pouring controller. The insight from GPT-3 applied to robotics: if you scale data and model size enough, a single neural network can learn to do everything.

## Key Insights

- RT-1 (Google, 2022): first demonstration that a single transformer policy trained on 130,000 robot demonstrations could generalize to new tasks and objects
- RT-2 (Google DeepMind, 2023): co-trained on internet-scale vision-language data AND robot demonstrations; emergent capability: novel semantic reasoning in manipulation
- RT-2 example: "place the extinct animal in front of the green object" — robot correctly identifies dinosaur toy, places it appropriately — zero-shot from language only
- Scale law for robotics: RT-2 used 55B parameter PaLM-E backbone; larger models generalize better but need hardware to run; a key engineering challenge
- Open X-Embodiment (Google + 33 institutions, 2023): pooled 22 different robot platforms, 527 skills, 160,000 demonstrations; trained single policy that works across platforms
- Key shift: data collection via teleoperation + internet pre-training → general policies; the bottleneck is now data quality and quantity, not algorithm design

## References

- **RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control** — Brohan et al. (2023). *CoRL 2023*
- **Open X-Embodiment: Robotic Learning Datasets and RT-X Models** — Open X-Embodiment Collaboration (2023). *arXiv 2310.08864*


---
title: "Hand Design: DOF, Actuation, and the 22-DOF Target"
sidebar_label: "6.1 Hand Design: DOF, Actuation, and the 22-DOF Target"
sidebar_position: 2
description: "The human hand has 27 bones, 29 joints, and over 30 muscles — producing 21 controllable DOF and extraordinary dexterity"
tags: ["hands", "dexterity", "DOF", "design"]
---

# Hand Design: DOF, Actuation, and the 22-DOF Target

**Duration:** 55 min · **Level:** Advanced · **Module:** 6. Dexterous Manipulation · **Tags:** `hands`, `dexterity`, `DOF`, `design`

## Overview

The human hand has 27 bones, 29 joints, and over 30 muscles — producing 21 controllable DOF and extraordinary dexterity. Modern robot hands target 16-22 DOF as a practical compromise between capability and cost/weight. Figure 02 and Tesla Optimus Gen 2 both chose 22 DOF as the G1 target.

## Key Insights

- Human hand DOF breakdown: 4 fingers × 4 DOF (MCP flexion/extension, MCP ab/adduction, PIP, DIP) + thumb × 5 DOF = 21 DOF total
- Tendon-driven hands (Shadow, Dexterous Hand): motors proximal to hand, cables run through fingers; lightweight fingers but complex routing; Shadow Hand has 20 DOF
- Direct-drive hands (LEAP Hand): small actuators at each joint; heavier fingers but simpler control and more transparent force feedback; LEAP Hand: 16 DOF, $4k
- LEAP Hand (Carnegie Mellon 2023): open-source, 3D-printable, affordable; used extensively in research; matches human-scale manipulation in most tasks
- Inspire Hands (used in Unitree G1 upgrade kit): 12 DOF, good dexterity, ~$8k/pair; commercially available with ROS support
- G1 hand specification: 22 DOF minimum for healthcare; tendon-driven preferred for finger weight reduction; custom tactile sensing on all 10 fingertips required

## References

- **LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning** — Shaw et al. (2023). *RSS 2023*


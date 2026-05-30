---
title: "G1 Architecture: Resolved Technology Decisions"
sidebar_label: "10.1 G1 Architecture: Resolved Technology Decisions"
sidebar_position: 2
description: "Pulling together all modules, this lesson makes the final architecture calls for Autosapien G1"
tags: ["G1", "architecture", "systems", "decisions"]
---

# G1 Architecture: Resolved Technology Decisions

**Duration:** 60 min · **Level:** Foundational · **Module:** 10. G1 Development Roadmap · **Tags:** `G1`, `architecture`, `systems`, `decisions`

## Overview

Pulling together all modules, this lesson makes the final architecture calls for Autosapien G1. Every decision is grounded in the state-of-the-art reviewed across this curriculum, with explicit rationale and known risks.

## Key Insights

- Actuation: QDD motors (locomotion joints) + SEA (arm joints) hybrid; HEBI Robotics X5-4 series for prototype; custom in-house actuators for production
- Locomotion: RL-based policy trained in Isaac Lab + MPC fallback controller; target 1.5 m/s walking, 0.5 m/s stair climbing, 0.3 m/s backward walking
- Perception: 4× RGB-D cameras (forward stereo + bilateral) + 2× fisheye cameras + 2× 6-axis IMU + 2× 6-DOF wrist F/T sensors + fingertip DIGIT tactile sensors
- AI brain: dual AGX Orin setup (perception + VLA on one; locomotion + safety monitor on second); fine-tuned π0 or OpenVLA as manipulation policy backbone
- Hands: custom 22-DOF tendon-driven design with DIGIT tactile on all 10 fingertips; total hand weight &lt;800g per hand; peak fingertip force 20N
- Power: 4.5 kWh LFP battery; 48V architecture; 8+ hours at moderate activity; hot-swap design in &lt;5 minutes


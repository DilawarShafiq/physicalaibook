---
title: "Terrain Adaptation & Push Recovery"
sidebar_label: "3.5 Terrain Adaptation & Push Recovery"
sidebar_position: 6
description: "Real-world environments are unforgiving: wet floors, loose rugs, uneven tile, staircases, and unexpected perturbations from humans"
tags: ["terrain", "push-recovery", "footstep-planning", "robustness"]
---

# Terrain Adaptation & Push Recovery

**Duration:** 50 min · **Level:** Advanced · **Module:** 3. Bipedal Locomotion & Whole-Body Control · **Tags:** `terrain`, `push-recovery`, `footstep-planning`, `robustness`

## Overview

Real-world environments are unforgiving: wet floors, loose rugs, uneven tile, staircases, and unexpected perturbations from humans. A robot that works in a hospital must handle all of these without falling. This lesson covers terrain perception, adaptive footstep planning, and recovery controllers.

## Key Insights

- Terrain estimation: fuse depth camera + IMU + proprioceptive foot contact to build real-time local elevation map at 100Hz update rate
- Footstep planning: project safe landing zones from elevation map; use A* or MPC to plan footstep sequence that avoids obstacles and respects terrain slope limits
- Push recovery strategies: ankle strategy (small perturbation → ankle torque), hip strategy (larger → hip flexion), step strategy (large → take a step to new support polygon)
- MIT Terrain-Adaptive Atlas (2023): trained RL policy on diverse terrain in simulation; zero-shot deployment across grass, gravel, stairs, and slippery surfaces
- Fall detection and protective response: detect impending fall at &gt;500ms before impact; robot enters protective posture — protect head, distribute impact across large body areas
- G1 specification: must withstand lateral push of 150N at hip height without falling; must recover from unexpected 20° slope at 1 m/s walking speed


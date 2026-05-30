---
title: "Unitree G1 & the Open Research Ecosystem"
sidebar_label: "1.5 Unitree G1 & the Open Research Ecosystem"
sidebar_position: 6
description: "Unitree Robotics released the G1 humanoid in May 2024 at $16,000 — an order of magnitude cheaper than competitors"
tags: ["unitree", "open-source", "research", "accessibility"]
---

# Unitree G1 & the Open Research Ecosystem

**Duration:** 35 min · **Level:** Foundational · **Module:** 1. The Humanoid Landscape · **Tags:** `unitree`, `open-source`, `research`, `accessibility`

## Overview

Unitree Robotics released the G1 humanoid in May 2024 at $16,000 — an order of magnitude cheaper than competitors. This price point has democratized humanoid research globally, spawning hundreds of academic labs working on locomotion, manipulation, and learning algorithms that will define the field by 2030.

## Key Insights

- Unitree G1: 1.32m, 35kg, 43 DOF, walking speed up to 2 m/s, $16,000 USD — ships with SDK and ROS 2 support
- H1 (predecessor): 1.8m, 47kg, achieved world-record 3.3 m/s walking speed in 2024 using RL-based locomotion policy
- Unitree's strategy: sell hardware to researchers at cost, capture data and algorithm IP; similar to NVIDIA's developer ecosystem model
- Over 200 research papers published using Unitree H1/G1 as the platform within 12 months of G1 release
- G1 uses off-the-shelf brushless DC motors with integrated electronics — easy to repair, widely understood
- Key limitation: no dexterous hands by default; 3-finger gripper is standard; aftermarket hand kits available from LEAP Hand, etc.

:::tip Lab

Given G1's published specs, calculate: (a) maximum joint torques needed for a 1-arm curl of 5kg payload, (b) estimated battery consumption at 1.5 m/s walking speed, (c) minimum compute needed for real-time VLA inference at 10 Hz.

:::

## References

- **Expressive Whole-Body Control for Humanoid Robots** — Cheng et al. (2024). *arXiv 2402.16796*


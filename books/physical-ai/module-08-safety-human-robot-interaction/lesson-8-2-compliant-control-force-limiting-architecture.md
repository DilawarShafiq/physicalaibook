---
title: "Compliant Control & Force-Limiting Architecture"
sidebar_label: "8.2 Compliant Control & Force-Limiting Architecture"
sidebar_position: 3
description: "Meeting ISO/TS 15066 force limits requires that G1 can sense and limit contact forces in real-time"
tags: ["compliance", "force-control", "impedance", "safety"]
---

# Compliant Control & Force-Limiting Architecture

**Duration:** 55 min · **Level:** Intermediate · **Module:** 8. Safety & Human-Robot Interaction · **Tags:** `compliance`, `force-control`, `impedance`, `safety`

## Overview

Meeting ISO/TS 15066 force limits requires that G1 can sense and limit contact forces in real-time. This lesson covers the hardware and software architecture for force-limited operation — from actuator torque sensing to predictive collision detection.

## Key Insights

- Joint torque sensing: strain gauges or current sensing at each motor enable contact force estimation; accuracy ±2-5N with proprioceptive sensing only
- Wrist F/T sensors: 6-DOF force/torque measurement at each wrist; provides accurate end-effector force for manipulation safety; ATI Mini45 or custom designs
- Impedance control: regulate robot endpoint stiffness/damping rather than position; "soft" impedance (low stiffness) makes contact gentle; used in Franka Emika Panda cobots
- Collision detection without external sensors: use momentum observer (estimated vs actual joint velocities) to detect unexpected contact; detects collisions in &lt;10ms
- Predictive safety: trajectory planning checks all future configurations against workspace model; stops motion before entering known occupied region
- ISO 10218-2 risk assessment: document all hazard scenarios, estimate probability and severity, implement mitigations until residual risk is acceptable


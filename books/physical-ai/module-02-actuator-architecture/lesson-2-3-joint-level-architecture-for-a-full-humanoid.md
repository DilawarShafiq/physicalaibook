---
title: "Joint-Level Architecture for a Full Humanoid"
sidebar_label: "2.3 Joint-Level Architecture for a Full Humanoid"
sidebar_position: 4
description: "A complete humanoid has 40-50 joints, each with different torque, speed, and compliance requirements"
tags: ["joint-design", "torque", "kinematics", "systems"]
---

# Joint-Level Architecture for a Full Humanoid

**Duration:** 55 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Tags:** `joint-design`, `torque`, `kinematics`, `systems`

## Overview

A complete humanoid has 40-50 joints, each with different torque, speed, and compliance requirements. Mapping the right actuator topology to each joint class — ankles, knees, hips, spine, shoulders, elbows, wrists, fingers — is one of the most consequential design decisions for G1.

## Key Insights

- Ankle joint: highest impact loads (3-5× bodyweight at heel strike); needs stiff, high-torque actuator; QDD with peak 200+ Nm recommended
- Knee joint: highest continuous torque; often uses the largest motor in the robot; 150-250 Nm continuous, 400+ Nm peak
- Hip joints (3 DOF each): abduction/adduction + flexion/extension + rotation; combined torque budget ~300 Nm; ball joint or 3-actuator universal joint
- Shoulder (3-4 DOF): human shoulder is the most complex joint; redundant DOF for reach; target 80 Nm to support 5kg arm raise
- Finger joints: micro-actuators or tendon-driven with proximal motors; 1-5 Nm per finger joint; cable routing is the key engineering challenge
- Spine: often underspecified; at least 2-3 DOF (pitch + roll + yaw); critical for whole-body motion; often uses low-ratio harmonic drives

:::tip Lab

Spec out the complete actuator selection for G1's 40-joint system: fill a table with joint name, DOF, peak torque requirement (estimated from dynamics), continuous torque, speed requirement, recommended actuator type, and approximate mass budget per joint.

:::


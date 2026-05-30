---
title: "Joint-Level Architecture for a Full Humanoid"
sidebar_label: "2.3 Joint-Level Architecture for a Full Humanoid"
sidebar_position: 4
description: "A complete humanoid has 40-50 joints, each with different torque, speed, and compliance requirements"
tags: ["joint-design", "torque", "kinematics", "systems"]
---

# Joint-Level Architecture for a Full Humanoid

**Duration:** 55 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Focus:** `joint-design`, `torque`, `kinematics`, `systems`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Ankle joint
- Knee joint
- Hip joints (3 DOF each)
- Shoulder (3-4 DOF)
- Finger joints

You will then consolidate these ideas in the hands-on lab below.
:::

## Why this matters

A complete humanoid has 40-50 joints, each with different torque, speed, and compliance requirements.

## Overview

A complete humanoid has 40-50 joints, each with different torque, speed, and compliance requirements. Mapping the right actuator topology to each joint class — ankles, knees, hips, spine, shoulders, elbows, wrists, fingers — is one of the most consequential design decisions for G1.

## Key concepts

:::note Key idea

Ankle joint: highest impact loads (3-5× bodyweight at heel strike); needs stiff, high-torque actuator; QDD with peak 200+ Nm recommended

:::

- Knee joint: highest continuous torque; often uses the largest motor in the robot; 150-250 Nm continuous, 400+ Nm peak
- Hip joints (3 DOF each): abduction/adduction + flexion/extension + rotation; combined torque budget ~300 Nm; ball joint or 3-actuator universal joint
- Shoulder (3-4 DOF): human shoulder is the most complex joint; redundant DOF for reach; target 80 Nm to support 5kg arm raise
- Finger joints: micro-actuators or tendon-driven with proximal motors; 1-5 Nm per finger joint; cable routing is the key engineering challenge
- Spine: often underspecified; at least 2-3 DOF (pitch + roll + yaw); critical for whole-body motion; often uses low-ratio harmonic drives

:::tip Hands-on lab

Spec out the complete actuator selection for G1's 40-joint system: fill a table with joint name, DOF, peak torque requirement (estimated from dynamics), continuous torque, speed requirement, recommended actuator type, and approximate mass budget per joint.

:::

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about Ankle joint?</summary>

highest impact loads (3-5× bodyweight at heel strike); needs stiff, high-torque actuator; QDD with peak 200+ Nm recommended

</details>

<details>
<summary>Q2. What do you know about Knee joint?</summary>

highest continuous torque; often uses the largest motor in the robot; 150-250 Nm continuous, 400+ Nm peak

</details>

<details>
<summary>Q3. What do you know about Hip joints (3 DOF each)?</summary>

abduction/adduction + flexion/extension + rotation; combined torque budget ~300 Nm; ball joint or 3-actuator universal joint

</details>

<details>
<summary>Q4. What do you know about Shoulder (3-4 DOF)?</summary>

human shoulder is the most complex joint; redundant DOF for reach; target 80 Nm to support 5kg arm raise

</details>

<details>
<summary>Q5. What do you know about Finger joints?</summary>

micro-actuators or tendon-driven with proximal motors; 1-5 Nm per finger joint; cable routing is the key engineering challenge

</details>

---

← Previous: **2.2 Series Elastic Actuators — Compliance by Design** · Next: **2.4 Battery Systems & Power Architecture** →

*Part of Module 2: Actuator Architecture.*


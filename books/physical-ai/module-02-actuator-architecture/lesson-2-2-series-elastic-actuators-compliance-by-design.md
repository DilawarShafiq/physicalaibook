---
title: "Series Elastic Actuators — Compliance by Design"
sidebar_label: "2.2 Series Elastic Actuators — Compliance by Design"
sidebar_position: 3
description: "Series Elastic Actuators (SEAs), invented by Gill Pratt and Matt Williamson at MIT in 1995, place a physical spring between the motor output and the robot's joint"
tags: ["SEA", "compliance", "force-control", "safety"]
---

# Series Elastic Actuators — Compliance by Design

**Duration:** 45 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Focus:** `SEA`, `compliance`, `force-control`, `safety`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- SEA principle
- Force control bandwidth of SEAs is limited by spring stiffness
- NASA Valkyrie, MIT HERMES, and Apptronik Apollo all use SEAs
- Apptronik Apollo (2023)
- SEA vs QDD choice by joint
:::

## Why this matters

Series Elastic Actuators (SEAs), invented by Gill Pratt and Matt Williamson at MIT in 1995, place a physical spring between the motor output and the robot's joint.

## Overview

Series Elastic Actuators (SEAs), invented by Gill Pratt and Matt Williamson at MIT in 1995, place a physical spring between the motor output and the robot's joint. This intentional compliance makes force control precise, protects the mechanism from impact, and makes contact inherently gentler. SEAs are the dominant choice for actuators that regularly touch humans.

## Key concepts

:::note Key idea

SEA principle: spring in series between gearbox and joint; force = spring displacement × stiffness; measure displacement → know force precisely

:::

- Force control bandwidth of SEAs is limited by spring stiffness: softer spring = more compliance but slower force response (~50 Hz typical)
- NASA Valkyrie, MIT HERMES, and Apptronik Apollo all use SEAs — chosen specifically for human-safe operation
- Apptronik Apollo (2023): all 28 joints use in-house SEAs rated at 300 Nm peak torque; designed for ISO/TS 15066 collaborative operation near humans
- SEA vs QDD choice by joint: ankles/knees → QDD (need stiffness for locomotion); wrists/shoulders → SEA (need compliance for manipulation near humans)
- Modern trend: variable stiffness actuators (VSA) that can switch between SEA and rigid mode — complex but gives best of both worlds

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about SEA principle?</summary>

spring in series between gearbox and joint; force = spring displacement × stiffness; measure displacement → know force precisely

</details>

<details>
<summary>Q2. What do you know about Force control bandwidth of SEAs is limited by spring stiffness?</summary>

softer spring = more compliance but slower force response (~50 Hz typical)

</details>

<details>
<summary>Q3. What do you know about NASA Valkyrie, MIT HERMES, and Apptronik Apollo all use SEAs?</summary>

chosen specifically for human-safe operation

</details>

<details>
<summary>Q4. What do you know about Apptronik Apollo (2023)?</summary>

all 28 joints use in-house SEAs rated at 300 Nm peak torque; designed for ISO/TS 15066 collaborative operation near humans

</details>

<details>
<summary>Q5. What do you know about SEA vs QDD choice by joint?</summary>

ankles/knees → QDD (need stiffness for locomotion); wrists/shoulders → SEA (need compliance for manipulation near humans)

</details>

## References

- **Series Elastic Actuators** — Pratt and Williamson (1995). *IROS 1995*
- **Actuator Design for the Apollo Humanoid Robot** — Apptronik Team (2023). *ICRA 2023*

---

← Previous: **2.1 The Actuator Revolution: From Hydraulics to QDD Electric** · Next: **2.3 Joint-Level Architecture for a Full Humanoid** →

*Part of Module 2: Actuator Architecture.*


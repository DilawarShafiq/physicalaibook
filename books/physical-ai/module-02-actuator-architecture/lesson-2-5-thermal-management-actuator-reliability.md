---
title: "Thermal Management & Actuator Reliability"
sidebar_label: "2.5 Thermal Management & Actuator Reliability"
sidebar_position: 6
description: "Motors generate heat"
tags: ["thermal", "reliability", "cooling", "design"]
---

# Thermal Management & Actuator Reliability

**Duration:** 40 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Focus:** `thermal`, `reliability`, `cooling`, `design`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Motor thermal model
- Thermal derating
- Cooling approaches
- Figure 02 uses liquid-cooled actuators in high-load joints
- Thermal monitoring
:::

## Why this matters

Motors generate heat.

## Overview

Motors generate heat. At high duty cycles, motor winding temperature is the primary constraint on sustained performance. Without active thermal management, a humanoid operating in a warm hospital or factory will progressively derate its motors — reducing torque, speed, and eventually causing fault shutdowns.

## Key concepts

:::note Key idea

Motor thermal model: winding temperature rises as I²R losses × thermal resistance; 150°C winding limit is typical for Class F insulation

:::

- Thermal derating: most motors must reduce torque by 50% when winding temperature reaches 120°C to prevent insulation damage
- Cooling approaches: passive (aluminum housing + fins), active air (miniature fans in joint housings), liquid (water/glycol loop through hollow shafts)
- Figure 02 uses liquid-cooled actuators in high-load joints — adds ~2kg but allows sustained operation without derating
- Thermal monitoring: thermistors in each motor winding, monitored at 100Hz by joint controller; feeds into real-time torque command limits
- MTBF target: commercial humanoid actuators should target 10,000+ operating hours MTBF; ball bearings are typically the first wear item

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about Motor thermal model?</summary>

winding temperature rises as I²R losses × thermal resistance; 150°C winding limit is typical for Class F insulation

</details>

<details>
<summary>Q2. What do you know about Thermal derating?</summary>

most motors must reduce torque by 50% when winding temperature reaches 120°C to prevent insulation damage

</details>

<details>
<summary>Q3. What do you know about Cooling approaches?</summary>

passive (aluminum housing + fins), active air (miniature fans in joint housings), liquid (water/glycol loop through hollow shafts)

</details>

<details>
<summary>Q4. What do you know about Figure 02 uses liquid-cooled actuators in high-load joints?</summary>

adds ~2kg but allows sustained operation without derating

</details>

<details>
<summary>Q5. What do you know about Thermal monitoring?</summary>

thermistors in each motor winding, monitored at 100Hz by joint controller; feeds into real-time torque command limits

</details>

---

← Previous: **2.4 Battery Systems & Power Architecture**

*Part of Module 2: Actuator Architecture.*


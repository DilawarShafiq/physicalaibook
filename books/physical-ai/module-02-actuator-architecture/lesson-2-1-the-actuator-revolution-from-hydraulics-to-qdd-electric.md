---
title: "The Actuator Revolution: From Hydraulics to QDD Electric"
sidebar_label: "2.1 The Actuator Revolution: From Hydraulics to QDD Electric"
sidebar_position: 2
description: "Boston Dynamics' hydraulic Atlas was an engineering marvel but hydraulics are noisy, leak-prone, and require bulky pumps"
tags: ["actuators", "QDD", "motors", "compliance"]
---

# The Actuator Revolution: From Hydraulics to QDD Electric

**Duration:** 50 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Tags:** `actuators`, `QDD`, `motors`, `compliance`

## Overview

Boston Dynamics' hydraulic Atlas was an engineering marvel but hydraulics are noisy, leak-prone, and require bulky pumps. The shift to quasi-direct drive (QDD) electric motors — pioneered by MIT's Biomimetic Robotics Lab — unlocked a new generation of robots that are quieter, cleaner, and inherently backdrivable.

## Key Insights

- MIT Mini Cheetah (2018) proved QDD motors at robot-scale: 6:1 planetary gear ratio, 12 motors, backdrivable, 14kg total weight
- QDD motors use low gear ratios (3:1 to 9:1) — the motor spins slowly but is directly coupled to the joint; backdriving requires only small forces, making contact inherently safe
- High gear ratio motors (100:1+) are strong but non-backdrivable — a force applied to the output cannot spin the motor; this is dangerous in contact situations
- Torque density comparison: QDD ~10-20 Nm/kg, SEA ~5-15 Nm/kg, hydraulic ~100+ Nm/kg — hydraulic still wins on raw torque but at 10x cost and complexity
- Key players: T-Motor (China) supplies QDD actuators to Unitree, many startups; Moog and Parker supply to industrial robots; BD designs in-house
- Peak vs. continuous torque: QDD motors can briefly produce 3-5x their continuous torque rating for impact absorption and dynamic maneuvers

## References

- **MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot** — Bledt et al. (2018). *IROS 2018*
- **Quasi-Direct Drive for Low-Cost Compliant Robotic Manipulation** — Bhatt et al. (2021). *ICRA 2021*


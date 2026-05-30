---
title: "Thermal Management & Actuator Reliability"
sidebar_label: "2.5 Thermal Management & Actuator Reliability"
sidebar_position: 6
description: "Motors generate heat"
tags: ["thermal", "reliability", "cooling", "design"]
---

# Thermal Management & Actuator Reliability

**Duration:** 40 min · **Level:** Intermediate · **Module:** 2. Actuator Architecture · **Tags:** `thermal`, `reliability`, `cooling`, `design`

## Overview

Motors generate heat. At high duty cycles, motor winding temperature is the primary constraint on sustained performance. Without active thermal management, a humanoid operating in a warm hospital or factory will progressively derate its motors — reducing torque, speed, and eventually causing fault shutdowns.

## Key Insights

- Motor thermal model: winding temperature rises as I²R losses × thermal resistance; 150°C winding limit is typical for Class F insulation
- Thermal derating: most motors must reduce torque by 50% when winding temperature reaches 120°C to prevent insulation damage
- Cooling approaches: passive (aluminum housing + fins), active air (miniature fans in joint housings), liquid (water/glycol loop through hollow shafts)
- Figure 02 uses liquid-cooled actuators in high-load joints — adds ~2kg but allows sustained operation without derating
- Thermal monitoring: thermistors in each motor winding, monitored at 100Hz by joint controller; feeds into real-time torque command limits
- MTBF target: commercial humanoid actuators should target 10,000+ operating hours MTBF; ball bearings are typically the first wear item


---
title: "ISO 10218 & ISO/TS 15066: The Collaborative Robot Safety Standards"
sidebar_label: "8.1 ISO 10218 & ISO/TS 15066: The Collaborative Robot Safety Standards"
sidebar_position: 2
description: "ISO 10218 (Parts 1 & 2) covers safety requirements for industrial robots"
tags: ["safety", "ISO", "standards", "compliance"]
---

# ISO 10218 & ISO/TS 15066: The Collaborative Robot Safety Standards

**Duration:** 50 min · **Level:** Intermediate · **Module:** 8. Safety & Human-Robot Interaction · **Tags:** `safety`, `ISO`, `standards`, `compliance`

## Overview

ISO 10218 (Parts 1 & 2) covers safety requirements for industrial robots. ISO/TS 15066 extends this to "collaborative robots" — robots that operate in direct physical contact with humans. These standards define the force and pressure limits that G1 must not exceed to be safe for healthcare deployment.

## Key Insights

- ISO/TS 15066 body contact limits: transient contact at the thorax must not exceed 65N force and 15 kN/m² pressure; limits vary by body region (head most sensitive)
- Four collaboration modes defined: safety-rated monitored stop, hand guiding, speed/separation monitoring, and power/force limiting — G1 primarily needs mode 4 (P/F limiting)
- Power and force limiting (mode 4): robot stops or reduces speed if sensed contact force exceeds threshold; requires joint torque sensing or external force sensor
- Speed and separation monitoring (mode 3): reduce robot speed as human approaches; stops when human within minimum protective distance calculated from robot stopping time
- CE marking (Europe) and UL certification (North America) for collaborative robots: require third-party risk assessment demonstrating compliance with applicable standards
- Healthcare-specific standards: IEC 62061 (functional safety of machinery), IEC 60601 (medical electrical equipment) — G1 targeting hospital deployment needs both


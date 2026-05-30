---
title: "Tactile Sensing: GelSight, DIGIT, and BioTac"
sidebar_label: "6.2 Tactile Sensing: GelSight, DIGIT, and BioTac"
sidebar_position: 3
description: "Tactile sensing is the missing modality in most robot hands — and it is critical for healthcare applications where gentle contact, detecting slip, and measuring contact forces are "
tags: ["tactile", "sensing", "GelSight", "DIGIT"]
---

# Tactile Sensing: GelSight, DIGIT, and BioTac

**Duration:** 55 min · **Level:** Advanced · **Module:** 6. Dexterous Manipulation · **Tags:** `tactile`, `sensing`, `GelSight`, `DIGIT`

## Overview

Tactile sensing is the missing modality in most robot hands — and it is critical for healthcare applications where gentle contact, detecting slip, and measuring contact forces are essential. GelSight-class sensors use a gel-covered surface illuminated from within to image contact geometry at sub-millimeter resolution.

## Key Insights

- GelSight (MIT, Tao et al. 2012): gel elastomer + internal LEDs + camera; images the deformed gel surface; recovers 3D geometry of contact at 0.1mm resolution
- DIGIT (Meta AI Research, Lambeta et al. 2020): compact GelSight redesigned for fingertip mounting; 20mm × 20mm × 24mm; 60 FPS tactile images; open-source design
- BioTac (SynTouch): fluid-filled silicone fingertip with 24 electrodes; measures distributed pressure, vibration, and temperature; FDA-cleared for medical applications
- Slip detection: tactile images reveal incipient slip (micro-motion before gross slip) 100-200ms before grasp failure; enables proactive grasp adjustment
- Manipulation capability enabled: threading a needle (requires &lt;0.5mm contact resolution), detecting full medication bottle vs empty (mass from grip force),  gentle patient contact (&lt;2N force)
- Learning from tactile data: convolutional neural networks on tactile images predict contact forces, object geometry, and slip onset with &gt;95% accuracy

## References

- **DIGIT: A Novel Design for a Low-Cost Compact High-Resolution Tactile Sensor with Application to In-Hand Manipulation** — Lambeta et al. (2020). *IEEE Robotics and Automation Letters 2020*


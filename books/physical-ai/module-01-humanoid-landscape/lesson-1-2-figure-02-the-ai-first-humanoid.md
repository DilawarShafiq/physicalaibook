---
title: "Figure 02 — The AI-First Humanoid"
sidebar_label: "1.2 Figure 02 — The AI-First Humanoid"
sidebar_position: 3
description: "Figure AI raised $675M at a $2.6B valuation in February 2024, backed by Microsoft, OpenAI, NVIDIA, and Amazon"
tags: ["figure", "VLA", "hardware", "deployment"]
---

# Figure 02 — The AI-First Humanoid

**Duration:** 45 min · **Level:** Foundational · **Module:** 1. The Humanoid Landscape · **Tags:** `figure`, `VLA`, `hardware`, `deployment`

## Overview

Figure AI raised $675M at a $2.6B valuation in February 2024, backed by Microsoft, OpenAI, NVIDIA, and Amazon. Figure 02, announced August 2024, is a ground-up redesign that treats the robot as a compute platform first, mechanical system second. Its OpenAI partnership produces end-to-end neural network policies that go directly from sensor input to motor commands.

## Key Insights

- Figure 02 stands 1.68m, weighs 60kg, and carries a 20kg payload — optimized for warehouse and manufacturing tasks
- Hands feature 16 DOF each with tactile sensing on fingertips; can pick up a USB-C cable and plug it in without external guidance
- Uses a custom NVIDIA Orin-based compute stack with Heliogen multimodal neural network for end-to-end task execution
- OpenAI collaboration produces Vision-Language-Action (VLA) policies: natural language instruction → sensory observation → motor output in a single forward pass
- BMW Spartanburg factory deployment (2024): Figure 02 performing parts transfer tasks at human-comparable cycle times after ~24 hours of in-context learning
- Figure's key architectural bet: scale data, not engineering — collect teleoperation demonstrations at scale and train general-purpose policies

:::tip Lab

Analyze Figure 02's end-to-end architecture: trace the data flow from RGB cameras → tokenized vision features → language encoder → action decoder → motor commands. Identify which components are off-the-shelf and which are custom.

:::

## References

- **Helix: A Vision-Language-Action Model for Generalist Humanoid Control** — Figure AI Research (2024). *Figure AI Technical Report*


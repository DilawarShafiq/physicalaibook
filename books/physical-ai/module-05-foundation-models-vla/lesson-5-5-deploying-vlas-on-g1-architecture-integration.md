---
title: "Deploying VLAs on G1: Architecture & Integration"
sidebar_label: "5.5 Deploying VLAs on G1: Architecture & Integration"
sidebar_position: 6
description: "Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architect"
tags: ["deployment", "inference", "hardware", "architecture"]
---

# Deploying VLAs on G1: Architecture & Integration

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Focus:** `deployment`, `inference`, `hardware`, `architecture`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- NVIDIA AGX Orin
- Two-level control architecture
- Quantization
- Speculative decoding
- Multi-GPU setup
:::

## Why this matters

Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architecture.

## Overview

Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architecture. This lesson covers the practical engineering of deploying foundation models on G1.

## Key concepts

:::note Key idea

NVIDIA AGX Orin: 275 TOPS INT8, 64GB LPDDR5; can run 7B parameter quantized model at ~5 Hz; sufficient for high-level task planning but not high-frequency control

:::

- Two-level control architecture: VLA at 5-10 Hz for language-conditioned task planning → low-level reactive controller at 1-2 kHz for joint execution
- Quantization: INT4/INT8 quantization reduces model size 2-4×, inference speed 2-3×; minimal quality loss for VLA action prediction with AWQ quantization
- Speculative decoding: generate action candidates in parallel; evaluate with discriminator; reduces effective latency by 2-3× for transformer-based policies
- Multi-GPU setup: Figure 02 uses 2× Orin NX modules — one for perception + VLA inference, one for locomotion control and safety monitoring
- Safety wrapper: VLA outputs pass through a safety filter that checks joint limits, velocity limits, and collision prediction before execution — essential for healthcare

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about NVIDIA AGX Orin?</summary>

275 TOPS INT8, 64GB LPDDR5; can run 7B parameter quantized model at ~5 Hz; sufficient for high-level task planning but not high-frequency control

</details>

<details>
<summary>Q2. What do you know about Two-level control architecture?</summary>

VLA at 5-10 Hz for language-conditioned task planning → low-level reactive controller at 1-2 kHz for joint execution

</details>

<details>
<summary>Q3. What do you know about Quantization?</summary>

INT4/INT8 quantization reduces model size 2-4×, inference speed 2-3×; minimal quality loss for VLA action prediction with AWQ quantization

</details>

<details>
<summary>Q4. What do you know about Speculative decoding?</summary>

generate action candidates in parallel; evaluate with discriminator; reduces effective latency by 2-3× for transformer-based policies

</details>

<details>
<summary>Q5. What do you know about Multi-GPU setup?</summary>

Figure 02 uses 2× Orin NX modules — one for perception + VLA inference, one for locomotion control and safety monitoring

</details>

---

← Previous: **5.4 Diffusion Policy — Visuomotor Control via Denoising**

*Part of Module 5: Foundation Models & VLA Architecture.*


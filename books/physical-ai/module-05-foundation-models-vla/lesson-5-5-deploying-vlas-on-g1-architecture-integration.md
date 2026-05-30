---
title: "Deploying VLAs on G1: Architecture & Integration"
sidebar_label: "5.5 Deploying VLAs on G1: Architecture & Integration"
sidebar_position: 6
description: "Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architect"
tags: ["deployment", "inference", "hardware", "architecture"]
---

# Deploying VLAs on G1: Architecture & Integration

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Tags:** `deployment`, `inference`, `hardware`, `architecture`

## Overview

Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architecture. This lesson covers the practical engineering of deploying foundation models on G1.

## Key Insights

- NVIDIA AGX Orin: 275 TOPS INT8, 64GB LPDDR5; can run 7B parameter quantized model at ~5 Hz; sufficient for high-level task planning but not high-frequency control
- Two-level control architecture: VLA at 5-10 Hz for language-conditioned task planning → low-level reactive controller at 1-2 kHz for joint execution
- Quantization: INT4/INT8 quantization reduces model size 2-4×, inference speed 2-3×; minimal quality loss for VLA action prediction with AWQ quantization
- Speculative decoding: generate action candidates in parallel; evaluate with discriminator; reduces effective latency by 2-3× for transformer-based policies
- Multi-GPU setup: Figure 02 uses 2× Orin NX modules — one for perception + VLA inference, one for locomotion control and safety monitoring
- Safety wrapper: VLA outputs pass through a safety filter that checks joint limits, velocity limits, and collision prediction before execution — essential for healthcare


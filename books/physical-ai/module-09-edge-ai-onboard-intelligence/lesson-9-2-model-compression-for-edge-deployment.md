---
title: "Model Compression for Edge Deployment"
sidebar_label: "9.2 Model Compression for Edge Deployment"
sidebar_position: 3
description: "A 7B parameter VLA model in FP32 requires 28GB of memory and inference at <1 Hz on an AGX Orin — unusable"
tags: ["quantization", "compression", "TensorRT", "efficiency"]
---

# Model Compression for Edge Deployment

**Duration:** 55 min · **Level:** Advanced · **Module:** 9. Edge AI & On-Board Intelligence · **Focus:** `quantization`, `compression`, `TensorRT`, `efficiency`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- INT4 quantization (AWQ, GPTQ)
- AWQ (Activation-Aware Weight Quantization)
- Knowledge distillation
- Pruning
- TensorRT
:::

## Overview

A 7B parameter VLA model in FP32 requires 28GB of memory and inference at &lt;1 Hz on an AGX Orin — unusable. Model compression techniques (quantization, pruning, distillation) can reduce this to 4GB and 5-10 Hz without meaningful accuracy loss.

## Key concepts

:::note Key idea

INT4 quantization (AWQ, GPTQ): reduce 7B FP16 (14GB) → 7B INT4 (3.5GB); 4× memory reduction, 3-4× speedup; &lt;1% accuracy drop on action prediction benchmarks

:::

- AWQ (Activation-Aware Weight Quantization): identifies and preserves "salient" weights in high-precision; best INT4 method for LLM-derived VLA models; available in TensorRT-LLM
- Knowledge distillation: train a smaller "student" model to match outputs of a larger "teacher" VLA; Pi0-small (1B params) can approach Pi0-3B performance with distillation
- Pruning: remove unimportant weights (structured: remove attention heads; unstructured: zero individual weights); 20-30% of weights can often be pruned with minimal impact
- TensorRT: NVIDIA's inference optimizer; fuses layers, optimizes memory access patterns; typically 2-4× speedup on top of quantization for Orin deployment
- Target spec for G1: 7B VLA at INT4 → 4-6 Hz on single AGX Orin AGX; combine with 1-2 kHz low-level reactive controller running on CPU

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>INT4 quantization (AWQ, GPTQ)</summary>

reduce 7B FP16 (14GB) → 7B INT4 (3.5GB); 4× memory reduction, 3-4× speedup; &lt;1% accuracy drop on action prediction benchmarks

</details>

<details>
<summary>AWQ (Activation-Aware Weight Quantization)</summary>

identifies and preserves "salient" weights in high-precision; best INT4 method for LLM-derived VLA models; available in TensorRT-LLM

</details>

<details>
<summary>Knowledge distillation</summary>

train a smaller "student" model to match outputs of a larger "teacher" VLA; Pi0-small (1B params) can approach Pi0-3B performance with distillation

</details>

<details>
<summary>Pruning</summary>

remove unimportant weights (structured: remove attention heads; unstructured: zero individual weights); 20-30% of weights can often be pruned with minimal impact

</details>

<details>
<summary>TensorRT</summary>

NVIDIA's inference optimizer; fuses layers, optimizes memory access patterns; typically 2-4× speedup on top of quantization for Orin deployment

</details>

## References

- **AWQ: Activation-aware Weight Quantization for On-Device LLM Compression and Acceleration** — Lin et al. (2023). *MLSys 2024*

---

← Previous: **9.1 NVIDIA Jetson AGX Orin: The Humanoid Brain** · Next: **9.3 Neuromorphic Computing & Event Cameras** →

*Part of Module 9: Edge AI & On-Board Intelligence.*


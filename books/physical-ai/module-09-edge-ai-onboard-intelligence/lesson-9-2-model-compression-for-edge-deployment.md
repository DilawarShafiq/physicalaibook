---
title: "Model Compression for Edge Deployment"
sidebar_label: "9.2 Model Compression for Edge Deployment"
sidebar_position: 3
description: "A 7B parameter VLA model in FP32 requires 28GB of memory and inference at <1 Hz on an AGX Orin — unusable"
tags: ["quantization", "compression", "TensorRT", "efficiency"]
---

# Model Compression for Edge Deployment

**Duration:** 55 min · **Level:** Advanced · **Module:** 9. Edge AI & On-Board Intelligence · **Tags:** `quantization`, `compression`, `TensorRT`, `efficiency`

## Overview

A 7B parameter VLA model in FP32 requires 28GB of memory and inference at &lt;1 Hz on an AGX Orin — unusable. Model compression techniques (quantization, pruning, distillation) can reduce this to 4GB and 5-10 Hz without meaningful accuracy loss.

## Key Insights

- INT4 quantization (AWQ, GPTQ): reduce 7B FP16 (14GB) → 7B INT4 (3.5GB); 4× memory reduction, 3-4× speedup; &lt;1% accuracy drop on action prediction benchmarks
- AWQ (Activation-Aware Weight Quantization): identifies and preserves "salient" weights in high-precision; best INT4 method for LLM-derived VLA models; available in TensorRT-LLM
- Knowledge distillation: train a smaller "student" model to match outputs of a larger "teacher" VLA; Pi0-small (1B params) can approach Pi0-3B performance with distillation
- Pruning: remove unimportant weights (structured: remove attention heads; unstructured: zero individual weights); 20-30% of weights can often be pruned with minimal impact
- TensorRT: NVIDIA's inference optimizer; fuses layers, optimizes memory access patterns; typically 2-4× speedup on top of quantization for Orin deployment
- Target spec for G1: 7B VLA at INT4 → 4-6 Hz on single AGX Orin AGX; combine with 1-2 kHz low-level reactive controller running on CPU

## References

- **AWQ: Activation-aware Weight Quantization for On-Device LLM Compression and Acceleration** — Lin et al. (2023). *MLSys 2024*


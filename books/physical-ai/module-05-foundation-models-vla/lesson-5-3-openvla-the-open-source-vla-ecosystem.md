---
title: "OpenVLA — The Open-Source VLA Ecosystem"
sidebar_label: "5.3 OpenVLA — The Open-Source VLA Ecosystem"
sidebar_position: 4
description: "OpenVLA (June 2024, Stanford + Berkeley) is the first fully open-source VLA that matches RT-2-X performance on standard benchmarks"
tags: ["OpenVLA", "open-source", "fine-tuning", "VLA"]
---

# OpenVLA — The Open-Source VLA Ecosystem

**Duration:** 60 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Tags:** `OpenVLA`, `open-source`, `fine-tuning`, `VLA`

## Overview

OpenVLA (June 2024, Stanford + Berkeley) is the first fully open-source VLA that matches RT-2-X performance on standard benchmarks. Its release democratized VLA research: any lab can fine-tune a state-of-the-art general-purpose robot policy on their own hardware.

## Key Insights

- OpenVLA architecture: 7.5B parameters based on Prismatic VLM (SigLIP vision encoder + Llama 2 language model); actions tokenized as discrete language tokens
- Performance: matches RT-2-X on BridgeV2 benchmark, outperforms on several manipulation tasks; runs at 6 Hz on single A100 GPU, 1.5 Hz on NVIDIA Orin AGX
- Fine-tuning: LoRA fine-tuning on 200-500 demonstrations converges in 2-4 hours on single GPU; makes task-specific adaptation practical for small labs
- Action tokenization: continuous actions discretized to 256 bins per dimension, encoded as language tokens; allows use of LLM training infrastructure directly
- OpenVLA-OFT (2025 follow-up): parallel decoding + action chunking reduces latency to 25+ Hz; enables contact-rich, high-frequency manipulation
- Available at: github.com/openvla/openvla with pre-trained checkpoints on HuggingFace; Apache 2.0 license

## References

- **OpenVLA: An Open-Source Vision-Language-Action Model** — Kim et al. (2024). *arXiv 2406.09246*


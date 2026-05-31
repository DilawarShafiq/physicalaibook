---
title: "Deploying VLAs on G1: Architecture & Integration"
sidebar_label: "5.5 Deploying VLAs on G1: Architecture & Integration"
sidebar_position: 6
description: "Running a 3-7B parameter VLA model at useful frequency on a battery-powered humanoid requires careful co-design of the inference pipeline, compute allocation, and control architect"
tags: ["deployment", "inference", "hardware", "architecture"]
---

# Deploying VLAs on G1: Architecture & Integration

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. Foundation Models & VLA Architecture · **Focus:** `deployment`, `inference`, `hardware`, `architecture`

A 3-to-7-billion-parameter VLA is a marvel in a data center and a problem on a robot. The G1 humanoid runs on a battery and carries its own compute, yet you want it to think with a foundation model *and* keep its balance at kilohertz rates. Those two demands cannot be met by one model on one loop. This lesson is about the engineering that reconciles them: how to allocate compute, split the control architecture into the right frequencies, shrink the model to fit, and — non-negotiably for healthcare — wrap the whole thing in a safety filter. By the end you will size a deployment that actually closes the loop.

## The compute budget you're working against

The reference onboard accelerator is the **NVIDIA AGX Orin: 275 TOPS of INT8 compute and 64 GB of LPDDR5 memory**. That is enough to run a **7B-parameter quantized model at about 5 Hz**. Read that number carefully. Five hertz is **sufficient for high-level task planning** — deciding *what* to do, conditioned on language and vision — but it is nowhere near enough for high-frequency control. A robot cannot stay upright or react to contact on a 5 Hz loop. The hardware does not let you run the big model fast; it lets you run it *slowly*, which turns out to be exactly the right speed for one specific job.

## The two-level control architecture

The resolution to the frequency problem is to stop pretending one model does everything. **Split control into two levels:**

- **The VLA runs at 5-10 Hz** as the high-level, language-conditioned task planner. It looks at the scene, reads the instruction, and decides the next sub-goal or action chunk.
- **A low-level reactive controller runs at 1-2 kHz** for joint execution — balance, contact reaction, trajectory tracking. This is the loop that keeps the robot standing and safe.

This maps cleanly onto the dual-system idea now common across the field (NVIDIA's GR00T N1, for example, frames it as a slow System 2 feeding a fast System 1): a slow, smart planner setting goals for a fast, reflexive controller that executes them.

```mermaid
flowchart LR
    Scene["Vision and language input"] --> VLA["VLA planner at 5-10 Hz"]
    VLA --> Goal["Sub-goal or action chunk"]
    Goal --> Safety["Safety filter checks limits"]
    Safety --> Ctrl["Reactive controller at 1-2 kHz"]
    Ctrl --> Joints["Joint execution and balance"]
``` The architectural rule to internalize: **match each job to its natural frequency.** Semantic decisions are slow and infrequent; physical stability is fast and constant. Forcing them onto one loop wastes the expensive model on a rate it can't sustain and starves the safety loop of the speed it needs.

## Making the big model fit and run faster

Even at 5-10 Hz, you often need to squeeze more out of Orin. Two techniques do most of the work.

**Quantization.** Reducing weights to **INT4 or INT8 cuts model size 2-4× and speeds inference 2-3×**, with **minimal quality loss for VLA action prediction when you use AWQ quantization**. AWQ (activation-aware weight quantization) preserves the weights that matter most, which is why action quality holds up. This is usually the single highest-leverage optimization: it is what turns "a 7B model that barely loads" into "a 7B model that runs at a useful rate."

**Speculative decoding.** Generate **action candidates in parallel, then evaluate them with a discriminator**, keeping the good ones. For transformer-based policies this **cuts effective latency by 2-3×**. It is a way to spend Orin's parallel throughput to hide the sequential cost of token-by-token generation.

Used together, quantization and speculative decoding are often what move a model from "technically runs" to "closes the loop at the frequency the planner needs."

## Scaling out: multiple accelerators and the safety wrapper

When one Orin is not enough, the production pattern is to **use more than one and split by function**. The reference point: **Figure 02 uses two Orin NX modules — one for perception plus VLA inference, one for locomotion control and safety monitoring.** This is the two-level architecture made physical. Perception and the slow VLA live on one chip; the fast control and safety loop live on another, so heavy inference can never stall the loop that keeps the robot upright.

That separation sets up the most important component for a healthcare robot. **Every VLA output must pass through a safety filter before execution.** The filter checks **joint limits, velocity limits, and collision prediction**, and rejects or clamps any command that violates them. The reasoning is blunt: a VLA is a learned, probabilistic system that will occasionally emit a bad action, and a humanoid working near patients cannot be allowed to execute one. The safety wrapper is the deterministic guarantee that sits between a fallible model and the actuators — **essential for healthcare**, and best run on the same fast, isolated controller that handles low-level execution so it can intervene at kilohertz rates.

## Putting it into practice

Size a real G1 deployment and prove the loop closes.

1. **Start from the budget.** Assume one AGX Orin (275 TOPS INT8, 64 GB). Take a 7B VLA at ~5 Hz quantized as your baseline planner rate.
2. **Lay out the two levels.** Assign the VLA to 5-10 Hz for task planning and a reactive controller to 1-2 kHz for joint execution. Write what each loop is responsible for, and confirm no semantic decision sits on the kHz loop and no balance decision sits on the 5 Hz loop.
3. **Apply the optimizations.** Choose INT4 or INT8 AWQ quantization and estimate the resulting size (2-4× smaller) and speed (2-3× faster). Decide whether to add speculative decoding for another 2-3× latency cut, and compute your projected planner frequency.
4. **Decide single- versus dual-accelerator.** If your projected rate or safety isolation is marginal, adopt the Figure-02 split — one accelerator for perception plus VLA, one for control plus safety — and say why.
5. **Specify the safety filter.** List the exact checks (joint limits, velocity limits, collision prediction), define what happens on a violation (clamp or reject), place it on the fast controller, and state in one sentence why it is mandatory before any actuator command in a healthcare setting.

## Key takeaways

- An NVIDIA AGX Orin (275 TOPS INT8, 64 GB) runs a 7B quantized VLA at about 5 Hz — enough for high-level task planning, not for high-frequency control.
- Split control into two levels: the VLA at 5-10 Hz for language-conditioned planning, and a reactive controller at 1-2 kHz for joint execution — matching each job to its natural frequency.
- Quantization (INT4/INT8 with AWQ) shrinks the model 2-4× and speeds it 2-3× with minimal action-quality loss; speculative decoding cuts transformer-policy latency another 2-3×.
- Scale out by function: Figure 02 uses two Orin NX modules, one for perception plus VLA inference and one for locomotion and safety, so heavy inference never stalls the control loop.
- A safety filter checking joint limits, velocity limits, and collision prediction must gate every VLA output before execution — the deterministic guard between a probabilistic model and the actuators, essential for healthcare.
- The recurring principle: a slow, smart planner setting goals for a fast, reflexive controller (the dual-system pattern) is how foundation models actually run on a battery-powered humanoid.

---

← Previous: **5.4 Diffusion Policy — Visuomotor Control via Denoising**

*Part of Module 5: Foundation Models & VLA Architecture.*


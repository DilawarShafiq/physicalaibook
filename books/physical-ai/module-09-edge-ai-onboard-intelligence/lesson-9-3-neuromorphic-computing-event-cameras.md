---
title: "Neuromorphic Computing & Event Cameras"
sidebar_label: "9.3 Neuromorphic Computing & Event Cameras"
sidebar_position: 4
description: "Neuromorphic chips (Intel Loihi 2, IBM NorthPole) and event cameras offer a radically different compute paradigm: sparse, asynchronous, ultra-low-power processing inspired by biolo"
tags: ["neuromorphic", "Loihi", "event-cameras", "low-power"]
---

# Neuromorphic Computing & Event Cameras

**Duration:** 55 min · **Level:** Advanced · **Module:** 9. Edge AI & On-Board Intelligence · **Tags:** `neuromorphic`, `Loihi`, `event-cameras`, `low-power`

## Overview

Neuromorphic chips (Intel Loihi 2, IBM NorthPole) and event cameras offer a radically different compute paradigm: sparse, asynchronous, ultra-low-power processing inspired by biological neural circuits. For G1, they offer a path to always-on perception that consumes milliwatts rather than watts.

## Key Insights

- Intel Loihi 2 (2022): 1 million neurons, 120 million synapses per chip, 0.5W power; runs spiking neural networks (SNNs) 100× more energy-efficiently than equivalent GPU ops
- Event cameras (DAVIS346, Prophesee EVK4): output asynchronous "events" (pixel, timestamp, polarity) when pixel brightness changes; μs latency, 120 dB dynamic range, no motion blur
- SNN locomotion controller: Intel/ETH Zurich collaboration (2023) — running SNN locomotion policy on Loihi 2 for ANYmal; 75× lower energy than GPU equivalent; no meaningful performance loss
- Event camera + neuromorphic: event stream → SNN on Loihi 2 → obstacle detection at 10,000 Hz update rate in &lt;1mW; critical for detecting fast-moving hazards near humans
- Limitation: programming SNNs requires specialized tools (NEST, PyNN, Intel's nxSDK); most researchers lack SNN expertise; steep learning curve
- G1 application: Loihi 2 co-processor for always-on safety monitoring (detect human approach, collision prediction) while main Orin is in low-power sleep mode

## References

- **Intel Loihi 2: A New Generation of Neuromorphic Processor** — Davies et al. (2022). *IEEE Micro 2022*
- **Neuromorphic Control of a Quadruped Robot** — Müller-Cleve et al. (2023). *Science Robotics 2023*


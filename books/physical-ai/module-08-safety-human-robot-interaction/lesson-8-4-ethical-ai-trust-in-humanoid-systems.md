---
title: "Ethical AI & Trust in Humanoid Systems"
sidebar_label: "8.4 Ethical AI & Trust in Humanoid Systems"
sidebar_position: 5
description: "A humanoid robot in a hospital operates in the most trust-sensitive environment imaginable"
tags: ["ethics", "trust", "HIPAA", "AI-Act"]
---

# Ethical AI & Trust in Humanoid Systems

**Duration:** 40 min · **Level:** Intermediate · **Module:** 8. Safety & Human-Robot Interaction · **Focus:** `ethics`, `trust`, `HIPAA`, `AI-Act`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- EU AI Act (2024)
- HIPAA compliance
- Explainability
- Fail-safe design
- Bias and fairness
:::

## Why this matters

A humanoid robot in a hospital operates in the most trust-sensitive environment imaginable.

## Overview

A humanoid robot in a hospital operates in the most trust-sensitive environment imaginable. Patients are vulnerable, stakes are high, and errors have immediate consequences. This lesson addresses the ethical framework, transparency requirements, and trust-building strategies for G1's deployment.

## Key concepts

:::note Key idea

EU AI Act (2024): classifies robots in healthcare as "high-risk AI systems" requiring conformity assessment, human oversight, and explainability of automated decisions

:::

- HIPAA compliance: any robot that handles patient data (names, conditions, medication info) or PHI must implement HIPAA-compliant data handling; no PHI in model training data
- Explainability: G1 should communicate its intent before acting — "I am going to hand you this medication" — and confirm understanding before proceeding; reduces startlement
- Fail-safe design: G1 must default to safe state on any system failure — drop to minimal motion, audible alert, wait for human intervention; never a "frozen at full torque" failure
- Bias and fairness: manipulation policies trained on limited demographics may perform worse on different body types, skin tones, or clothing; evaluate across diverse populations
- Public trust: OpenAI and Figure demonstrate tasks publicly before deployment; transparency in capabilities and limitations builds trust faster than secrecy

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about EU AI Act (2024)?</summary>

classifies robots in healthcare as "high-risk AI systems" requiring conformity assessment, human oversight, and explainability of automated decisions

</details>

<details>
<summary>Q2. What do you know about HIPAA compliance?</summary>

any robot that handles patient data (names, conditions, medication info) or PHI must implement HIPAA-compliant data handling; no PHI in model training data

</details>

<details>
<summary>Q3. What do you know about Explainability?</summary>

G1 should communicate its intent before acting — "I am going to hand you this medication" — and confirm understanding before proceeding; reduces startlement

</details>

<details>
<summary>Q4. What do you know about Fail-safe design?</summary>

G1 must default to safe state on any system failure — drop to minimal motion, audible alert, wait for human intervention; never a "frozen at full torque" failure

</details>

<details>
<summary>Q5. What do you know about Bias and fairness?</summary>

manipulation policies trained on limited demographics may perform worse on different body types, skin tones, or clothing; evaluate across diverse populations

</details>

---

← Previous: **8.3 Behavior Trees & Task-Level Safety**

*Part of Module 8: Safety & Human-Robot Interaction.*


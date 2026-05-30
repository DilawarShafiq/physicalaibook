---
title: "Building with Claude: Prompting, Tools & Compliance"
sidebar_label: "H5.2 Building with Claude: Prompting, Tools & Compliance"
sidebar_position: 3
description: "Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regu"
tags: ["Claude", "Anthropic", "HIPAA-BAA", "tool-use", "prompt-engineering"]
---

# Building with Claude: Prompting, Tools & Compliance

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. The Personal Medical Biller · **Focus:** `Claude`, `Anthropic`, `HIPAA-BAA`, `tool-use`, `prompt-engineering`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Anthropic HIPAA BAA
- System prompt design
- Tool use design
- PHI in prompts
- Prompt caching
:::

## Why this matters

Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regulated tasks, and reliable refusal of PHI-mishandling requests.

## Overview

Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regulated tasks, and reliable refusal of PHI-mishandling requests. This lesson covers the specific engineering patterns for Claude-based healthcare agents.

## Key concepts

:::note Key idea

Anthropic HIPAA BAA: Anthropic offers BAAs for Claude API customers via Claude for Enterprise; required before any PHI can be sent to the Claude API; sign before building, not after; covers Claude 3 family models

:::

- System prompt design: healthcare agent system prompts must explicitly state: role and permitted actions, PHI handling instructions (never output PHI in plaintext logs), required disclaimers (not a medical advice system), escalation triggers, and compliance constraints
- Tool use design: use Claude's tool_use API to give agents structured access to EHR APIs, payer APIs, and database; tools enforce data access controls (Claude can only read data the tool explicitly returns) — more secure than letting the model interpret raw database dumps
- PHI in prompts: never include more PHI in a prompt than necessary for the specific task; use patient ID + encounter ID for context rather than full name + SSN; retrieve PHI in the tool response only when needed
- Prompt caching: Anthropic's prompt caching feature caches long system prompts and reference documents (payer policy library, ICD-10 codebook) — reduces cost and latency for agents that make many calls with the same base context; cache hit rate should be &gt;80%
- Structured output: use Claude's JSON mode or constrained output for all agent actions that feed downstream systems; unstructured free-text output from a coding agent cannot be safely passed to a claims system

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about Anthropic HIPAA BAA?</summary>

Anthropic offers BAAs for Claude API customers via Claude for Enterprise; required before any PHI can be sent to the Claude API; sign before building, not after; covers Claude 3 family models

</details>

<details>
<summary>Q2. What do you know about System prompt design?</summary>

healthcare agent system prompts must explicitly state: role and permitted actions, PHI handling instructions (never output PHI in plaintext logs), required disclaimers (not a medical advice system), escalation triggers, and compliance constraints

</details>

<details>
<summary>Q3. What do you know about Tool use design?</summary>

use Claude's tool_use API to give agents structured access to EHR APIs, payer APIs, and database; tools enforce data access controls (Claude can only read data the tool explicitly returns) — more secure than letting the model interpret raw database dumps

</details>

<details>
<summary>Q4. What do you know about PHI in prompts?</summary>

never include more PHI in a prompt than necessary for the specific task; use patient ID + encounter ID for context rather than full name + SSN; retrieve PHI in the tool response only when needed

</details>

<details>
<summary>Q5. What do you know about Prompt caching?</summary>

Anthropic's prompt caching feature caches long system prompts and reference documents (payer policy library, ICD-10 codebook) — reduces cost and latency for agents that make many calls with the same base context; cache hit rate should be &gt;80%

</details>

## References

- **Claude API Documentation — Tool Use and HIPAA** — Anthropic (2024). *docs.anthropic.com*

---

← Previous: **H5.1 Architecture: The Personal Medical Biller as a System of Agents** · Next: **H5.3 Patient-Facing Agent: The Billing Advocate Experience** →

*Part of Module 5: The Personal Medical Biller.*


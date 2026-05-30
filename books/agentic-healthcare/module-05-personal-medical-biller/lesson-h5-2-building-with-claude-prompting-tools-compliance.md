---
title: "Building with Claude: Prompting, Tools & Compliance"
sidebar_label: "H5.2 Building with Claude: Prompting, Tools & Compliance"
sidebar_position: 3
description: "Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regu"
tags: ["Claude", "Anthropic", "HIPAA-BAA", "tool-use", "prompt-engineering"]
---

# Building with Claude: Prompting, Tools & Compliance

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. The Personal Medical Biller · **Tags:** `Claude`, `Anthropic`, `HIPAA-BAA`, `tool-use`, `prompt-engineering`

## Overview

Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regulated tasks, and reliable refusal of PHI-mishandling requests. This lesson covers the specific engineering patterns for Claude-based healthcare agents.

## Key Insights

- Anthropic HIPAA BAA: Anthropic offers BAAs for Claude API customers via Claude for Enterprise; required before any PHI can be sent to the Claude API; sign before building, not after; covers Claude 3 family models
- System prompt design: healthcare agent system prompts must explicitly state: role and permitted actions, PHI handling instructions (never output PHI in plaintext logs), required disclaimers (not a medical advice system), escalation triggers, and compliance constraints
- Tool use design: use Claude's tool_use API to give agents structured access to EHR APIs, payer APIs, and database; tools enforce data access controls (Claude can only read data the tool explicitly returns) — more secure than letting the model interpret raw database dumps
- PHI in prompts: never include more PHI in a prompt than necessary for the specific task; use patient ID + encounter ID for context rather than full name + SSN; retrieve PHI in the tool response only when needed
- Prompt caching: Anthropic's prompt caching feature caches long system prompts and reference documents (payer policy library, ICD-10 codebook) — reduces cost and latency for agents that make many calls with the same base context; cache hit rate should be &gt;80%
- Structured output: use Claude's JSON mode or constrained output for all agent actions that feed downstream systems; unstructured free-text output from a coding agent cannot be safely passed to a claims system

## References

- **Claude API Documentation — Tool Use and HIPAA** — Anthropic (2024). *docs.anthropic.com*


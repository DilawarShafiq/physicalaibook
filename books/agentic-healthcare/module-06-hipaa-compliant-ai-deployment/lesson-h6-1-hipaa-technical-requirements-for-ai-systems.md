---
title: "HIPAA Technical Requirements for AI Systems"
sidebar_label: "H6.1 HIPAA Technical Requirements for AI Systems"
sidebar_position: 2
description: "HIPAA's Security Rule was written in 2003 — before large language models, cloud computing, and AI agents existed"
tags: ["HIPAA", "security", "BAA", "audit-trail", "compliance"]
---

# HIPAA Technical Requirements for AI Systems

**Duration:** 55 min · **Level:** Advanced · **Module:** 6. HIPAA-Compliant AI Agent Deployment · **Tags:** `HIPAA`, `security`, `BAA`, `audit-trail`, `compliance`

## Overview

HIPAA's Security Rule was written in 2003 — before large language models, cloud computing, and AI agents existed. Applying its principles to modern AI agent architectures requires careful interpretation. The core requirement is unchanged: protect ePHI with administrative, physical, and technical safeguards appropriate to the risk.

## Key Insights

- BAA chain: every vendor in the data flow that receives PHI must sign a BAA; this includes: cloud infrastructure (AWS BAA, Azure BAA, Google Cloud BAA), LLM provider (Anthropic Claude Enterprise BAA), database vendor, clearinghouse, and any subcontractors
- PHI minimization in AI: the least PHI in AI model inputs, the lower the risk; design agents to use patient IDs and retrieve PHI only when the specific task requires it; do not batch-send full patient records to LLM for general processing
- Audit trail requirements: all PHI access, creation, modification, and disclosure must be logged with user ID (or agent ID), timestamp, action, and the specific PHI involved; logs must be retained per retention policy and protected from modification
- Encryption at rest and in transit: all ePHI must be encrypted at rest (AES-256) and in transit (TLS 1.3); LLM API calls containing PHI must be over TLS; log files containing PHI must be encrypted
- Agent identity management: each agent must have a distinct identity (service account), not use human credentials; least-privilege access (agent can only access the APIs it needs for its role); credentials rotated regularly
- Risk analysis for AI: HIPAA requires documented risk analysis; for AI systems, this includes: risk of LLM model hallucination leading to PHI disclosure, risk of prompt injection attacks, risk of training data memorization, risk of model API breach


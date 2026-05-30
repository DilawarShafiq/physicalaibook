---
title: "LLM-Based Diagnosis Code Extraction: The Architecture"
sidebar_label: "H3.1 LLM-Based Diagnosis Code Extraction: The Architecture"
sidebar_position: 2
description: "Large language models understand clinical text better than any previous NLP technology"
tags: ["ICD-10", "LLM-coding", "diagnosis-extraction", "Claude", "AI-coding"]
---

# LLM-Based Diagnosis Code Extraction: The Architecture

**Duration:** 60 min · **Level:** Advanced · **Module:** 3. AI Medical Coding & CDI · **Tags:** `ICD-10`, `LLM-coding`, `diagnosis-extraction`, `Claude`, `AI-coding`

## Overview

Large language models understand clinical text better than any previous NLP technology. A well-prompted Claude or GPT-4 can read a discharge summary and suggest appropriate ICD-10-CM codes with accuracy that approaches — and in some domains exceeds — trained human coders. The engineering challenge is not the model; it is the context, the validation layer, and the workflow integration.

## Key Insights

- LLM coding approach: feed the complete discharge summary + progress notes + operative report → ask model to identify all reportable diagnoses and procedures → model returns codes with supporting text citations → validation layer checks codes against ICD-10 tabular list
- Context engineering: LLMs need the complete clinical picture; feeding only the discharge diagnosis list misses secondary diagnoses, CCs/MCCs (complication/comorbidity codes that increase DRG weight significantly)
- Principal diagnosis selection: the condition "after study" chiefly responsible for admission; LLMs sometimes misidentify this as the patient's chief complaint; fine-tuning or few-shot examples on specialty-specific cases improves accuracy
- MCC/CC coding impact: each additional complication or comorbidity code (sepsis, respiratory failure, acute kidney injury) can add $5,000-$15,000 in DRG reimbursement for the same admission; AI catches these more consistently than fatigued human coders
- AHIMA/AHA Coding Clinic: official guidance on ICD-10 coding conventions; must be incorporated into AI system prompts as authoritative reference; published quarterly; subscription required
- Model benchmark (2024 data): Claude 3 Opus achieves 91-94% code-level accuracy on Medicare discharge summaries in untuned zero-shot setting; fine-tuned models achieve 96-98% on in-distribution data

:::tip Lab

Build a minimal coding agent: (1) take a 500-word simulated discharge summary as input, (2) prompt Claude to extract principal diagnosis, secondary diagnoses, and procedure codes with textual evidence, (3) validate each code against a local ICD-10 code list, (4) output a structured JSON with codes, confidence, and supporting text citation.

:::

## References

- **Automated ICD Coding with Large Language Models** — Huang et al. (2024). *JAMIA 2024*


---
title: "LLM-Based Diagnosis Code Extraction: The Architecture"
sidebar_label: "H3.1 LLM-Based Diagnosis Code Extraction: The Architecture"
sidebar_position: 2
description: "Large language models understand clinical text better than any previous NLP technology"
tags: ["ICD-10", "LLM-coding", "diagnosis-extraction", "Claude", "AI-coding"]
---

# LLM-Based Diagnosis Code Extraction: The Architecture

**Duration:** 60 min · **Level:** Advanced · **Module:** 3. AI Medical Coding & CDI · **Focus:** `ICD-10`, `LLM-coding`, `diagnosis-extraction`, `Claude`, `AI-coding`

Medical coding is the act of translating a clinical narrative — a discharge summary, a stack of progress notes, an operative report — into the discrete ICD-10-CM and procedure codes that drive reimbursement. It is high-volume, rule-bound, and expensive, which makes it an ideal target for automation. Large language models now understand clinical text better than any previous NLP technology. A well-prompted Claude or GPT-4 can read a discharge summary and suggest appropriate ICD-10-CM codes with accuracy that approaches — and in some domains exceeds — trained human coders. The engineering challenge is not the model. It is the context you feed it, the validation layer that catches its mistakes, and the workflow that puts its output in front of a human at the right moment.

## The pipeline, end to end

A coding agent is a pipeline, not a single prompt. The shape that works in practice is this: feed the model the complete clinical picture — discharge summary plus progress notes plus operative report — and ask it to identify every reportable diagnosis and procedure. The model returns codes, each one tied to the specific span of text that supports it. A validation layer then checks each suggested code against the ICD-10 tabular list to confirm it is a real, currently valid code and not a hallucinated string. Only codes that survive validation reach the human coder for sign-off.

That last point is the heart of the design. The LLM is a *candidate generator*. It is fast, it is thorough, and it occasionally invents something plausible-but-wrong. The validation layer and the human reviewer are what convert raw model output into a billable, auditable result. Treat the model as a brilliant junior coder who never gets tired but always needs a check — never as the final authority.

## Context is the whole game

The single most common way to build a bad coding agent is to feed it too little. If you hand the model only the discharge diagnosis list, it will code the discharge diagnosis list — and miss everything else. Secondary diagnoses, and especially CCs and MCCs (complication and comorbidity codes), live in the *body* of the chart: the labs, the medication orders, the daily progress notes, the consult findings. Those are exactly the codes that move money. Each additional complication or comorbidity code — sepsis, respiratory failure, acute kidney injury — can add roughly $5,000 to $15,000 in DRG reimbursement for the *same* admission. AI catches these more consistently than a fatigued human coder working a queue at 4 p.m., but only if the complete record is in its context window.

So context engineering, not prompt cleverness, is where you spend your effort. Assemble the full encounter. Strip nothing that carries clinical signal. The model can only code what it can see.

## Getting the principal diagnosis right

Not every diagnosis is equal. The **principal diagnosis** is the condition that, *after study*, is established as chiefly responsible for the admission — and it anchors the entire DRG. This is where LLMs stumble most reliably. A model will often grab the patient's *chief complaint* — the symptom they walked in with — and label it the principal diagnosis, when the correct answer is the underlying condition determined during the workup. Chest pain is a chief complaint; the NSTEMI found after study is the principal diagnosis.

The fix is not a bigger model; it is better examples. Few-shot prompting with specialty-specific cases, or fine-tuning on real principal-diagnosis selections, teaches the model the "after study" logic that the official guidelines demand. And those guidelines are not optional folklore: the **AHA Coding Clinic for ICD-10-CM/PCS** is the authoritative quarterly publication that governs ICD-10 coding conventions. Its guidance must be baked into your system prompt as the reference of record. (Note that Coding Clinic is a subscription publication — you cannot scrape it freely, and you should respect that in your architecture.)

## How good is good enough?

The benchmark numbers in this lesson set realistic expectations. On Medicare discharge summaries in an *untuned, zero-shot* setting, a frontier model like Claude 3 Opus reaches roughly 91–94% code-level accuracy. Fine-tuned models reach 96–98% on in-distribution data. Those numbers are good — better than many people expect — but they are not 100%, and the gap is precisely why the human-in-the-loop and the validation layer are non-negotiable. A 94% agent that submits unreviewed claims is a compliance incident waiting to happen. A 94% agent whose output a human confirms in seconds instead of composing from scratch is a genuine productivity multiplier.

## Choosing your stack

You have two broad architectural options.

**Option A — zero-shot or few-shot with a frontier model.** Fast to build, no training pipeline, easy to update as guidelines change. You pay in per-token cost and accept the lower end of the accuracy band. Best for getting a working agent in front of coders quickly and for lower-acuity, well-documented encounters.

**Option B — a fine-tuned model on your own coded charts.** Higher accuracy on the case mix you actually see, better principal-diagnosis selection, lower per-inference cost at scale. You pay in a training and maintenance pipeline and the need for clean labeled data.

**Recommendation:** start with Option A inside a HIPAA-compliant agentic platform, instrument it against a human-coder benchmark, and only graduate to Option B once you have enough validated, labeled charts to make fine-tuning worthwhile. Do not build the training pipeline before you have proven the workflow.

## Putting it into practice

Build a minimal coding agent end to end and measure it.

1. Take a 500-word simulated discharge summary as input. Do not use real PHI for this exercise.
2. Prompt the model to extract the principal diagnosis, all secondary diagnoses, and procedure codes — and require it to return the supporting text span for each.
3. Validate every returned code against a local ICD-10 code list, dropping or flagging any code that is not a valid entry.
4. Output a structured JSON record with `code`, `confidence`, and `supporting_text` for each finding — the exact shape a human reviewer needs to approve or reject in seconds.

## Key takeaways

- Treat the LLM as a candidate generator, not the final coder: model → validation against the ICD-10 tabular list → human sign-off.
- Context is the whole game — feed the complete record (notes, labs, meds, operative report), because secondary diagnoses and CCs/MCCs live in the body, not the discharge list.
- CCs and MCCs matter financially: each can add roughly $5,000–$15,000 in DRG reimbursement for the same admission, and AI catches them more consistently than tired humans.
- Principal-diagnosis selection (the condition responsible "after study") is the hardest part; few-shot examples or fine-tuning fix the chief-complaint confusion.
- Anchor the system to authoritative guidance — the AHA Coding Clinic — and expect ~91–94% accuracy zero-shot, 96–98% fine-tuned, which is exactly why human review stays in the loop.

## References

- **Automated ICD Coding with Large Language Models** — Huang et al. (2024). *JAMIA 2024*

---

Next: **H3.2 CDI: Generating Physician Queries Automatically** →

*Part of Module 3: AI Medical Coding & CDI.*


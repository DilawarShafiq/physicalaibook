---
title: "CDI: Generating Physician Queries Automatically"
sidebar_label: "H3.2 CDI: Generating Physician Queries Automatically"
sidebar_position: 3
description: "Clinical Documentation Improvement (CDI) is the practice of querying physicians to clarify documentation that is vague, conflicting, or insufficient for accurate coding"
tags: ["CDI", "physician-query", "documentation-improvement", "revenue-integrity"]
---

# CDI: Generating Physician Queries Automatically

**Duration:** 55 min · **Level:** Advanced · **Module:** 3. AI Medical Coding & CDI · **Focus:** `CDI`, `physician-query`, `documentation-improvement`, `revenue-integrity`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- CDI query triggers
- AHIMA/ACDIS query practice guidelines
- AI query generation
- Integration point
- Query response tracking
:::

## Overview

Clinical Documentation Improvement (CDI) is the practice of querying physicians to clarify documentation that is vague, conflicting, or insufficient for accurate coding. Traditionally CDI specialists manually review records; AI can generate query opportunities automatically from discharge documentation and route them in real-time during the hospitalization.

## Key concepts

:::note Key idea

CDI query triggers: conflicting documentation (body of note says "sepsis" but diagnosis list says "infection"), clinical indicators without diagnosis (high WBC + antibiotics + fever but no sepsis diagnosis), vague diagnoses ("respiratory failure" vs "acute hypoxic respiratory failure")

:::

- AHIMA/ACDIS query practice guidelines: queries must be unbiased (present options without leading); must be based on clinical documentation not assumption; compliant queries ask for clarification, not new information
- AI query generation: LLM reads the complete record, identifies clinical indicators (lab values, vitals, medications, imaging) that suggest diagnoses not explicitly documented, generates a compliant query text presenting the clinical evidence and asking the physician to clarify
- Integration point: real-time CDI query generated when attending physician opens the patient's note for a new progress note entry → query displayed as EHR Best Practice Advisory → physician responds in EHR → coding agent receives response immediately
- Query response tracking: track query response rate (target &gt;85%), query agree rate (% physician agrees with suggested documentation), revenue impact per query ($X DRG improvement); this data feeds the AI model improvement loop
- Compliance guardrails: AI-generated queries must be reviewed by a human CDI specialist or coded by a human before being sent to physicians in most health systems; full automation is a future state pending more validation data

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>CDI query triggers</summary>

conflicting documentation (body of note says "sepsis" but diagnosis list says "infection"), clinical indicators without diagnosis (high WBC + antibiotics + fever but no sepsis diagnosis), vague diagnoses ("respiratory failure" vs "acute hypoxic respiratory failure")

</details>

<details>
<summary>AHIMA/ACDIS query practice guidelines</summary>

queries must be unbiased (present options without leading); must be based on clinical documentation not assumption; compliant queries ask for clarification, not new information

</details>

<details>
<summary>AI query generation</summary>

LLM reads the complete record, identifies clinical indicators (lab values, vitals, medications, imaging) that suggest diagnoses not explicitly documented, generates a compliant query text presenting the clinical evidence and asking the physician to clarify

</details>

<details>
<summary>Integration point</summary>

real-time CDI query generated when attending physician opens the patient's note for a new progress note entry → query displayed as EHR Best Practice Advisory → physician responds in EHR → coding agent receives response immediately

</details>

<details>
<summary>Query response tracking</summary>

track query response rate (target &gt;85%), query agree rate (% physician agrees with suggested documentation), revenue impact per query ($X DRG improvement); this data feeds the AI model improvement loop

</details>

---

← Previous: **H3.1 LLM-Based Diagnosis Code Extraction: The Architecture** · Next: **H3.3 CPT Coding for Outpatient & E&M Automation** →

*Part of Module 3: AI Medical Coding & CDI.*


---
title: "Real-Time Eligibility: The 270/271 EDI Transaction"
sidebar_label: "H2.1 Real-Time Eligibility: The 270/271 EDI Transaction"
sidebar_position: 2
description: "Insurance eligibility verification answers a simple question: is this patient covered for this service today? Answering it manually takes 3-8 minutes of hold time"
tags: ["eligibility", "270/271", "EDI", "Availity", "agent-design"]
---

# Real-Time Eligibility: The 270/271 EDI Transaction

**Duration:** 55 min · **Level:** Intermediate · **Module:** 2. Eligibility & Prior Auth Agents · **Tags:** `eligibility`, `270/271`, `EDI`, `Availity`, `agent-design`

## Overview

Insurance eligibility verification answers a simple question: is this patient covered for this service today? Answering it manually takes 3-8 minutes of hold time. Answering it via EDI 270/271 transaction takes 0.8 seconds. An eligibility agent checks every scheduled appointment overnight, flags coverage issues before the patient arrives, and eliminates 90% of front-desk insurance calls.

## Key Insights

- HIPAA 270 transaction: eligibility inquiry; sent by provider to payer; contains: provider NPI, patient member ID + DOB, service type codes (e.g., 30=health benefit plan coverage, 1=medical care)
- HIPAA 271 transaction: eligibility response; returned by payer; contains: coverage status, deductible amounts, copay, coinsurance, benefits by service type, coverage dates, coordination of benefits (COB)
- Availity API: Availity is the largest multi-payer real-time eligibility network (1000+ payers); offers REST API wrapper around EDI 270/271; authentication via OAuth 2.0; $0.05-0.15 per transaction
- Coverage parsing: 271 responses vary significantly by payer; deductible remaining, out-of-pocket max, and benefit-specific copays must be extracted from complex X12 EDI XML; NLP/regex parsing often required for non-standard payer responses
- Agent workflow: nightly batch job → retrieve scheduled appointments for next 3 days → submit 270 for each patient/payer combination → parse 271 responses → flag issues (inactive coverage, coordination, prior auth required) → route alerts to front desk and scheduling
- Edge cases agents must handle: terminated coverage effective mid-month, Medicare as secondary payer, dependent on parent's plan (age 26 cutoff), retroactive coverage changes, Medicaid spend-down

:::tip Lab

Design an eligibility agent: write the data schema for a patient appointment record, define the API call to Availity (or mock), parse a sample 271 response to extract active coverage + deductible remaining + prior auth required flags, and define the three escalation paths (active/needs-PA/inactive coverage).

:::

## References

- **HIPAA EDI Transaction Set 270/271** — CMS and ASC X12 (2023). *CMS.gov Technical Reference*


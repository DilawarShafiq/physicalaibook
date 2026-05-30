---
title: "Prior Authorization Automation: From Request to Approval"
sidebar_label: "H2.2 Prior Authorization Automation: From Request to Approval"
sidebar_position: 3
description: "Prior authorization is where healthcare administrative waste is most visible and most painful"
tags: ["prior-authorization", "CMS-rule", "FHIR-PA", "CoverMyMeds", "automation"]
---

# Prior Authorization Automation: From Request to Approval

**Duration:** 60 min · **Level:** Intermediate · **Module:** 2. Eligibility & Prior Auth Agents · **Tags:** `prior-authorization`, `CMS-rule`, `FHIR-PA`, `CoverMyMeds`, `automation`

## Overview

Prior authorization is where healthcare administrative waste is most visible and most painful. CoverMyMeds (now RxBenefits), Surescripts, and payer portals each handle different authorization types. CMS finalized rules in 2024 requiring payers to support FHIR-based PA APIs — a landmark shift that makes prior auth automation dramatically more feasible.

## Key Insights

- CMS Interoperability and Prior Authorization Final Rule (January 2024): requires Medicare Advantage, Medicaid, and CHIP plans to implement FHIR-based PA APIs by January 2027; must respond within 72 hours (standard) and 24 hours (urgent)
- PA request components: requesting provider NPI + contact, patient demographics + insurance ID, diagnosis codes (ICD-10), procedure/drug codes (CPT/HCPCS/NDC), clinical documentation (clinical notes, labs, prior treatments tried)
- X-FHIR-PA API: the FHIR-based PA standard emerging from CMS rules; uses FHIR R4 Claim resources for submission and ClaimResponse for decisions; integrates directly with EHR via SMART on FHIR
- Gold carding: some payers grant "gold card" status to physicians with high PA approval rates — exempting them from PA requirements; track approval rates by payer to identify gold card opportunities
- PA agent design: (1) detect services requiring PA from the order (CPT + diagnosis + payer lookup), (2) gather clinical documentation from EHR, (3) submit via API or portal, (4) poll for determination, (5) alert on approval/denial/additional info request, (6) auto-escalate to human if peer-to-peer needed
- CoverMyMeds/RxBenefits API: largest pharmacy PA clearinghouse; REST API for drug PAs; covers 95% of commercially insured lives; significant pharmacy PA automation through this single integration


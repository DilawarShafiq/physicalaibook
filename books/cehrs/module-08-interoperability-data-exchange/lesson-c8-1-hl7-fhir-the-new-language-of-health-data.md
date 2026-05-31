---
title: "HL7 FHIR: The New Language of Health Data"
sidebar_label: "C8.1 HL7 FHIR: The New Language of Health Data"
sidebar_position: 2
description: "HL7 FHIR (Fast Healthcare Interoperability Resources) is the modern standard for exchanging health data via RESTful APIs"
tags: ["FHIR", "HL7", "interoperability", "APIs", "information-blocking"]
---

# HL7 FHIR: The New Language of Health Data

**Duration:** 55 min · **Level:** Intermediate · **Module:** 8. Interoperability & Data Exchange · **Focus:** `FHIR`, `HL7`, `interoperability`, `APIs`, `information-blocking`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- FHIR R4
- SMART on FHIR
- ONC Cures Act Final Rule (2020)
- Information blocking
- FHIR vs HL7 v2
:::

## Overview

HL7 FHIR (Fast Healthcare Interoperability Resources) is the modern standard for exchanging health data via RESTful APIs. Unlike the older HL7 v2 standard, FHIR uses web-standard formats (JSON, XML) and enables patient-facing apps to access health data directly. Since April 2021, certified EHRs must expose FHIR R4 APIs.

## Key concepts

:::note Key idea

FHIR R4: current stable version of HL7 FHIR; defines "resources" (Patient, Observation, Condition, MedicationRequest, DiagnosticReport) that map to specific data elements

:::

- SMART on FHIR: authorization framework built on OAuth 2.0 that allows third-party apps to securely access EHR data with patient consent; enables Apple Health, patient portals, and care coordination apps
- ONC Cures Act Final Rule (2020): requires EHR vendors to publish FHIR R4 APIs; prohibits information blocking; empowers patients to access their data via third-party apps without paying for it
- Information blocking: any practice by a health IT developer, health network, or healthcare provider that interferes with access, exchange, or use of electronic health information (EHI); ONC can impose civil monetary penalties up to $1M per violation
- FHIR vs HL7 v2: HL7 v2 is pipe-delimited text (MSH|^~\&|...) used in hospital messaging (ADT, lab results, orders); FHIR is JSON/XML via REST API used for patient-facing and cross-organization exchange; both are used simultaneously
- CDS Hooks: FHIR-based standard for delivering clinical decision support into EHR workflow; a third-party CDS service can display alerts and recommendations inside the EHR via standardized hooks

## Check your understanding

Cover the answers and try to recall each point before expanding it.

<details>
<summary>FHIR R4</summary>

current stable version of HL7 FHIR; defines "resources" (Patient, Observation, Condition, MedicationRequest, DiagnosticReport) that map to specific data elements

</details>

<details>
<summary>SMART on FHIR</summary>

authorization framework built on OAuth 2.0 that allows third-party apps to securely access EHR data with patient consent; enables Apple Health, patient portals, and care coordination apps

</details>

<details>
<summary>ONC Cures Act Final Rule (2020)</summary>

requires EHR vendors to publish FHIR R4 APIs; prohibits information blocking; empowers patients to access their data via third-party apps without paying for it

</details>

<details>
<summary>Information blocking</summary>

any practice by a health IT developer, health network, or healthcare provider that interferes with access, exchange, or use of electronic health information (EHI); ONC can impose civil monetary penalties up to $1M per violation

</details>

<details>
<summary>FHIR vs HL7 v2</summary>

HL7 v2 is pipe-delimited text (MSH|^~\&|...) used in hospital messaging (ADT, lab results, orders); FHIR is JSON/XML via REST API used for patient-facing and cross-organization exchange; both are used simultaneously

</details>

---

Next: **C8.2 HIEs, Carequality & CommonWell — Networks for Health Data** →

*Part of Module 8: Interoperability & Data Exchange.*


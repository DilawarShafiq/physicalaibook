---
title: "HL7 FHIR: The New Language of Health Data"
sidebar_label: "C8.1 HL7 FHIR: The New Language of Health Data"
sidebar_position: 2
description: "HL7 FHIR (Fast Healthcare Interoperability Resources) is the modern standard for exchanging health data via RESTful APIs"
tags: ["FHIR", "HL7", "interoperability", "APIs", "information-blocking"]
---

# HL7 FHIR: The New Language of Health Data

**Duration:** 55 min · **Level:** Intermediate · **Module:** 8. Interoperability & Data Exchange · **Focus:** `FHIR`, `HL7`, `interoperability`, `APIs`, `information-blocking`

For decades, hospital systems talked to each other in a terse, pipe-delimited dialect of HL7 that only integration engineers could read. It worked for moving lab results and admit messages between machines inside a building, but it was never built for a patient who wants to pull their own chart into an app on their phone. HL7 FHIR is the answer to that newer demand. It takes the same web technology that powers every other modern application — RESTful APIs, JSON, OAuth — and applies it to health data. Since April 2021, certified EHRs in the United States have been required to expose FHIR R4 APIs, which means this is no longer a future-facing topic. It is the law, and the CEHRS exam treats it as one of the defining shifts in how health information moves.

## FHIR R4 and the idea of resources

FHIR stands for **Fast Healthcare Interoperability Resources**, and **R4** is the current stable version you should name on the exam. The word that does the heavy lifting is *resource*. Instead of one giant message blob, FHIR breaks health data into discrete, reusable building blocks — each one a *resource* mapped to a specific kind of clinical information. The high-yield examples to memorize are **Patient, Observation, Condition, MedicationRequest, and DiagnosticReport.** A vital sign is an Observation; a diagnosis is a Condition; an order for a drug is a MedicationRequest.

The memory hook is to read the resource name literally: it usually describes exactly what it holds. If an exam question asks which FHIR resource carries a lab result or a blood pressure, *Observation* is the safe answer. This modular design is what lets an app request just the one slice of data it needs rather than parsing an entire document.

## SMART on FHIR and patient-facing apps

FHIR defines *what* the data looks like, but a second standard governs *who* is allowed to reach it. That standard is **SMART on FHIR**, an authorization framework built on **OAuth 2.0.** SMART on FHIR is what allows a third-party app to securely access EHR data *with the patient's consent* — without the patient ever handing over their portal password.

This is the plumbing behind the patient-controlled access the Cures Act envisioned. It is what lets **Apple Health**, patient portals, and care-coordination apps pull records directly from the EHR. For the exam, pair the two names together: FHIR is the data format, SMART on FHIR (OAuth 2.0) is the secure permission layer on top of it.

## The Cures Act Final Rule and information blocking

The legal engine behind FHIR adoption is the **ONC Cures Act Final Rule (2020).** It does three things worth memorizing as a set: it **requires EHR vendors to publish FHIR R4 APIs**, it **prohibits information blocking**, and it **empowers patients to access their data through third-party apps without paying for it.**

That last clause is deliberate — patients cannot be charged a fee simply to retrieve their own electronic health information through an app. **Information blocking** itself is any practice by a health IT developer, a health information network, or a healthcare provider that *interferes with the access, exchange, or use of electronic health information (EHI).* The consequences are real: **ONC can impose civil monetary penalties of up to $1 million per violation.** A strong exam answer connects the dots — the Cures Act mandated the FHIR APIs, banned blocking, and put a price tag on getting it wrong.

## FHIR versus HL7 v2 — two standards, one ecosystem

A frequent point of confusion is assuming FHIR replaced the old HL7 v2 standard. It did not; they run side by side, each suited to a different job. **HL7 v2 is pipe-delimited text** — you can recognize it by the leading `MSH|^~\&|` segment — and it lives inside hospital messaging: ADT (admit-discharge-transfer), lab results, and orders flowing between systems in the same facility. **FHIR is JSON or XML delivered over a REST API**, and it shines for patient-facing and cross-organization exchange.

The clean distinction to hold for the exam: HL7 v2 is the internal hospital workhorse; FHIR is the external, web-friendly, patient-and-partner-facing layer. *Both are used simultaneously.* Recognizing the `MSH|` opening as HL7 v2 versus a JSON resource as FHIR is exactly the kind of identification the exam likes to test.

## Clinical decision support delivered through FHIR

One more FHIR-based standard is worth naming: **CDS Hooks.** This is the mechanism for delivering clinical decision support directly into the EHR workflow. A third-party CDS service can surface alerts and recommendations *inside* the EHR through standardized "hooks" — for example, when a clinician opens a chart or signs an order. The takeaway is that FHIR is not only about moving records between systems; it also enables outside intelligence to appear at the point of care without bolting on a separate application.

## Putting it into practice

1. Make a flashcard for each of the five core FHIR resources — Patient, Observation, Condition, MedicationRequest, DiagnosticReport — with a one-line example of the data it holds. Drill until you can match a clinical fact to its resource instantly.
2. Write the Cures Act Final Rule (2020) as a three-part list: requires FHIR R4 APIs, bans information blocking, gives patients free app-based access. Add the penalty figure (up to $1M per violation) underneath.
3. Practice telling HL7 v2 from FHIR by sight: a string starting with `MSH|^~\&|` is v2; a JSON object naming a resource is FHIR. Note that v2 handles internal ADT/lab/order messaging while FHIR handles patient-facing and cross-organization exchange.
4. Pair the standards by function — FHIR (format), SMART on FHIR / OAuth 2.0 (secure consent), CDS Hooks (alerts in workflow) — so a question about *which* standard does *what* has a ready answer.

## Key takeaways

- **FHIR R4** is the current HL7 standard; it organizes data into **resources** (Patient, Observation, Condition, MedicationRequest, DiagnosticReport) exchanged via RESTful APIs in JSON or XML.
- **SMART on FHIR** is the **OAuth 2.0**-based authorization framework that lets third-party apps reach EHR data with patient consent — the plumbing behind Apple Health and patient portals.
- The **ONC Cures Act Final Rule (2020)** requires FHIR R4 APIs, prohibits information blocking, and lets patients access their data through apps for free.
- **Information blocking** — interfering with access, exchange, or use of EHI — can draw ONC penalties of **up to $1 million per violation.**
- **HL7 v2** (pipe-delimited `MSH|^~\&|` text) runs internal hospital messaging (ADT, labs, orders); **FHIR** (JSON/XML over REST) runs patient-facing and cross-organization exchange — and both operate at the same time.
- **CDS Hooks** is a FHIR-based standard that delivers clinical decision support alerts directly into the EHR workflow.

---

Next: **C8.2 HIEs, Carequality & CommonWell — Networks for Health Data** →

*Part of Module 8: Interoperability & Data Exchange.*


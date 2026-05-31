---
title: "Patient Registration: Required Data Elements & Insurance Verification"
sidebar_label: "C4.2 Patient Registration: Required Data Elements & Insurance Verification"
sidebar_position: 3
description: "Registration collects the demographic, financial, and clinical information needed for care, billing, and communication"
tags: ["registration", "insurance-verification", "eligibility", "demographics"]
---

# Patient Registration: Required Data Elements & Insurance Verification

**Duration:** 45 min · **Level:** Intermediate · **Module:** 4. Patient Registration & Data Management · **Focus:** `registration`, `insurance-verification`, `eligibility`, `demographics`

Registration is where the entire patient record begins, and where the most expensive mistakes are made. The demographic, financial, and clinical information collected at the front desk feeds care delivery, billing, and communication all at once. Get it wrong and the consequences ripple outward: claim denials, delayed care, and HIPAA violations. The CEHRS exam treats registration as a foundational revenue-cycle skill, and it expects you to know not just *what* fields are collected but *why* each one exists and how it is validated.

## The required data elements

Registration starts with demographics, and each field has a specific purpose. The **full legal name** must match a government-issued ID — this is the anchor that ties the encounter to the right identity and, ultimately, to the right MPI record. Alongside the name, registration captures **date of birth, address, phone, and sex.** **Race and ethnicity** are collected specifically because they are required for quality reporting. The **Social Security number** is optional but used to strengthen identity matching, and an **emergency contact** rounds out the core set.

A useful framing for the exam: every demographic field serves at least one downstream system. Name and DOB serve identity, address and phone serve communication, sex and race/ethnicity serve clinical care and quality reporting, and the SSN serves matching. If a field seems pointless, you have not yet found the system that depends on it.

## Verifying insurance before the patient is seen

Demographics establish *who* the patient is; insurance verification establishes *who pays.* Before a service is delivered, registration staff confirm four things: that **coverage is active**, that the **patient is eligible**, that the **service is covered**, and that any required **authorization number** has been obtained. This is done through **payer portals** — Availity being the common example named in the curriculum — which allow real-time eligibility checks against the insurer's records.

The mechanism behind those checks is the **270/271 HIPAA transaction**, the standardized electronic eligibility exchange: the provider sends a 270 inquiry and the payer returns a 271 response. This is called **real-time eligibility (RTE)**, and it is standard practice precisely because it catches coverage problems at the front end — before they become claim denials at the back end. The exam tends to reward this front-end logic: it is far cheaper to prevent a denial at registration than to fight one after the claim is submitted.

## Prior authorization and the cost of missing it

Some services require more than verified coverage — they require the payer's **prior authorization**, a written approval obtained *before* the service is delivered. Prior authorization typically applies to certain procedures, specialist visits, and inpatient admissions. The resulting **authorization number is documented in registration**, where it later attaches to the claim. The timing rule is what the exam tests: authorization must be obtained *before* service. Perform the procedure first and seek approval afterward, and the payer can deny the claim outright.

## Who is uninsured, and who is responsible for the bill

Two registration concepts round out the financial picture and both generate exam questions.

First, **self-pay and charity care.** Uninsured patients must be **screened for financial assistance eligibility.** This is not optional goodwill — the **Affordable Care Act requires non-profit hospitals to maintain charity care policies**, and screening typically uses **Federal Poverty Level (FPL) guidelines** to determine who qualifies. A patient who walks in without insurance should leave registration having been evaluated for assistance.

Second, the **guarantor-versus-subscriber distinction**, which the exam likes because the terms are easy to confuse. The **guarantor** is the person responsible for the bill — often the patient. The **subscriber** is the person who holds the insurance policy — which may be a *spouse or parent* rather than the patient. A child seen at a clinic might be the patient, with a parent as both guarantor and subscriber. Both roles must be registered correctly, because the subscriber drives the claim and the guarantor drives collections. Memory hook: *the subscriber owns the policy; the guarantor owns the bill.*

## Putting it into practice

Build a registration checklist you can run from memory, because that is precisely the muscle the exam measures.

1. List the required demographic fields and, beside each, the system it serves (name → identity, address → communication, race/ethnicity → quality reporting, SSN → matching).
2. Write the four insurance-verification checks — coverage active, patient eligible, service covered, authorization obtained — as a single line.
3. Note that real-time eligibility runs on the 270/271 transaction and that front-end verification prevents back-end denials.
4. Define guarantor versus subscriber in one sentence each, then invent a family scenario (child patient, parent subscriber) and assign the roles correctly.
5. Self-test: given an uninsured patient, state the required next step (screen for charity care using FPL guidelines) without prompting.

## Key takeaways

- Required demographics include full legal name (matched to government ID), DOB, address, phone, sex, race/ethnicity (for quality reporting), optional SSN, and emergency contact — each field feeds a downstream system.
- Insurance verification confirms coverage is active, the patient is eligible, the service is covered, and any authorization is obtained, typically via payer portals like Availity.
- Real-time eligibility uses the 270/271 HIPAA transaction to catch coverage problems at the front end and prevent claim denials before they happen.
- Prior authorization must be obtained *before* service; the authorization number is documented in registration.
- The subscriber holds the insurance policy (possibly a spouse or parent); the guarantor is responsible for the bill (often the patient) — both must be registered correctly, and uninsured patients must be screened for charity care under ACA rules using FPL guidelines.

---

← Previous: **C4.1 The Master Patient Index — The Foundation of Identity Management** · Next: **C4.3 Consent, Advance Directives & Patient Rights Documentation** →

*Part of Module 4: Patient Registration & Data Management.*


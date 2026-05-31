---
title: "Data Quality: Accuracy, Completeness & Integrity"
sidebar_label: "C4.4 Data Quality: Accuracy, Completeness & Integrity"
sidebar_position: 5
description: "Poor data quality in health records has direct consequences: wrong medications, missed diagnoses, claim denials, and inaccurate quality reporting"
tags: ["data-quality", "AHIMA", "audit-trails", "integrity", "governance"]
---

# Data Quality: Accuracy, Completeness & Integrity

**Duration:** 40 min · **Level:** Intermediate · **Module:** 4. Patient Registration & Data Management · **Focus:** `data-quality`, `AHIMA`, `audit-trails`, `integrity`, `governance`

Poor data quality in a health record is not an abstract problem — it has a body count. A transposed digit, a missing allergy, or a duplicate diagnosis can lead to the wrong medication, a missed diagnosis, a denied claim, or a quality report built on bad numbers. Because the stakes are this concrete, AHIMA defines a formal set of data quality characteristics, and the CEHRS exam expects you to apply them to every data element you touch. This lesson covers what good data looks like, how errors creep in, and — critically — what you are and are not allowed to do when you find one.

## The six AHIMA data quality characteristics

The backbone of this entire topic is the **AHIMA data quality model**, six characteristics that together define a trustworthy data element. Memorize them, because the exam tests them by name and by example:

- **Accuracy** — the data is correct.
- **Completeness** — all required elements are present.
- **Consistency** — no contradictions exist within a record or across records.
- **Timeliness** — the data is documented when required.
- **Relevance** — the information is appropriate to the patient and encounter.
- **Granularity** — there is sufficient detail.

A handy way to lock these in: each characteristic answers a different question. Accuracy asks *is it right?*, completeness asks *is anything missing?*, consistency asks *does it contradict itself?*, timeliness asks *was it entered on time?*, relevance asks *does it belong here?*, and granularity asks *is it detailed enough?* When the exam describes a flawed data element, your job is to name which of the six it violates.

## What bad data actually looks like

The curriculum gives a concrete catalog of **common data quality errors**, and recognizing them is exactly the skill the exam measures. The list includes **wrong patient (an overlay)**, **transposed dates** — the memorable example being a birth year of 1982 entered as 1928 — **gender mismatch**, **incorrect medication dosage**, **missing allergies**, and **duplicate diagnoses with contradictory laterality** (for instance, the same condition coded once as left and once as right). Each of these maps back to a violated characteristic: a transposed date fails accuracy, a missing allergy fails completeness, contradictory laterality fails consistency. Practicing that mapping — *error to characteristic* — is the most efficient way to study this material.

## The systems that keep data honest

Three structures work behind the scenes to protect data quality, and the CEHRS specialist interacts with all of them.

**Data governance** is the formal program that defines *who owns* data quality, *how errors are reported*, and *how corrections are made.* It is the organizational chart of accountability for data. When you find a quality issue, governance tells you where it goes — and the curriculum is explicit that CEHRS specialists **report quality issues to the data governance committee or HIM director.**

**Audit trails** are the EHR's permanent memory. The system **logs every access, entry, edit, and deletion** with a **user ID, timestamp, and action.** The defining property is that **audit trails cannot be edited or deleted** — which is exactly what makes them reliable for **investigating errors and HIPAA breaches.** If you remember one fact about audit trails for the exam, remember their immutability.

**Data integrity testing** keeps the EHR aligned with the systems that feed it. Through **regular comparison between EHR data and source systems** — lab, pharmacy, registration — the organization detects **interface failures** before they corrupt the record. CEHRS staff may **run reconciliation reports** as part of this work, surfacing mismatches where an interface dropped or garbled data in transit.

## The rule that matters most: report, do not fix

The single most important exam point in this lesson concerns *behavior*, not definitions. When a CEHRS specialist identifies a quality issue — a wrong MRN on a lab result, a duplicate patient record — the required action is to **follow the facility's data correction policy**, and explicitly **not to attempt to fix it unilaterally.** This is **defective data reporting.** The temptation is understandable: you can see the error, you know it's wrong, and fixing it seems helpful. But unilateral correction can destroy the audit trail, mask a deeper interface problem, or introduce a new error of its own. The discipline the exam rewards is restraint: *identify, report, and route through policy — do not freelance.* This single principle ties data quality back to governance and audit trails, because both exist to make controlled correction possible.

## Putting it into practice

Drill the two skills the exam actually scores: classifying errors and choosing the correct response.

1. Write the six AHIMA characteristics on a card and, beside each, the one-word question it answers (right? missing? contradictory? on time? relevant? detailed?).
2. Take the common-error list — transposed date, missing allergy, gender mismatch, contradictory laterality, wrong patient — and map each to the characteristic it violates.
3. Memorize the audit-trail fact: it logs user ID, timestamp, and action, and it cannot be edited or deleted.
4. For data integrity, note that reconciliation reports compare EHR data against lab, pharmacy, and registration source systems to catch interface failures.
5. Self-test with a scenario: you spot a wrong MRN on a result. Write your action in one line — report through the facility's data correction policy; do not fix it yourself.

## Key takeaways

- AHIMA's six data quality characteristics are accuracy, completeness, consistency, timeliness, relevance, and granularity — each answers a distinct question about a data element.
- Common errors include wrong patient (overlay), transposed dates, gender mismatch, incorrect dosage, missing allergies, and duplicate diagnoses with contradictory laterality; map each error to the characteristic it violates.
- Data governance defines ownership, reporting, and correction of quality issues; CEHRS staff report problems to the data governance committee or HIM director.
- Audit trails log every access, entry, edit, and deletion with user ID, timestamp, and action and cannot be edited or deleted — essential for investigating errors and breaches.
- When you find a quality issue, follow the facility's data correction policy and report it — do not attempt to fix it unilaterally; data integrity testing uses reconciliation reports against lab, pharmacy, and registration systems to catch interface failures.

---

← Previous: **C4.3 Consent, Advance Directives & Patient Rights Documentation**

*Part of Module 4: Patient Registration & Data Management.*


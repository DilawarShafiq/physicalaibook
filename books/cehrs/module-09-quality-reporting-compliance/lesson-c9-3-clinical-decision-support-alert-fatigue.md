---
title: "Clinical Decision Support & Alert Fatigue"
sidebar_label: "C9.3 Clinical Decision Support & Alert Fatigue"
sidebar_position: 4
description: "Clinical Decision Support (CDS) tools — built into EHR systems — present real-time alerts, reminders, and recommendations to clinicians"
tags: ["CDS", "alert-fatigue", "clinical-decision-support", "EHR-optimization"]
---

# Clinical Decision Support & Alert Fatigue

**Duration:** 45 min · **Level:** Advanced · **Module:** 9. Quality Reporting & Compliance · **Focus:** `CDS`, `alert-fatigue`, `clinical-decision-support`, `EHR-optimization`

Clinical Decision Support is the EHR trying to keep clinicians safe in real time. As a physician writes an order, the system checks it against the patient's medications, allergies, and care gaps, then surfaces an alert, reminder, or recommendation. In principle this catches dangerous mistakes before they reach the patient. In practice, clinicians are buried under so many of these alerts that they reflexively click past them — a problem with a name, alert fatigue, that is itself a patient safety concern. The CEHRS specialist frequently sits at the center of this tension, managing alert configuration and monitoring how often alerts fire and how often they are ignored. This lesson covers what CDS is, why it backfires, and how it is measured.

## What CDS looks like inside the EHR

**Clinical Decision Support (CDS)** tools are **built into the EHR** and present **real-time alerts, reminders, and recommendations** to clinicians. The exam expects you to recognize the common types:

- **Drug-drug interaction alerts** — flagging a dangerous combination of medications.
- **Drug-allergy alerts** — catching an order for something the patient is allergic to.
- **Duplicate order checks** — preventing the same test or medication from being ordered twice.
- **Preventive care reminders** — such as "flu shot due."
- **Quality measure reminders** — nudging the clinician toward actions that satisfy HEDIS or MIPS measures.
- **Sepsis screening alerts** — firing when a patient's data suggests early sepsis.

The throughline is that CDS turns the EHR from a passive record into an active safety net that watches orders as they happen.

## Alert fatigue: the safety net that gets ignored

The central problem is **alert fatigue** — clinicians receive **so many EHR alerts that they begin ignoring them.** This is not a fringe issue: studies show **49% to 96% of medication alerts are overridden.** When the override rate is that high, the alert system stops functioning as a safety net, because the one critical warning gets dismissed with the same reflexive click as the ninety trivial ones before it. That is why **alert fatigue is itself a patient safety concern** — too many alerts can make care *less* safe, not more.

The memory hook: *an alert that is always overridden protects no one.* The goal of alert management is not to maximize the number of alerts but to make sure the ones that fire actually change behavior.

## Managing and tuning alerts

This is where the CEHRS or **clinical informatics** role becomes hands-on. Staff **review alert firing rates and override rates** to judge whether each alert is earning its place. The specific threshold to remember: **alerts overridden more than 90% of the time should be reviewed for appropriateness and potentially retired.** An alert dismissed nine times out of ten is mostly noise, and removing it can actually *improve* safety by reducing the fatigue that causes clinicians to ignore the alerts that matter.

A related design lever is **how disruptive an alert is.** **Best practice advisories (BPAs)** come in two flavors. **Non-interruptive** CDS appears in the sidebar **without stopping the workflow** — the clinician can take it or leave it. **Interruptive** CDS **requires the clinician to act before proceeding.** The exam principle is that **interruptive alerts are reserved for the highest-risk situations**, precisely because they are costly to the workflow; using them for low-stakes reminders is how you manufacture alert fatigue.

## The regulatory and measurement layer

CDS is not just good practice — it is partly mandated. Under **Meaningful Use / Promoting Interoperability**, an organization must **implement 5 CDS interventions targeting 5 high-priority conditions, plus one drug-drug interaction alert and one drug-allergy alert.** The same program requires documenting **clinical quality measure (CQM) reporting.** If the exam asks how many CDS interventions the program requires, the answer set is *five interventions for five high-priority conditions, plus the two specific medication alerts.*

Finally, all of this can be measured because the EHR keeps the receipts. **Audit logs capture every alert shown, whether it was accepted or overridden, and the reason given for the override.** The CEHRS specialist can **extract this data for quality improvement** — which is exactly the data that feeds the firing-rate and override-rate reviews described above. The loop closes: alerts generate audit data, audit data reveals which alerts are noise, and that evidence justifies retiring the ones overridden more than 90% of the time.

## Putting it into practice

Build a CDS reference sheet and rehearse the tuning logic with a realistic scenario.

1. List the six CDS types from memory: drug-drug, drug-allergy, duplicate order, preventive reminder, quality measure reminder, sepsis screening. Beside each, note in a few words what it prevents or prompts.
2. Write the alert-fatigue facts: 49-96% of medication alerts overridden; alert fatigue is a patient safety concern; review and potentially retire alerts overridden more than 90% of the time.
3. Contrast **non-interruptive** (sidebar, workflow continues) versus **interruptive** (must act to proceed), and write the rule that interruptive alerts are reserved for the highest-risk situations.
4. State the Meaningful Use / PI requirement (5 interventions, 5 high-priority conditions, plus one drug-drug and one drug-allergy alert), then explain how audit logs supply the override data that drives alert tuning. If you can walk that full loop, you own this lesson.

## Key takeaways

- **CDS** is built into the EHR and delivers real-time alerts, reminders, and recommendations: drug-drug, drug-allergy, duplicate-order, preventive-care, quality-measure, and sepsis-screening alerts.
- **Alert fatigue** — driven by an override rate of **49-96% for medication alerts** — is a **patient safety concern**, because over-alerting causes clinicians to ignore even critical warnings.
- Staff review firing and override rates; alerts overridden **more than 90% of the time** should be reviewed and **potentially retired.**
- **Non-interruptive** BPAs sit in the sidebar without stopping work; **interruptive** alerts demand action and are **reserved for the highest-risk situations.**
- **Meaningful Use / Promoting Interoperability** requires **5 CDS interventions for 5 high-priority conditions, plus one drug-drug and one drug-allergy alert**, with CQM reporting documented.
- **Audit logs** record every alert shown, its acceptance or override, and the override reason — data CEHRS extracts to drive quality improvement and alert tuning.

---

← Previous: **C9.2 Record Retention, Destruction & Legal Holds**

*Part of Module 9: Quality Reporting & Compliance.*


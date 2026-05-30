---
title: "The Master Patient Index — The Foundation of Identity Management"
sidebar_label: "C4.1 The Master Patient Index — The Foundation of Identity Management"
sidebar_position: 2
description: "The Master Patient Index (MPI) is the database that uniquely identifies every patient who has ever received care at an organization"
tags: ["MPI", "EMPI", "duplicates", "patient-identity", "data-quality"]
---

# The Master Patient Index — The Foundation of Identity Management

**Duration:** 50 min · **Level:** Intermediate · **Module:** 4. Patient Registration & Data Management · **Focus:** `MPI`, `EMPI`, `duplicates`, `patient-identity`, `data-quality`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- MPI purpose
- Enterprise MPI (EMPI)
- Duplicate records
- Overlay records
- Duplicate detection
:::

## Why this matters

The Master Patient Index (MPI) is the database that uniquely identifies every patient who has ever received care at an organization.

## Overview

The Master Patient Index (MPI) is the database that uniquely identifies every patient who has ever received care at an organization. It is the most critical database in the health system — errors in the MPI can result in wrong-patient medical errors, billing fraud, and HIPAA violations. CEHRS specialists are MPI custodians.

## Key concepts

:::note Key idea

MPI purpose: assigns a unique Medical Record Number (MRN) to each patient; links all encounters across the organization to the correct patient identity

:::

- Enterprise MPI (EMPI): extends the MPI across multiple facilities within a health system; critical when patients move between hospitals, clinics, and ambulatory centers within the same network
- Duplicate records: when the same patient is registered as two separate people (different MRNs); risk: clinical data split across two records; providers see incomplete medication lists, allergies, history
- Overlay records: when two patients' records are merged incorrectly — most dangerous MPI error; can result in wrong patient receiving wrong treatment; must be reported as a patient safety event
- Duplicate detection: algorithms compare name, DOB, SSN, address, phone using probabilistic matching; CEHRS staff review potential duplicates flagged by the system
- EMPI maintenance: regular MPI cleanup (deduplication) projects; most large health systems have 3-8% duplicate rate; industry standard target is &lt;2%

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about MPI purpose?</summary>

assigns a unique Medical Record Number (MRN) to each patient; links all encounters across the organization to the correct patient identity

</details>

<details>
<summary>Q2. What do you know about Enterprise MPI (EMPI)?</summary>

extends the MPI across multiple facilities within a health system; critical when patients move between hospitals, clinics, and ambulatory centers within the same network

</details>

<details>
<summary>Q3. What do you know about Duplicate records?</summary>

when the same patient is registered as two separate people (different MRNs); risk: clinical data split across two records; providers see incomplete medication lists, allergies, history

</details>

<details>
<summary>Q4. What do you know about Overlay records?</summary>

when two patients' records are merged incorrectly — most dangerous MPI error; can result in wrong patient receiving wrong treatment; must be reported as a patient safety event

</details>

<details>
<summary>Q5. What do you know about Duplicate detection?</summary>

algorithms compare name, DOB, SSN, address, phone using probabilistic matching; CEHRS staff review potential duplicates flagged by the system

</details>

---

Next: **C4.2 Patient Registration: Required Data Elements & Insurance Verification** →

*Part of Module 4: Patient Registration & Data Management.*


---
title: "Multi-Tenant HIPAA SaaS Architecture"
sidebar_label: "H7.1 Multi-Tenant HIPAA SaaS Architecture"
sidebar_position: 2
description: "A multi-tenant SaaS serving multiple healthcare practices must provide complete data isolation between tenants while sharing infrastructure efficiently"
tags: ["multi-tenant", "SaaS", "HIPAA", "data-isolation", "architecture"]
---

# Multi-Tenant HIPAA SaaS Architecture

**Duration:** 55 min · **Level:** Advanced · **Module:** 7. Deploying at Scale: Multi-Tenant SaaS · **Focus:** `multi-tenant`, `SaaS`, `HIPAA`, `data-isolation`, `architecture`

:::info Learning objectives

By the end of this lesson you will be able to explain and apply:

- Tenant isolation models
- Tenant data models
- EHR integration patterns
- Payer credential management
- Agent customization per tenant
:::

## Why this matters

A multi-tenant SaaS serving multiple healthcare practices must provide complete data isolation between tenants while sharing infrastructure efficiently.

## Overview

A multi-tenant SaaS serving multiple healthcare practices must provide complete data isolation between tenants while sharing infrastructure efficiently. HIPAA requires that data for Practice A can never be accessible to Practice B. The architectural pattern that achieves both is schema-per-tenant or database-per-tenant with shared application layer.

## Key concepts

:::note Key idea

Tenant isolation models: shared database (lowest cost, highest breach risk), schema-per-tenant (moderate isolation, moderate cost), database-per-tenant (highest isolation, higher cost); for HIPAA-sensitive healthcare data, database-per-tenant or at minimum schema-per-tenant required

:::

- Tenant data models: each tenant (practice) has: practice profile (NPI, Tax ID, payer contracts), patient roster (MRN mapping), EHR integration credentials, payer portal credentials, agent configuration, billing/subscription data
- EHR integration patterns: each tenant practice uses a different EHR (Epic, Cerner, Athena, eCW, etc.); integration layer must support multiple EHR connectors; FHIR R4 APIs enable standardized connection to certified EHRs; custom connectors needed for legacy systems
- Payer credential management: each practice has different payer contracts and portal credentials; credentials stored encrypted in per-tenant vault (HashiCorp Vault or AWS Secrets Manager per tenant); never stored in shared database
- Agent customization per tenant: specialty-specific coding rules (cardiology vs primary care vs orthopedics), payer mix (Medicare-heavy vs commercial-heavy), claim dollar thresholds, communication preferences; stored as per-tenant agent configuration
- Compliance scope: each tenant's BAA with Autosapien covers their patient data; Autosapien's BAA with AWS/Anthropic covers the infrastructure; layered BAA chain must be documented and maintained

## Check your understanding

Try to recall each answer before expanding it.

<details>
<summary>Q1. What do you know about Tenant isolation models?</summary>

shared database (lowest cost, highest breach risk), schema-per-tenant (moderate isolation, moderate cost), database-per-tenant (highest isolation, higher cost); for HIPAA-sensitive healthcare data, database-per-tenant or at minimum schema-per-tenant required

</details>

<details>
<summary>Q2. What do you know about Tenant data models?</summary>

each tenant (practice) has: practice profile (NPI, Tax ID, payer contracts), patient roster (MRN mapping), EHR integration credentials, payer portal credentials, agent configuration, billing/subscription data

</details>

<details>
<summary>Q3. What do you know about EHR integration patterns?</summary>

each tenant practice uses a different EHR (Epic, Cerner, Athena, eCW, etc.); integration layer must support multiple EHR connectors; FHIR R4 APIs enable standardized connection to certified EHRs; custom connectors needed for legacy systems

</details>

<details>
<summary>Q4. What do you know about Payer credential management?</summary>

each practice has different payer contracts and portal credentials; credentials stored encrypted in per-tenant vault (HashiCorp Vault or AWS Secrets Manager per tenant); never stored in shared database

</details>

<details>
<summary>Q5. What do you know about Agent customization per tenant?</summary>

specialty-specific coding rules (cardiology vs primary care vs orthopedics), payer mix (Medicare-heavy vs commercial-heavy), claim dollar thresholds, communication preferences; stored as per-tenant agent configuration

</details>

---

Next: **H7.2 Practice Onboarding Automation: Live in 24 Hours** →

*Part of Module 7: Deploying at Scale: Multi-Tenant SaaS.*


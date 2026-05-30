---
title: "Multi-Tenant HIPAA SaaS Architecture"
sidebar_label: "H7.1 Multi-Tenant HIPAA SaaS Architecture"
sidebar_position: 2
description: "A multi-tenant SaaS serving multiple healthcare practices must provide complete data isolation between tenants while sharing infrastructure efficiently"
tags: ["multi-tenant", "SaaS", "HIPAA", "data-isolation", "architecture"]
---

# Multi-Tenant HIPAA SaaS Architecture

**Duration:** 55 min · **Level:** Advanced · **Module:** 7. Deploying at Scale: Multi-Tenant SaaS · **Tags:** `multi-tenant`, `SaaS`, `HIPAA`, `data-isolation`, `architecture`

## Overview

A multi-tenant SaaS serving multiple healthcare practices must provide complete data isolation between tenants while sharing infrastructure efficiently. HIPAA requires that data for Practice A can never be accessible to Practice B. The architectural pattern that achieves both is schema-per-tenant or database-per-tenant with shared application layer.

## Key Insights

- Tenant isolation models: shared database (lowest cost, highest breach risk), schema-per-tenant (moderate isolation, moderate cost), database-per-tenant (highest isolation, higher cost); for HIPAA-sensitive healthcare data, database-per-tenant or at minimum schema-per-tenant required
- Tenant data models: each tenant (practice) has: practice profile (NPI, Tax ID, payer contracts), patient roster (MRN mapping), EHR integration credentials, payer portal credentials, agent configuration, billing/subscription data
- EHR integration patterns: each tenant practice uses a different EHR (Epic, Cerner, Athena, eCW, etc.); integration layer must support multiple EHR connectors; FHIR R4 APIs enable standardized connection to certified EHRs; custom connectors needed for legacy systems
- Payer credential management: each practice has different payer contracts and portal credentials; credentials stored encrypted in per-tenant vault (HashiCorp Vault or AWS Secrets Manager per tenant); never stored in shared database
- Agent customization per tenant: specialty-specific coding rules (cardiology vs primary care vs orthopedics), payer mix (Medicare-heavy vs commercial-heavy), claim dollar thresholds, communication preferences; stored as per-tenant agent configuration
- Compliance scope: each tenant's BAA with Autosapien covers their patient data; Autosapien's BAA with AWS/Anthropic covers the infrastructure; layered BAA chain must be documented and maintained


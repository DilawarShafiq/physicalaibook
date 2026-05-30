---
title: "The US Payer Landscape: Medicare, Medicaid & Commercial Insurance"
sidebar_label: "H1.2 The US Payer Landscape: Medicare, Medicaid & Commercial Insurance"
sidebar_position: 3
description: "Every AI agent you build for healthcare administration operates within a specific payer environment"
tags: ["Medicare", "Medicaid", "commercial-payers", "EDI", "payer-landscape"]
---

# The US Payer Landscape: Medicare, Medicaid & Commercial Insurance

**Duration:** 45 min · **Level:** Foundational · **Module:** 1. The US Healthcare Admin Crisis · **Tags:** `Medicare`, `Medicaid`, `commercial-payers`, `EDI`, `payer-landscape`

## Overview

Every AI agent you build for healthcare administration operates within a specific payer environment. The rules, portals, formats, and quirks vary enormously between Medicare, Medicaid, and the hundreds of commercial payers. Understanding this landscape is prerequisite to building agents that actually work.

## Key Insights

- Medicare: federal program for 65+ and disabled; ~65M beneficiaries; administered by CMS; Medicare Advantage (MA) — private plans contracting with CMS — now covers &gt;50% of Medicare enrollees; MA plans have their own PA requirements beyond CMS
- Medicaid: state-federal program for low-income; each of 50 states has different rules, formularies, and PA requirements; managed Medicaid (MCO) = private plans managing state Medicaid = 3 layers of rules (federal + state + MCO)
- Commercial payers: UnitedHealth (~$90B revenue), Anthem/Elevance (~$60B), Aetna/CVS (~$55B), Cigna (~$45B), Humana (~$36B) — the "Big 5" cover ~190M lives; each has own portal, EDI specs, and PA criteria
- EDI transactions: all payer interactions standardized via HIPAA EDI: 270/271 (eligibility), 278 (PA request), 837P/837I (professional/institutional claims), 835 (remittance), 277 (claim status); knowing the transaction set is essential for agent design
- Payer portal fragmentation: each payer has its own web portal (Availity for multi-payer, NaviMedic, CoverMyMeds for PA, Emdeon/Waystar for claims); most still require manual portal entry for many transactions
- AI agent challenge: agents must handle payer-specific rules that change quarterly; building a maintenance-free universal payer integration layer is the core engineering challenge in healthcare AI


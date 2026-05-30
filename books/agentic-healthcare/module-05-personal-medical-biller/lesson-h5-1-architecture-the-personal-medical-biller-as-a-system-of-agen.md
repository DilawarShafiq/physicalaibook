---
title: "Architecture: The Personal Medical Biller as a System of Agents"
sidebar_label: "H5.1 Architecture: The Personal Medical Biller as a System of Agents"
sidebar_position: 2
description: "A Personal Medical Biller is not a single agent — it is an orchestrated system of specialized sub-agents, each expert in one domain, coordinated by a master orchestrator that maint"
tags: ["agent-architecture", "orchestration", "Personal-Medical-Biller", "Agent-SMITH", "multi-agent"]
---

# Architecture: The Personal Medical Biller as a System of Agents

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. The Personal Medical Biller · **Tags:** `agent-architecture`, `orchestration`, `Personal-Medical-Biller`, `Agent-SMITH`, `multi-agent`

## Overview

A Personal Medical Biller is not a single agent — it is an orchestrated system of specialized sub-agents, each expert in one domain, coordinated by a master orchestrator that maintains the patient's complete billing lifecycle. This architecture follows the panaversity Digital FTE model: a role-oriented AI employee with defined tools, memory, and accountability.

## Key Insights

- Master orchestrator: knows the patient's complete billing history, active claims, open issues, and next actions; routes work to specialized sub-agents; surfaces issues requiring human attention; generates daily briefing for the billing manager
- Sub-agents: Eligibility Agent (runs coverage checks), PA Agent (tracks authorization status), Coding Agent (reviews encounter documentation), Claims Agent (tracks claim lifecycle), Appeals Agent (manages denial resolution), Patient Communication Agent (handles patient billing questions)
- Agent memory architecture: per-patient state stored in structured database (claim ID → status, PA number → expiry date, denial → appeal deadline); vector store for clinical documentation retrieval; conversation memory for ongoing patient interactions
- Tool inventory: EHR read API, payer EDI integration, clearinghouse API, phone/fax automation (Twilio + fax API), email automation, document generation (PDF statement, appeal letter), CRM write (update patient account notes)
- Human escalation protocol: any action with dollar value &gt;$500, any patient complaint, any potential HIPAA violation, any regulatory ambiguity → escalate to human supervisor with full context briefing; target escalation rate &lt;10% of total actions
- Autosapien Agent SMITH integration: SMITH provides the orchestration layer, HIPAA-compliant data handling, audit trail, and multi-tenant deployment infrastructure; sub-agents deployed as SMITH skills


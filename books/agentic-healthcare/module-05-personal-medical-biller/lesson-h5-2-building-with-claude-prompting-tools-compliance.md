---
title: "Building with Claude: Prompting, Tools & Compliance"
sidebar_label: "H5.2 Building with Claude: Prompting, Tools & Compliance"
sidebar_position: 3
description: "Claude (Anthropic) is the recommended LLM for US healthcare agent applications because of its HIPAA Business Associate Agreement availability, strong instruction-following for regu"
tags: ["Claude", "Anthropic", "HIPAA-BAA", "tool-use", "prompt-engineering"]
---

# Building with Claude: Prompting, Tools & Compliance

**Duration:** 65 min · **Level:** Advanced · **Module:** 5. The Personal Medical Biller · **Focus:** `Claude`, `Anthropic`, `HIPAA-BAA`, `tool-use`, `prompt-engineering`

Choosing the model is the first irreversible decision you make in a healthcare agent, because it determines whether you can legally send Protected Health Information at all. For US healthcare applications, Claude from Anthropic is the recommended LLM — not because of any single benchmark, but because of three properties that matter specifically for regulated work: a HIPAA Business Associate Agreement is available, the model follows instructions reliably on rule-bound tasks, and it refuses requests that would mishandle PHI rather than complying unsafely. This lesson is the engineering layer: the concrete patterns that turn that choice into a compliant, production-grade agent.

## Sign the BAA before you build

A Business Associate Agreement is the legal foundation, and it is not something you bolt on later. Anthropic offers BAAs for Claude API customers — sign one *before* any PHI ever reaches the Claude API, not after you have a working prototype full of patient data. The agreement covers first-party API use under Anthropic's HIPAA-ready terms.

Two practical notes from the current Anthropic guidance reinforce the "sign first" rule. Coverage applies to specific products and configurations — the API and eligible Enterprise usage — and explicitly does *not* extend to consumer plans or the Workbench/Console playground, so never paste real PHI into a console to "test something quickly." And not every API feature is in scope; check Anthropic's implementation guide for the eligible-feature list before you depend on a feature in a PHI path. The order is always: legal agreement, then architecture, then PHI.

## The system prompt is a compliance document

For a healthcare agent, the system prompt is not flavor text — it is where you encode the rules the agent must never break. A healthcare system prompt must state, explicitly and unambiguously:

- **Role and permitted actions** — what this agent is allowed to do, and by implication what it is not.
- **PHI handling instructions** — most importantly, never output PHI in plaintext logs.
- **Required disclaimers** — that this is a billing system, not a medical advice system.
- **Escalation triggers** — the conditions that hand control to a human.
- **Compliance constraints** — the regulatory boundaries the agent operates within.

Write these as direct, enumerated rules. Claude's strength on instruction-following is only useful if the instructions are present and precise; an agent cannot honor a constraint you never wrote down.

## Tools enforce data access — the model does not

The most important security pattern in a Claude healthcare agent is to give it data through *tools*, using the tool-use API, rather than dumping raw data into the prompt. When the agent reaches EHR APIs, payer APIs, and the database through defined tools, the tool itself enforces access control: Claude can only see the data the tool explicitly returns. That is fundamentally more secure than pasting a raw database dump into context and trusting the model to interpret — and ignore — the parts it should not touch.

This connects directly to PHI minimization. Never put more PHI in a prompt than the specific task requires. For context, pass a patient ID plus an encounter ID rather than a full name plus SSN; the identifiers are enough for the agent to reason and act, and the sensitive fields stay out of the prompt. When the task genuinely needs a piece of PHI, retrieve it in the tool *response* at the moment of need — scoped, logged, and minimal — rather than front-loading it into every call.

There is also an output-direction discipline. Use Claude's JSON mode or constrained output for any agent action that feeds a downstream system. A Coding Agent that emits free-form prose cannot have its output safely passed to a claims system, where a malformed code or stray sentence becomes a rejected claim or a compliance problem. Structured output is what makes agent-to-system handoffs safe.

## Make repeated context cheap with prompt caching

Healthcare agents are heavy on stable reference material: payer policy libraries, the ICD-10 codebook, long compliance-laden system prompts. An agent that makes many calls re-sends that same base context every time, and without caching you pay full price for it on every call. Anthropic's prompt caching feature caches those long, unchanging prefixes — system prompts and reference documents — so subsequent calls reuse them at a fraction of the cost and latency.

This matters most in exactly the multi-agent, high-volume design you built in the previous lesson, where sub-agents fire repeatedly against the same payer policies and codebooks. Cache reads are billed at a fraction of the base input price, and prompt caching can substantially cut both cost and latency for long, stable prompts — which is why a healthy cache hit rate, above 80%, should be an explicit operational target, not an afterthought. Caching also extends to tool definitions, so the agent's stable tool inventory rides along in the cached prefix.

## Putting it into practice

Stand up the compliance and engineering scaffolding for one Claude-based sub-agent end to end.

1. Confirm the BAA path first: verify a Claude API BAA is in place (or queued) and that the feature you plan to use is on Anthropic's eligible list. Do not send PHI until this is true.
2. Write the system prompt as five explicit sections: role and permitted actions, PHI handling (no PHI in plaintext logs), disclaimers (not medical advice), escalation triggers, and compliance constraints.
3. Define the agent's data access as tools (EHR read, payer API, database) and confirm each returns only minimal, task-scoped data. Pass patient ID + encounter ID for context; retrieve sensitive PHI only in tool responses.
4. Set structured (JSON) output for every action that feeds a downstream system, and enable prompt caching on the system prompt plus reference docs (payer policies, ICD-10), targeting a cache hit rate above 80%.

## Key takeaways

- Claude is recommended for US healthcare agents for three regulated-work reasons: BAA availability, reliable instruction-following, and safe refusal of PHI-mishandling requests.
- Sign Anthropic's BAA before sending any PHI; coverage is scoped to eligible API/Enterprise use and excludes consumer plans and the console.
- The healthcare system prompt is a compliance document: encode role, PHI handling (no PHI in plaintext logs), disclaimers, escalation triggers, and compliance constraints.
- Give the agent data through the tool-use API so tools enforce access control; minimize PHI in prompts by passing IDs and retrieving sensitive data only in tool responses.
- Use structured JSON output for any action feeding a downstream system; free-text from a coding agent cannot be safely passed to a claims system.
- Enable prompt caching on long system prompts and reference documents (payer policies, ICD-10) and target a cache hit rate above 80% to cut cost and latency.

## References

- **Claude API Documentation — Tool Use and HIPAA** — Anthropic (2024). *docs.anthropic.com*

---

← Previous: **H5.1 Architecture: The Personal Medical Biller as a System of Agents** · Next: **H5.3 Patient-Facing Agent: The Billing Advocate Experience** →

*Part of Module 5: The Personal Medical Biller.*


---
title: "A/R Follow-Up Agents: Working Every Dollar"
sidebar_label: "H4.3 A/R Follow-Up Agents: Working Every Dollar"
sidebar_position: 4
description: "Accounts Receivable (A/R) management is the sustained pursuit of unpaid claims"
tags: ["A/R", "claim-status", "276/277", "follow-up", "aging"]
---

# A/R Follow-Up Agents: Working Every Dollar

**Duration:** 50 min · **Level:** Advanced · **Module:** 4. Claims Submission & Denial Management AI · **Focus:** `A/R`, `claim-status`, `276/277`, `follow-up`, `aging`

Preventing denials and winning appeals matter, but a claim can be clean, accepted, and still unpaid simply because no one followed up on it. **Accounts Receivable (A/R) management** is the sustained, unglamorous pursuit of every unpaid claim until the money arrives or the account is closed for cause. It is work that rewards consistency and punishes fatigue — exactly the profile that breaks human follow-up teams, who triage by the squeakiest wheel and let aging claims slip. An A/R agent works *every* claim systematically: checking status, reworking pended claims, escalating the stuck ones, and knowing when to stop. This lesson builds it.

## Aging buckets define the rhythm

A/R is organized by how old a claim is, and each age band carries a different action. Your agent's core loop is driven by these buckets:

- **0–30 days** — active adjudication; the payer is still working it, so leave it alone.
- **31–60 days** — follow-up begins; the claim is now old enough to question.
- **61–90 days** — escalated follow-up; something is likely stuck.
- **91–120 days** — demand letter or appeal; the claim needs pressure.
- **120+ days** — write-off or external collections consideration; decide whether it is still worth pursuing.

The discipline here is restraint as much as persistence: chasing a claim at day ten wastes effort on a payer that is simply still adjudicating. The agent should let the bucket dictate the action, applying energy when it will actually move the claim and not before.

## Checking status with 276/277

The agent's daily heartbeat is the status check. The standard mechanism is the **HIPAA 276/277 transaction** — a 276 inquiry sent to the payer, a 277 response coming back — which gives electronic, structured claim status. Where electronic status is unavailable, the agent falls back to **Availity or payer portals** for manual lookup. The operating rule from the lesson: check the status of every claim **older than 21 days, daily**, and trigger the next action from the status code returned.

Those **277 status codes** are the agent's decision inputs, and each maps to a specific response:

- **A3** — claim received, pending; wait and recheck.
- **A6** — payer considers the claim complete, payment forthcoming; expect payment, monitor.
- **A7** — payer received incorrect information; rework and resubmit.
- **F2** — more information needed; gather and supply it.

Encode this mapping explicitly. The status code is not a notification to a human queue — it is a branch in the agent's workflow, and most codes resolve to an action the agent can take or stage without a person.

## Learning payer behavior and building work queues

Payers are not uniform, and the agent's effectiveness comes from learning each one's habits. Some payers **systematically pend certain claim types** for manual review — high-dollar claims, particular diagnosis codes, claims from flagged providers. A human team rediscovers these patterns claim by claim; the agent learns them once from historical data and **builds payer-specific work queues** that anticipate the pend instead of reacting to it.

This is the same principle that ran through claim scrubbing and appeals: your own history is the richest source of payer intelligence, and the agent that mines it works smarter than one running on generic rules. A queue that already knows "this payer always pends high-dollar cardiology claims for review" can prepare the supporting documentation before the pend even posts.

## Patient balances and knowing when to stop

Two endpoints close out the A/R lifecycle. After insurance adjudicates, the remaining **patient balance** must be billed. Here the agent generates a personalized statement, sends it through the patient's preferred channel — mail, email, or text — monitors for payment, and **offers a payment plan when the balance exceeds a threshold**. Meeting patients where they are, on their channel, with a realistic plan, collects more than a stack of identical paper statements.

The other endpoint is the **write-off decision**, and getting it right is what keeps the agent efficient. Some claims genuinely cannot be pursued — those past **timely filing limits**, or those in **states with balance billing restrictions** that bar collection. The agent should **flag these for write-off approval** rather than burning follow-up cycles on uncollectable accounts. Knowing when to stop is as valuable as knowing when to push: every hour spent on a truly dead claim is an hour stolen from a recoverable one. Keep the write-off as a *recommendation* to a human approver, not an automatic action, so accountability stays with a person.

## Putting it into practice

Build an A/R follow-up agent driven by aging buckets and status codes.

1. Load your open claims and bucket each by age (0–30, 31–60, 61–90, 91–120, 120+), with the bucket selecting the next action.
2. Implement a daily status check via 276/277 for every claim older than 21 days, parsing the 277 response code.
3. Map each status code to an action — A3 monitor, A6 monitor for payment, A7 rework and resubmit, F2 gather and supply information.
4. Mine historical data to build payer-specific work queues for claim types that are systematically pended.
5. Add the two endpoints: a patient-balance workflow with preferred-channel statements and threshold-triggered payment plans, and a write-off recommender that flags timely-filing and balance-billing-restricted claims for human approval.

## Key takeaways

- A/R is the systematic pursuit of unpaid claims; its payoff comes from consistency, which is where human follow-up teams fail and an agent excels.
- Aging buckets (0–30 through 120+) set the rhythm — apply effort when it will move the claim, and leave actively-adjudicating claims alone.
- The 276/277 transaction is the status heartbeat; check every claim older than 21 days daily and branch on the 277 code (A3, A6, A7, F2).
- Learn payer-specific pend patterns from your own history and build work queues that anticipate manual review rather than react to it.
- Bill patient balances on the patient's preferred channel and offer payment plans above a threshold to lift collection.
- Recommend — never auto-execute — write-offs for timely-filing and balance-billing-restricted claims, so follow-up effort goes only where dollars are recoverable.

---

← Previous: **H4.2 Automated Denial Appeals: Clinical Evidence + Regulatory Citations**

*Part of Module 4: Claims Submission & Denial Management AI.*


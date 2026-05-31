---
title: "Breach Notification: When, Who & How Fast"
sidebar_label: "C6.3 Breach Notification: When, Who & How Fast"
sidebar_position: 4
description: "A HIPAA breach is the unauthorized acquisition, access, use, or disclosure of PHI that compromises its security or privacy"
tags: ["breach-notification", "HITECH", "HHS", "penalties", "incident-response"]
---

# Breach Notification: When, Who & How Fast

**Duration:** 50 min · **Level:** Intermediate · **Module:** 6. HIPAA Security Rule & Access Control · **Focus:** `breach-notification`, `HITECH`, `HHS`, `penalties`, `incident-response`

Safeguards and access controls exist to prevent breaches. The Breach Notification Rule governs what happens when prevention fails. Added by **HITECH in 2009**, it defines a HIPAA breach as the **unauthorized acquisition, access, use, or disclosure of PHI** that compromises its security or privacy — and it sets strict, calendar-driven obligations to notify patients, the federal government, and sometimes the press. The clock that matters most starts on the **day the breach is discovered**, and the exam will test whether you know who must be told, how fast, and what happens to organizations that fail. This lesson walks the breach lifecycle from "did a breach even occur?" through notification and penalties.

## Was it actually a breach? The 4-factor risk assessment

Not every unauthorized disclosure is a reportable breach. The Rule applies to **unsecured (unencrypted) PHI**, and even then a covered entity can avoid notification if it can **demonstrate a low probability that the PHI was compromised.** That demonstration runs through a **four-factor risk assessment**:

1. The **nature and extent** of the PHI involved (a full record vs. a name on a list).
2. **Who** accessed or used the PHI (a vetted clinician vs. an unknown outsider).
3. Whether the PHI was **actually acquired or viewed** (merely exposed vs. genuinely seen).
4. The **extent to which the risk has been mitigated** (recovered, attested-destroyed, etc.).

The logic to remember: the **default presumption is that a breach occurred**, and the four factors are how an organization rebuts that presumption with documentation. Connect this back to the prior lessons — encrypted PHI is *secured*, so the safe harbor means the Rule does not even reach a lost encrypted device.

## Notifying affected individuals

If a breach is confirmed, the first duty is to the people whose data was exposed. Notification to **affected individuals** must occur **without unreasonable delay and no later than 60 days from discovery.** That 60-day figure is the spine of the entire Rule, and the same outer limit recurs for the other recipients, so anchor on it.

The notice cannot be vague. It must include a **description of the breach**, the **types of PHI involved**, the **steps individuals should take** to protect themselves, and **contact information** for follow-up. The phrase "without unreasonable delay" matters: 60 days is a ceiling, not a target — an organization that sits on a known breach for 59 days "because it had time" has likely still violated the standard.

## Notifying HHS and the media

The next two duties depend on a single threshold: **500 individuals.**

- **HHS notification.** For breaches affecting **500 or more individuals**, the covered entity must **notify HHS within 60 days of discovery.** For breaches affecting **fewer than 500 individuals**, the entity instead **logs them and submits an annual report to HHS by March 1 of the following year.** The hook: big breaches go to HHS *immediately* (within 60 days); small breaches are batched into a yearly filing.
- **Media notification.** Breaches affecting **500 or more individuals in a single state or jurisdiction** also require notice to **prominent media outlets** in that state, again **within 60 days.** This is **in addition to** notifying the individuals — it does not replace it. Media notice is the requirement candidates most often forget; tie it to the same 500 / 60-day numbers so it travels with the HHS rule.

A clean way to hold all of this: **everyone urgent gets 60 days.** Individuals always within 60 days; HHS within 60 days if 500+, otherwise by March 1 annually; media within 60 days if 500+ in one jurisdiction.

## The penalty tiers

The teeth behind the Rule are the **HIPAA penalty tiers**, expanded post-HITECH and scaled to culpability — how much the organization knew and whether it fixed the problem:

- **Tier 1 — didn't know:** $100 to $50,000 per violation.
- **Tier 2 — reasonable cause** (knew or should have known, but not willful neglect): $1,000 to $100,000 per violation.
- **Tier 3 — willful neglect, corrected** (in time): $10,000 to $250,000 per violation.
- **Tier 4 — willful neglect, not corrected:** $50,000 to $1.9 million per year.

The pattern to internalize is the gradient: penalties climb as **knowledge and negligence rise**, and the worst outcome is reserved for **willful neglect that the organization failed to fix.** The difference between Tier 3 and Tier 4 is correction — an entity that discovers willful neglect and remedies it lands far better than one that lets it stand. This is why incident response speed is not just operational hygiene; it directly moves an organization down the penalty scale.

## Putting it into practice

Turn the breach lifecycle into a decision flow you can run under exam pressure.

1. **Did a breach occur?** Apply the four factors — nature/extent of PHI, who accessed it, whether it was actually acquired or viewed, and mitigation — remembering the default presumption is *yes*.
2. **Notify individuals:** always, within **60 days**, with the four required contents (description, PHI types, protective steps, contact info).
3. **Count the people.** **500+** → notify HHS within 60 days *and* notify prominent media within 60 days (same state/jurisdiction). **&lt;500** → log it and report to HHS by **March 1** of the next year.
4. **Place it on the penalty ladder:** map a scenario to Tier 1–4 by asking what the entity knew and whether it corrected the problem.
5. Drill the numbers cold: write **60 days, 500 individuals, March 1, and the four Tier ceilings** until you can reproduce them without hesitation.

## Key takeaways

- A **breach** is the unauthorized acquisition, access, use, or disclosure of **unsecured (unencrypted) PHI**, unless a **4-factor risk assessment** shows a low probability of compromise (nature/extent, who accessed, actually acquired/viewed, mitigation).
- **Affected individuals** must be notified **without unreasonable delay, no later than 60 days** from discovery, with a description, the PHI types, protective steps, and contact information.
- **HHS:** breaches of **500+** individuals reported **within 60 days**; breaches of **fewer than 500** logged and reported annually by **March 1** of the following year.
- **Media:** breaches affecting **500+** individuals in one state/jurisdiction require notice to prominent media outlets within 60 days — **in addition to** individual notification.
- The recurring exam anchors are **60 days** and **500 individuals** — nearly every urgent notice shares the 60-day ceiling.
- **Penalty tiers** scale with culpability: Tier 1 (didn't know, $100–$50K) → Tier 4 (willful neglect, uncorrected, $50K–$1.9M/year); the jump to Tier 4 is failing to *correct* willful neglect.

---

← Previous: **C6.2 Role-Based Access Control & Minimum Necessary Access**

*Part of Module 6: HIPAA Security Rule & Access Control.*


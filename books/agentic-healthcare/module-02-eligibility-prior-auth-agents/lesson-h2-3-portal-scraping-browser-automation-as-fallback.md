---
title: "Portal Scraping & Browser Automation as Fallback"
sidebar_label: "H2.3 Portal Scraping & Browser Automation as Fallback"
sidebar_position: 4
description: "Not every payer offers an API"
tags: ["browser-automation", "Playwright", "portal-scraping", "HIPAA-compliance", "fallback"]
---

# Portal Scraping & Browser Automation as Fallback

**Duration:** 50 min · **Level:** Intermediate · **Module:** 2. Eligibility & Prior Auth Agents · **Tags:** `browser-automation`, `Playwright`, `portal-scraping`, `HIPAA-compliance`, `fallback`

## Overview

Not every payer offers an API. For the tail of smaller regional payers, Medicare Advantage plans, and Medicaid MCOs that still require manual portal entry, browser automation is the engineering solution. Done right — with proper HIPAA controls and audit trails — this is legal, effective, and can achieve 60-80% automation rates on portal-only payers.

## Key Insights

- Browser automation tools: Playwright (Microsoft, modern, fast, supports Chrome/Firefox/Safari), Selenium (older, widely used), Puppeteer (Chrome-focused); all can automate repetitive payer portal navigation
- HIPAA compliance for portal automation: each portal session must use facility credentials (not personal), all PHI entered must be logged with audit trail, session cookies stored encrypted, no PHI cached in plain text
- Captcha handling: payer portals increasingly use CAPTCHA and bot detection; solutions: 2captcha/Anti-Captcha services ($0.001/CAPTCHA), rotated residential IPs, human-in-the-loop for initial login with automated session reuse
- Portal change management: payer portals update their UIs without notice; agents must detect failures, escalate to human backup, and generate screenshots for debugging; CSS selector-based automation is fragile — use semantic selectors and visual AI where possible
- AI-powered form filling: Claude or GPT-4o can parse clinical documentation and extract the specific fields required by a portal form; reduces manual field mapping per payer; handles variation in how payers ask for the same information
- Hybrid architecture: EDI/API first → portal automation fallback → human-in-the-loop escalation; SLA targets: EDI transactions &lt;1 second, portal automation 30-120 seconds, human escalation &lt;4 hours


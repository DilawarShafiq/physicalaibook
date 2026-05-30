---
title: "Role-Based Access Control & Minimum Necessary Access"
sidebar_label: "C6.2 Role-Based Access Control & Minimum Necessary Access"
sidebar_position: 3
description: "Every EHR user should have access to exactly the PHI they need to do their job — nothing more"
tags: ["RBAC", "access-control", "audit", "minimum-necessary", "EHR-security"]
---

# Role-Based Access Control & Minimum Necessary Access

**Duration:** 45 min · **Level:** Intermediate · **Module:** 6. HIPAA Security Rule & Access Control · **Tags:** `RBAC`, `access-control`, `audit`, `minimum-necessary`, `EHR-security`

## Overview

Every EHR user should have access to exactly the PHI they need to do their job — nothing more. Role-Based Access Control (RBAC) implements this principle technically. CEHRS specialists help design access roles, process access requests, and investigate inappropriate access incidents.

## Key Insights

- RBAC principle: assign access permissions to roles (ED nurse, billing specialist, physician, HIM coder) not individuals; assign users to roles; access is inherited from the role
- Minimum necessary access: each role should have only the access required for job function; a billing specialist needs demographic and insurance data but not full clinical notes; a hospitalist does not need to see records of patients not on their panel
- Break-the-glass access: emergency override allowing a provider to access a patient record they don't normally have access to; always logged and always reviewed; typically requires attestation of emergency reason
- VIP/celebrity records: high-profile patients require additional access restriction; most EHR systems have a "sensitive patient" flag that restricts access to a specific approved list and generates audit alerts on every access
- Terminated employee access: access must be revoked on or before the last day of employment; HR → IT workflow must be documented in policy; terminated employee who retained access = HIPAA risk
- Audit log review: CEHRS staff or compliance team reviews audit logs for: snooping (accessing records without treatment relationship), bulk downloads, after-hours access from unusual locations, access by departed staff


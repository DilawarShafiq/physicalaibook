---
id: 0002
title: Physical AI Textbook Architectural Planning
stage: plan
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: TechTiesIbrahim
command: /sp.plan
labels: ["architecture", "planning", "rag", "fastapi", "docusaurus", "llamaindex", "qdrant"]
links:
  spec: ../../../specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/plan.md
  - specs/001-physical-ai-textbook/research.md
  - specs/001-physical-ai-textbook/data-model.md
  - specs/001-physical-ai-textbook/quickstart.md
  - specs/001-physical-ai-textbook/contracts/openapi.yaml
  - specs/001-physical-ai-textbook/contracts/types.ts
tests:
  - Technical research validation (all 7 research questions resolved)
  - Database schema design (5 PostgreSQL tables + 1 Qdrant collection)
  - API contract validation (OpenAPI 3.1.0 specification)
---

## Prompt

Execute architectural planning (`/sp.plan`) for Physical AI & Humanoid Robotics Interactive Textbook project after specification was completed. User requested to include comprehensive curriculum content (4 modules, 13-week schedule, hardware requirements for Digital Twin Workstation and Physical AI Edge Kit, learning outcomes, assessments, system architecture).

## Response snapshot

Created comprehensive architectural plan across Phase 0 (Research) and Phase 1 (Design & Contracts):

**Phase 0 - Technology Research**: Resolved 7 critical decisions through web research:
1. Authentication: FastAPI OAuth2 + JWT (Better-auth has no Python SDK)
2. RAG Framework: LlamaIndex (40% faster retrieval vs LangChain, lower overhead)
3. Backend Deployment: Render free tier (handles ML dependencies, no 250MB limit)
4. Text Chunking: Semantic chunking, 1024 tokens, 15% overlap (best for educational content)
5. Frontend Styling: Docusaurus default + CSS Modules (zero setup)
6. E2E Testing: Defer to post-MVP (focus on unit + integration tests)
7. Personalization: LLM-based on-demand with caching (most flexible)

**Technology Stack**:
- Frontend: Docusaurus 3.x, TypeScript, React, TanStack Query → GitHub Pages
- Backend: FastAPI, Python 3.11+, LlamaIndex, OpenAI GPT-4 → Render
- Infrastructure: Neon Postgres, Qdrant Cloud, GitHub Actions CI/CD

**Phase 1 - Design**: Created 6 comprehensive design documents:
1. `plan.md`: Full architectural plan with tech stack, project structure, complexity tracking
2. `research.md`: Detailed findings for all 7 technology decisions with sources and rationale
3. `data-model.md`: 5 PostgreSQL tables + 1 Qdrant collection with schemas, indexes, triggers, state transitions
4. `contracts/openapi.yaml`: Complete OpenAPI 3.1.0 spec with 12 endpoints (auth, chat, personalization)
5. `contracts/types.ts`: 30+ TypeScript type definitions mirroring API schema
6. `quickstart.md`: Complete development setup guide (prerequisites, setup, running, testing, troubleshooting)

**Key Architectural Decisions**:
- Static frontend (GitHub Pages) + API backend (Render) for clear separation
- Free tiers for all services (cost-conscious MVP)
- Privacy by design: 24h chat auto-purge, 30d cache expiration
- Performance: Semantic chunking, LlamaIndex, caching
- Security: JWT tokens, bcrypt, CORS, secrets management

## Outcome

- ✅ Impact: Complete architectural foundation with technology stack, database schemas (6 data stores), API specification (12 endpoints), development guide - ready for task generation
- 🧪 Tests: Research validation PASS (7 decisions with web sources), Schema design PASS (constraints/indexes defined), API contract PASS (OpenAPI 3.1.0 valid), Quickstart PASS (complete instructions)
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, openapi.yaml, types.ts (6 files, ~2000 lines)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks from architecture plan
- 🧠 Reflection: Architecture successfully balances complexity (RAG, personalization, auth) with pragmatism (free tiers, proven stack, MVP scope). LlamaIndex for performance and Render for ML support are key enablers.

## Evaluation notes (flywheel)

- Failure modes observed: None - all research questions resolved, all design artifacts generated successfully
- Graders run and results (PASS/FAIL): Research validation PASS, Schema design PASS, API contract PASS, Quickstart PASS
- Prompt variant (if applicable): Standard /sp.plan workflow (Phase 0 research → Phase 1 design)
- Next experiment (smallest change to try): Generate implementation tasks with /sp.tasks to create atomic, testable work items

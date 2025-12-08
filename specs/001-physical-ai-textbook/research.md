# Technology Research & Decisions

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Date**: 2025-12-08
**Phase**: Phase 0 - Research

## Overview

This document captures research findings and technology decisions for implementing the interactive textbook platform. All NEEDS CLARIFICATION items from plan.md are resolved with rationale and alternatives considered.

## Research Findings & Decisions

### 1. Authentication Solution (Better-auth Alternative)

**Research Question**: Does better-auth provide a Python SDK, or do we use REST API directly?

**Finding**: [Better Auth](https://www.better-auth.com/) is a comprehensive authentication framework **for TypeScript only** - there is **no official Python SDK** available as of 2025-12-08. Better Auth is built specifically for TypeScript/JavaScript ecosystems.

**Decision**: **Use FastAPI built-in OAuth2 with JWT tokens**

**Rationale**:
- FastAPI provides robust built-in security primitives via `OAuth2PasswordBearer` for token-based authentication
- Industry-standard approach with extensive documentation and community support
- No additional external service dependency (keeps architecture simpler)
- Full control over user data and authentication logic
- Integrates seamlessly with Neon Postgres for user storage
- Lower operational complexity than integrating external auth service

**Alternatives Considered**:
1. **Auth0** ([guide](https://auth0.com/blog/build-and-secure-fastapi-server-with-auth0/))
   - ✅ Enterprise-grade, managed solution
   - ❌ Adds external dependency and potential costs
   - ❌ Overkill for MVP requirements

2. **fastapi-users** library
   - ✅ Batteries-included authentication for FastAPI  - ❌ Adds framework-specific dependency
   - ❌ May constrain future customization

3. **Custom OAuth2 implementation with JWT**
   - ✅ Full control, no external dependencies
   - ✅ Standard pattern well-documented in FastAPI docs
   - ✅ **SELECTED**: Best balance of control, simplicity, and compatibility

**Implementation Notes**:
- Use `python-jose` for JWT token generation/validation
- Use `passlib` with bcrypt for password hashing  - Session management via JWT tokens (stateless)
- Store user profiles in Neon Postgres
- Collect background info (software/hardware experience) during signup

**Sources**:
- [Better Auth Introduction](https://www.better-auth.com/docs/introduction)
- [FastAPI Security Tutorial](https://fastapi.tiangolo.com/tutorial/security/first-steps/)
- [Authentication and Authorization with FastAPI - Better Stack](https://betterstack.com/community/guides/scaling-python/authentication-fastapi/)

---

### 2. RAG Framework Selection

**Research Question**: Use LangChain, LlamaIndex, or custom RAG implementation?

**Finding**: Based on 2025 benchmarks, **LlamaIndex achieves 40% faster document retrieval speeds than LangChain** and has lower framework overhead (~6ms vs ~10ms for LangChain). LlamaIndex is specifically optimized for data indexing and retrieval use cases, while LangChain excels at complex multi-step AI workflows.

**Decision**: **LlamaIndex for RAG implementation**

**Rationale**:
- **Performance**: 40% faster retrieval, critical for <5 second chatbot response target
- **Lower overhead**: ~6ms framework overhead vs ~10ms LangChain
- **Token efficiency**: ~1.60k tokens vs ~2.40k for LangChain (cost savings)
- **Purpose-built for RAG**: Optimized for search & retrieval use cases
- **Qdrant integration**: Seamless integration with Qdrant vector database
- **Simpler API**: More straightforward for basic RAG patterns
- **Better for this use case**: Our primary need is document retrieval, not complex agent workflows

**Alternatives Considered**:
1. **LangChain** ([comparison](https://latenode.com/blog/langchain-vs-llamaindex-2025-complete-rag-framework-comparison))
   - ✅ More flexible for complex AI pipelines
   - ✅ Better for multi-step reasoning tasks
   - ❌ Higher overhead (~10ms vs ~6ms)
   - ❌ Higher token usage (~2.40k vs ~1.60k)
   - ❌ Overkill for straightforward Q&A retrieval

2. **Custom RAG implementation**
   - ✅ Full control over implementation
   - ❌ Requires significant ML expertise
   - ❌ Error-prone prompt engineering
   - ❌ Longer development time
   - ❌ Missing established best practices

3. **LlamaIndex**
   - ✅ **SELECTED**: Best performance for retrieval use case
   - ✅ Lower overhead and token usage (cost-effective)
   - ✅ Qdrant integration out-of-the-box
   - ✅ Simpler API for RAG patterns

**Implementation Notes**:
- Use LlamaIndex `VectorStoreIndex` with Qdrant backend
- Leverage built-in citation extraction for source tracking
- Integrate OpenAI GPT-4 for chat responses
- Use `text-embedding-3-small` for cost-effective embeddings

**Sources**:
- [LangChain vs LlamaIndex 2025 Comparison](https://latenode.com/blog/langchain-vs-llamaindex-2025-complete-rag-framework-comparison)
- [RAG Frameworks Benchmark](https://research.aimultiple.com/rag-frameworks/)
- [Qdrant RAG Use Cases](https://qdrant.tech/rag/)
- [Vector Stores Comparison 2025](https://www.glukhov.org/post/2025/12/vector-stores-for-rag-comparison/)

---

### 3. Backend Deployment Platform

**Research Question**: Deploy FastAPI to Vercel, Railway, Render, Fly.io, or other platform?

**Finding**: For FastAPI applications with **heavy dependencies** (ML libraries, RAG frameworks), **Render provides full server hosting** without the serverless function size limits that affect Vercel. Vercel's serverless functions have a 250MB unzipped size limit and are best for lightweight endpoints.

**Decision**: **Render (free tier with upgrade path)**

**Rationale**:
- **Handles heavy dependencies**: LlamaIndex, OpenAI SDK, Qdrant client won't hit size limits
- **Full server hosting**: Not constrained by serverless function limitations
- **Free tier**: 750 hours/month web service runtime (sufficient for MVP testing)
- **Managed PostgreSQL**: Can run database alongside API (though using Neon separately)
- **Sleep after inactivity**: Acceptable cold start delay for free tier MVP
- **Upgrade path**: Simple migration to paid tier if needed ($7/month starter)
- **Docker support**: Can containerize application for consistency

**Alternatives Considered**:
1. **Vercel** ([deployment guide](https://medium.com/@aaishamdha/deploying-a-fastapi-service-in-minutes-for-free-vercel-choreo-and-render-0527cd57b75b))
   - ✅ Free tier available
   - ✅ Excellent for lightweight APIs
   - ❌ 250MB serverless function size limit
   - ❌ Heavy dependencies (LlamaIndex, ML libs) likely exceed limit
   - ❌ Designed for Next.js frontend + lightweight Python glue code

2. **Railway**
   - ✅ Simple container deployment
   - ❌ Only $5 free credits per month (very limited)
   - ❌ Usage-based pricing ($20/vCPU) expensive for continuous operation

3. **Render**
   - ✅ **SELECTED**: Full server hosting, no size limits
   - ✅ 750 hours/month free tier (sufficient for MVP)
   - ✅ Can handle ML/RAG dependencies
   - ⚠️ Cold start delay on free tier (acceptable trade-off)

**Implementation Notes**:
- Deploy as Docker container for consistency
- Configure auto-sleep on free tier (acceptable for MVP)
- Set up GitHub Actions for CI/CD
- Use Render environment variables for secrets (OpenAI API key, Neon DB URL)
- Monitor usage to anticipate upgrade needs

**Sources**:
- [FastAPI Deployment Options Comparison 2025](https://www.nandann.com/blog/python-hosting-options-comparison)
- [Deploying FastAPI on Vercel, Choreo, and Render](https://medium.com/@aaishamdha/deploying-a-fastapi-service-in-minutes-for-free-vercel-choreo-and-render-0527cd57b75b)
- [Ultimate Guide to Deploying FastAPI 2025](https://medium.com/@zafarobad/ultimate-guide-to-deploying-next-js-d57ab72f6ba6)

---

### 4. Text Chunking Strategy for RAG

**Research Question**: How to chunk textbook content for optimal retrieval (by section, paragraph, semantic similarity)?

**Finding**: **Semantic chunking has proven to be the most effective strategy** for ensuring coherent information within chunks and outperforming other strategies, though it's slower. **Recursive character splitting is recommended as the default approach** for most text content, balancing speed and quality. Industry best practices recommend **10-20% overlap** with **500-1024 token chunk sizes**.

**Decision**: **Hybrid approach - Semantic chunking for initial indexing, 15% overlap**

**Rationale**:
- **Quality over speed**: Initial indexing is one-time operation, prioritize accuracy
- **Semantic coherence**: Textbook content has natural semantic boundaries (concepts, examples, definitions)
- **NVIDIA testing**: 15% overlap performs best with 1024-token chunks
- **Alignment with content**: Educational content benefits from concept-complete chunks
- **Acceptable trade-off**: Slower indexing acceptable for better retrieval accuracy (impacts RAG quality)
- **Performance gap**: Can create up to 9% difference in recall between strategies

**Alternatives Considered**:
1. **Fixed-size chunking** ([Weaviate guide](https://weaviate.io/blog/chunking-strategies-for-rag))
   - ✅ Simplest approach
   - ✅ Fastest processing
   - ❌ May split concepts mid-sentence
   - ❌ Lower retrieval quality

2. **Recursive character splitting**
   - ✅ Balanced approach (default recommendation)
   - ✅ Respects text boundaries (paragraphs, sentences)
   - ⚠️ Good for most content, but educational content benefits from semantic approach

3. **Semantic chunking** ([Unstructured guide](https://unstructured.io/blog/chunking-for-rag-best-practices))
   - ✅ **SELECTED**: Best accuracy for educational content
   - ✅ Preserves concept completeness
   - ✅ Better retrieval relevance
   - ⚠️ Slower processing (acceptable for one-time indexing)

**Implementation Parameters**:
- **Chunk size**: 1024 tokens (aligns with NVIDIA findings)
- **Overlap**: 15% (~150 tokens)
- **Strategy**: Semantic chunking via LlamaIndex's semantic splitter
- **Metadata**: Include chapter, section, module tags for filtering
- **Boundaries**: Respect curriculum structure (concepts, examples, code blocks)

**Implementation Notes**:
- Use LlamaIndex's `SemanticSplitterNodeParser` for automatic semantic chunking
- Add metadata: `{chapter_id, module_number, section_title, difficulty_level}`
- Test chunk quality with sample queries before full indexing
- Script: `scripts/seed-vector-db.py` to populate Qdrant from markdown content

**Sources**:
- [Best Chunking Strategies for RAG 2025](https://www.firecrawl.dev/blog/best-chunking-strategies-rag-2025)
- [NVIDIA: Finding the Best Chunking Strategy](https://developer.nvidia.com/blog/finding-the-best-chunking-strategy-for-accurate-ai-responses/)
- [Databricks: Ultimate Guide to Chunking Strategies](https://community.databricks.com/t5/technical-blog/the-ultimate-guide-to-chunking-strategies-for-rag-applications/ba-p/113089)
- [Chunking for RAG Best Practices - Unstructured](https://unstructured.io/blog/chunking-for-rag-best-practices)

---

### 5. Frontend Styling Approach

**Research Question**: Use Docusaurus default CSS, CSS Modules, or add Tailwind CSS?

**Decision**: **Docusaurus default theming with CSS Modules for custom components**

**Rationale**:
- **Built-in quality**: Docusaurus 3.x has excellent default theming (dark mode, responsive, accessible)
- **Less complexity**: No additional build configuration or dependencies
- **Faster development**: Focus on content and features, not styling system
- **CSS Modules**: Already supported by Docusaurus for component-specific styles
- **Customization**: Can override Docusaurus theme via `src/css/custom.css`
- **Sufficient for MVP**: Styling not a differentiator for educational content platform

**Alternatives Considered**:
1. **Tailwind CSS**
   - ✅ Utility-first rapid development
   - ❌ Requires additional Docusaurus configuration
   - ❌ Increases bundle size
   - ❌ Overkill for content-focused site

2. **Docusaurus default + CSS Modules**
   - ✅ **SELECTED**: Zero additional setup
   - ✅ Works out-of-the-box
   - ✅ Sufficient customization for components
   - ✅ Smaller bundle size

**Implementation Notes**:
- Customize theme colors via `docusaurus.config.js` theme config
- Use CSS Modules (`ComponentName.module.css`) for custom React components
- Override defaults in `src/css/custom.css` as needed

---

### 6. E2E Testing Requirements

**Research Question**: Is Playwright E2E testing needed for MVP, or sufficient for later phases?

**Decision**: **Defer E2E testing to post-MVP** (rely on unit + integration tests for MVP)

**Rationale**:
- **MVP scope**: Focus on core functionality first (content delivery, chatbot, auth)
- **Coverage via integration tests**: API integration tests cover critical user flows
- **Manual QA acceptable**: For initial launch, manual testing of key flows sufficient
- **Add later**: Introduce Playwright E2E tests in iteration 2 after MVP validation
- **CI/CD complexity**: Reduces initial CI/CD setup time and maintenance

**Implementation Notes**:
- **MVP testing strategy**:
  - Frontend: Jest + React Testing Library for component tests
  - Backend: pytest for unit tests, httpx for API integration tests
  - Manual: Manual QA for complete user flows (signup → chat → personalization)
- **Post-MVP**: Add Playwright for critical flows (auth, chatbot interaction, personalization)

---

### 7. Content Personalization Implementation

**Research Question**: How to adapt content presentation - dynamic rendering, pre-generated variants, or AI-generated summaries?

**Decision**: **AI-generated content adaptation** (LLM-based on-demand personalization)

**Rationale**:
- **Flexibility**: Can adapt any content without pre-generation
- **User experience**: Click "Personalize" button triggers LLM to adjust explanation depth
- **Lower maintenance**: No need to maintain multiple content versions manually
- **Better for learning**: Can provide customized explanations based on actual user background
- **Cost acceptable**: Only triggered on-demand when user clicks personalization button (not automatic)
- **Caching**: Can cache personalized versions per user/chapter to reduce API calls

**Alternatives Considered**:
1. **Pre-generated variants**
   - ✅ Fast delivery (no API calls)
   - ❌ High maintenance burden (3 versions per chapter: beginner/intermediate/advanced)
   - ❌ Limited granularity (can't combine software/hardware backgrounds dynamically)

2. **Dynamic rendering with conditional sections**
   - ✅ Simple implementation
   - ❌ Constrained personalization (show/hide only)
   - ❌ Limited value over no personalization

3. **AI-generated adaptation**
   - ✅ **SELECTED**: Most flexible and valuable to users
   - ✅ True personalization based on background
   - ⚠️ Requires API calls (mitigated by caching)
   - ⚠️ Cost per personalization request (mitigated by on-demand only)

**Implementation Notes**:
- Use OpenAI GPT-4 to rewrite chapter sections based on user background
- Prompt template: "Adapt this {topic} explanation for a user with {software_level} software background and {hardware_level} hardware background"
- Cache personalized content in Neon Postgres: `personalized_content(user_id, chapter_id, content, created_at)`
- Implement rate limiting: Max 5 personalizations per user per day (prevent API abuse)
- Show loading indicator while generating personalized content

**Cost Estimation**:
- Assume ~1000 tokens input (chapter section) + ~1500 tokens output = ~2500 tokens per personalization
- GPT-4 cost: ~$0.10 per 1M tokens = $0.00025 per personalization
- 100 users × 5 chapters each = 500 personalizations = ~$0.125 (negligible cost)

---

## Technology Stack Summary

Based on research findings, the final technology stack is:

### Frontend
- **Framework**: Docusaurus 3.x
- **Language**: TypeScript 5.x, Node.js 20.x LTS
- **Styling**: Docusaurus default theme + CSS Modules
- **State Management**: React Context API, TanStack Query

### Backend
- **Language**: Python 3.11+
- **Framework**: FastAPI 0.109+
- **Authentication**: FastAPI OAuth2 + JWT (python-jose, passlib)
- **RAG Framework**: LlamaIndex
- **AI Models**: OpenAI GPT-4 (chat), text-embedding-3-small (embeddings)

### Infrastructure
- **Frontend Hosting**: GitHub Pages
- **Backend Hosting**: Render (free tier → paid upgrade path)
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud Free Tier
- **CI/CD**: GitHub Actions

### Development
- **Testing**: Jest (frontend), pytest (backend), manual QA for MVP
- **Chunking**: Semantic chunking, 1024 tokens, 15% overlap
- **Personalization**: LLM-based on-demand content adaptation

---

## Next Steps

1. ✅ Research complete - all NEEDS CLARIFICATION resolved
2. **Proceed to Phase 1**: Generate data models, API contracts, and quickstart guide
3. **Update agent context**: Run `.specify/scripts/bash/update-agent-context.sh claude`
4. **Begin implementation planning**: Create tasks.md with `/sp.tasks` command

**Research Status**: ✅ COMPLETE
**Blockers Removed**: All technology decisions finalized
**Ready for**: Phase 1 (Design & Contracts)

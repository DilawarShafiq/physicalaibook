# Tasks: Physical AI & Humanoid Robotics Interactive Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml

**Tests**: Tests are OPTIONAL for this project (not explicitly requested in specification). Focus on implementation first, add tests in future iterations if needed.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story (P1→P2→P3→P4→P5).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

This is a **web application** with separate frontend and backend:
- Frontend (Docusaurus): Repository root - `docs/`, `src/`, `docusaurus.config.js`
- Backend (FastAPI): `backend/app/` for application code, `backend/tests/` for tests

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and basic structure

- [ ] T001 Initialize Docusaurus project at repository root with TypeScript template
- [ ] T002 [P] Initialize FastAPI backend project in backend/ with Python 3.11+ virtual environment
- [ ] T003 [P] Install frontend dependencies (Docusaurus 3.x, React 18, TypeScript 5.x, TanStack Query) via package.json
- [ ] T004 [P] Install backend dependencies (FastAPI, LlamaIndex, OpenAI SDK, Qdrant Client, asyncpg, python-jose, passlib) via backend/requirements.txt
- [ ] T005 [P] Configure TypeScript in tsconfig.json for frontend (strict mode, React JSX)
- [ ] T006 [P] Configure Python linting (black, ruff) and testing (pytest, pytest-asyncio) in backend/pyproject.toml
- [ ] T007 Create .env.example files for both frontend (.env.local.example) and backend (backend/.env.example)
- [ ] T008 [P] Configure CORS in backend/app/main.py to allow requests from localhost:3000 (development)
- [ ] T009 [P] Set up GitHub repository with .gitignore for Node.js and Python projects
- [ ] T010 Create project documentation structure: docs/curriculum/, docs/hardware/, docs/architecture/ directories

**Checkpoint**: Project structure initialized - both frontend and backend have basic scaffolding

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T011 Set up Neon Postgres connection in backend/app/database/postgres.py with asyncpg
- [ ] T012 [P] Set up Qdrant Cloud connection in backend/app/database/qdrant.py with qdrant_client
- [ ] T013 [P] Initialize Alembic for database migrations in backend/app/database/migrations/
- [ ] T014 Create database models foundation: backend/app/models/__init__.py with SQLAlchemy Base
- [ ] T015 [P] Implement JWT token utilities in backend/app/utils/security.py (create_access_token, verify_token, hash_password, verify_password)
- [ ] T016 [P] Create environment configuration loader in backend/app/config.py (load from .env, validate required vars)
- [ ] T017 [P] Implement global error handling middleware in backend/app/main.py (APIException handler, validation errors)
- [ ] T018 Create shared Pydantic base models in backend/app/models/base.py (BaseResponse, ErrorResponse)
- [ ] T019 [P] Set up logging configuration in backend/app/utils/logging_config.py (structured logs, log levels)
- [ ] T020 Create dependency injection helpers in backend/app/dependencies.py (get_db, get_current_user)

### Frontend Foundation

- [ ] T021 Configure Docusaurus in docusaurus.config.js (site metadata, theme config, navbar, footer)
- [ ] T022 [P] Create custom CSS theme in src/css/custom.css (color scheme, typography, responsive breakpoints)
- [ ] T023 [P] Set up API client base in src/services/api.ts (axios/fetch wrapper, base URL config, error handling)
- [ ] T024 [P] Create TypeScript types from contracts in src/types/index.ts (import from specs/001-physical-ai-textbook/contracts/types.ts)
- [ ] T025 Create React Context for authentication in src/contexts/AuthContext.tsx (user state, token management)
- [ ] T026 [P] Create React Context for chat in src/contexts/ChatContext.tsx (conversations, messages state)
- [ ] T027 [P] Create React Context for personalization in src/contexts/PersonalizationContext.tsx (recommendations state)
- [ ] T028 [P] Implement local storage utility in src/services/storage.ts (save/load JWT token, user data)
- [ ] T029 Create custom Docusaurus layout in src/components/Layout/CustomLayout.tsx (integrate auth state, chatbot widget)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Browse Interactive Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Deliver publicly accessible textbook with 4-module curriculum covering Physical AI & Humanoid Robotics. Users can navigate chapters, view hardware requirements, and access content on any device.

**Independent Test**: Publish sample chapters to localhost:3000, navigate between chapters, verify responsive layout on mobile/desktop, confirm page load < 3 seconds.

### Content Creation for User Story 1

- [ ] T030 [P] [US1] Create curriculum introduction page in docs/curriculum/introduction.md (focus & goal, quarter overview covering ROS 2, Gazebo, NVIDIA Isaac)
- [ ] T031 [P] [US1] Create Module 1 structure in docs/curriculum/module-1/ with index.md (ROS 2 Fundamentals)
- [ ] T032 [P] [US1] Write sample content for Module 1: ROS 2 basics chapter in docs/curriculum/module-1/ros2-basics.md (publishers, subscribers, nodes)
- [ ] T033 [P] [US1] Create Module 2 structure in docs/curriculum/module-2/ with index.md (Digital Twin & Gazebo Simulation)
- [ ] T034 [P] [US1] Write sample content for Module 2: Gazebo intro chapter in docs/curriculum/module-2/gazebo-intro.md (simulation concepts)
- [ ] T035 [P] [US1] Create Module 3 structure in docs/curriculum/module-3/ with index.md (AI-Robot Brain & NVIDIA Isaac)
- [ ] T036 [P] [US1] Create Module 4 structure in docs/curriculum/module-4/ with index.md (VLA & LLMs for Robotics)
- [ ] T037 [P] [US1] Create hardware requirements page in docs/hardware/workstation.md (Digital Twin Workstation: GPU, CPU, RAM, OS specs)
- [ ] T038 [P] [US1] Create hardware requirements page in docs/hardware/edge-kit.md (Physical AI Edge Kit: Jetson Orin, RealSense, ReSpeaker specs)
- [ ] T039 [P] [US1] Create system architecture page in docs/architecture/system-overview.md (development-to-deployment pipeline diagram)
- [ ] T040 [P] [US1] Create 13-week schedule page in docs/curriculum/schedule.md (week-by-week breakdown, module alignment)
- [ ] T041 [P] [US1] Create learning outcomes page in docs/curriculum/learning-outcomes.md (skills, concepts, practical capabilities)
- [ ] T042 [P] [US1] Create assessments page in docs/curriculum/assessments.md (graded components, evaluation methods)

### Navigation & UI for User Story 1

- [ ] T043 [US1] Configure sidebar navigation in sidebars.js (4 modules hierarchy, hardware section, architecture section)
- [ ] T044 [P] [US1] Customize homepage in src/pages/index.tsx (course overview, call-to-action, module cards)
- [ ] T045 [P] [US1] Add search functionality config in docusaurus.config.js (Algolia DocSearch or local search)
- [ ] T046 [US1] Test responsive design on mobile (viewport < 768px), tablet (768px-1024px), desktop (>1024px)
- [ ] T047 [US1] Optimize page load performance (code splitting, image optimization, lazy loading) to achieve <3s load time

**Checkpoint**: User Story 1 complete - Textbook content is browsable, responsive, and performant. Can deploy MVP to GitHub Pages.

---

## Phase 4: User Story 2 - Create Account and Share Background (Priority: P2)

**Goal**: Enable user registration and authentication. Collect software/hardware experience levels during signup for personalization. Users can sign in, sign out, and update their profiles.

**Independent Test**: Create new account with email/password/background → Sign out → Sign in with same credentials → Update profile → Verify data persisted in Neon Postgres.

### Backend Database Models for User Story 2

- [ ] T048 [P] [US2] Create User model in backend/app/models/user.py (id, email, hashed_password, software_experience, hardware_experience, timestamps) with SQLAlchemy ORM
- [ ] T049 [US2] Create Alembic migration for users table in backend/app/database/migrations/ (run alembic revision --autogenerate -m "create_users_table")
- [ ] T050 [US2] Apply migration to Neon Postgres (run alembic upgrade head)

### Backend Pydantic Schemas for User Story 2

- [ ] T051 [P] [US2] Create User Pydantic schemas in backend/app/models/user.py (UserCreate, UserUpdate, UserResponse, Token)
- [ ] T052 [P] [US2] Add validation to UserCreate schema (email format, password min length 8, experience enums)

### Backend Services for User Story 2

- [ ] T053 [US2] Implement AuthService in backend/app/services/auth_service.py (create_user, authenticate_user, get_user_by_email methods)
- [ ] T054 [P] [US2] Implement password hashing in AuthService using passlib with bcrypt
- [ ] T055 [P] [US2] Implement JWT token generation in AuthService using python-jose

### Backend API Endpoints for User Story 2

- [ ] T056 [P] [US2] Implement POST /auth/signup endpoint in backend/app/routers/auth.py (create user, return user + access_token)
- [ ] T057 [P] [US2] Implement POST /auth/signin endpoint in backend/app/routers/auth.py (OAuth2 password flow, return access_token)
- [ ] T058 [P] [US2] Implement GET /auth/me endpoint in backend/app/routers/auth.py (return current user, requires Bearer token)
- [ ] T059 [P] [US2] Implement PATCH /auth/profile endpoint in backend/app/routers/auth.py (update software/hardware experience)
- [ ] T060 [US2] Register auth router in backend/app/main.py with /v1/auth prefix

### Frontend Components for User Story 2

- [ ] T061 [P] [US2] Create SignUp component in src/components/Auth/SignUp.tsx (email, password, software experience dropdown, hardware experience dropdown)
- [ ] T062 [P] [US2] Create SignIn component in src/components/Auth/SignIn.tsx (email, password, remember me)
- [ ] T063 [P] [US2] Create ProfileForm component in src/components/Auth/ProfileForm.tsx (edit software/hardware experience)
- [ ] T064 [P] [US2] Create UserMenu component in src/components/Auth/UserMenu.tsx (display user email, profile link, sign out button)

### Frontend Services for User Story 2

- [ ] T065 [US2] Implement auth API calls in src/services/auth.ts (signup, signin, getMe, updateProfile functions)
- [ ] T066 [P] [US2] Integrate auth API with AuthContext in src/contexts/AuthContext.tsx (signup handler, signin handler, signout handler, token refresh)
- [ ] T067 [US2] Implement protected route wrapper in src/components/Auth/ProtectedRoute.tsx (redirect to signin if not authenticated)

### Frontend Integration for User Story 2

- [ ] T068 [US2] Add SignUp/SignIn buttons to navbar in docusaurus.config.js (using custom navbar items)
- [ ] T069 [P] [US2] Add UserMenu to navbar when user is authenticated
- [ ] T070 [US2] Create profile page in src/pages/profile.tsx (display user info, ProfileForm component)
- [ ] T071 [US2] Test signup → signin → profile update flow end-to-end

**Checkpoint**: User Story 2 complete - Authentication working, user profiles stored in Neon Postgres. Can demo signup/signin independently.

---

## Phase 5: User Story 3 - Ask Questions via RAG Chatbot (Priority: P3)

**Goal**: Embed AI-powered chatbot in textbook pages. Users ask questions about content, receive accurate answers grounded in textbook chapters with source citations. Chatbot maintains conversation context. Chat history persists for 24 hours.

**Independent Test**: Load sample chapters into Qdrant → Open chatbot → Ask "What are ROS 2 publishers?" → Verify answer cites Module 1 content → Ask follow-up → Verify context maintained → Check chat history in database → Verify auto-deletion after 24h.

### Backend Database Models for User Story 3

- [ ] T072 [P] [US3] Create ChatConversation model in backend/app/models/chat.py (id, user_id FK, title, created_at, expires_at, message_count)
- [ ] T073 [P] [US3] Create ChatMessage model in backend/app/models/chat.py (id, conversation_id FK, role, content, sources JSONB, sequence_number)
- [ ] T074 [US3] Create Alembic migration for chat tables in backend/app/database/migrations/ (include 24h expiration trigger)
- [ ] T075 [US3] Apply migration to Neon Postgres

### Backend Pydantic Schemas for User Story 3

- [ ] T076 [P] [US3] Create Chat Pydantic schemas in backend/app/models/chat.py (ChatConversationResponse, ChatMessageCreate, ChatMessageResponse, MessageSource)

### Backend RAG Services for User Story 3

- [ ] T077 [US3] Implement EmbeddingService in backend/app/services/embedding_service.py (generate embeddings using OpenAI text-embedding-3-small, chunk text with semantic splitter 1024 tokens / 15% overlap)
- [ ] T078 [US3] Implement QdrantService in backend/app/services/qdrant_service.py (create_collection, upsert_vectors, search_similar methods)
- [ ] T079 [US3] Implement RAGService in backend/app/services/rag_service.py using LlamaIndex (VectorStoreIndex with Qdrant backend, query with GPT-4, extract citations)
- [ ] T080 [US3] Implement ChatService in backend/app/services/chat_service.py (create_conversation, add_message, get_conversation_history, generate_ai_response methods)

### Backend Scripts for User Story 3

- [ ] T081 [US3] Create vector database seeding script in scripts/seed-vector-db.py (read markdown files from docs/curriculum/, chunk with EmbeddingService, upload to Qdrant)
- [ ] T082 [P] [US3] Create chat cleanup cron script in scripts/cleanup-chat-history.py (delete conversations where expires_at < now)
- [ ] T083 [US3] Run seed-vector-db.py to populate Qdrant with initial textbook content

### Backend API Endpoints for User Story 3

- [ ] T084 [P] [US3] Implement GET /chat/conversations endpoint in backend/app/routers/chat.py (list user's conversations, pagination)
- [ ] T085 [P] [US3] Implement POST /chat/conversations endpoint in backend/app/routers/chat.py (create new conversation)
- [ ] T086 [P] [US3] Implement GET /chat/conversations/{id}/messages endpoint in backend/app/routers/chat.py (retrieve messages for conversation)
- [ ] T087 [US3] Implement POST /chat/conversations/{id}/messages endpoint in backend/app/routers/chat.py (send user message, generate AI response via RAGService, return both messages with sources)
- [ ] T088 [US3] Register chat router in backend/app/main.py with /v1/chat prefix

### Frontend Components for User Story 3

- [ ] T089 [P] [US3] Create ChatBot component in src/components/ChatBot/ChatBot.tsx (floating widget, expandable chat window)
- [ ] T090 [P] [US3] Create MessageList component in src/components/ChatBot/MessageList.tsx (display user and assistant messages, render source citations as links)
- [ ] T091 [P] [US3] Create ChatInput component in src/components/ChatBot/ChatInput.tsx (textarea, send button, loading state)
- [ ] T092 [P] [US3] Create ConversationList component in src/components/ChatBot/ConversationList.tsx (sidebar with past conversations)

### Frontend Services for User Story 3

- [ ] T093 [US3] Implement chat API calls in src/services/chat.ts (getConversations, createConversation, getMessages, sendMessage functions)
- [ ] T094 [US3] Integrate chat API with ChatContext in src/contexts/ChatContext.tsx (manage active conversation, messages state, send message handler)

### Frontend Integration for User Story 3

- [ ] T095 [US3] Add ChatBot widget to CustomLayout in src/components/Layout/CustomLayout.tsx (bottom-right floating button)
- [ ] T096 [US3] Implement keyboard shortcut to open chatbot (Ctrl+K or Cmd+K)
- [ ] T097 [US3] Test chatbot: ask question → verify answer → verify sources clickable → verify conversation persisted → verify follow-up maintains context

**Checkpoint**: User Story 3 complete - RAG chatbot functional, answers grounded in textbook content with citations. Can demo asking questions independently.

---

## Phase 6: User Story 4 - Receive Personalized Learning Path Recommendations (Priority: P4)

**Goal**: Generate personalized chapter recommendations based on user's software/hardware background. Display recommendations on dashboard. Allow users to personalize chapter content presentation (adjust explanation depth) by clicking button at start of each chapter.

**Independent Test**: Create user with beginner software / no hardware background → Verify recommendations start with fundamentals → Create user with advanced/professional background → Verify recommendations skip basics → Click "Personalize Content" on a chapter → Verify explanation adapted → Check personalized content cached in database.

### Backend Database Models for User Story 4

- [ ] T098 [P] [US4] Create LearningPathRecommendation model in backend/app/models/recommendation.py (id, user_id FK, recommended_chapters JSONB, rationale, experience snapshot, expires_at)
- [ ] T099 [P] [US4] Create PersonalizedContentCache model in backend/app/models/recommendation.py (id, user_id FK, chapter_id, original_content, personalized_content, experience snapshot, expires_at)
- [ ] T100 [US4] Create Alembic migration for recommendation tables in backend/app/database/migrations/
- [ ] T101 [US4] Apply migration to Neon Postgres

### Backend Pydantic Schemas for User Story 4

- [ ] T102 [P] [US4] Create Personalization Pydantic schemas in backend/app/models/recommendation.py (LearningPathRecommendationResponse, RecommendedChapter, PersonalizeChapterRequest, PersonalizedContentResponse)

### Backend Services for User Story 4

- [ ] T103 [US4] Implement RecommendationService in backend/app/services/recommendation_service.py (generate_learning_path method using user background to prioritize chapters, create recommendation rationale)
- [ ] T104 [US4] Implement PersonalizationService in backend/app/services/personalization_service.py (personalize_chapter_content method using OpenAI GPT-4 to adapt explanation depth, cache result)

### Backend API Endpoints for User Story 4

- [ ] T105 [P] [US4] Implement GET /personalization/recommendations endpoint in backend/app/routers/personalization.py (return active learning path or generate new one)
- [ ] T106 [P] [US4] Implement POST /personalization/recommendations endpoint in backend/app/routers/personalization.py (force regenerate recommendations)
- [ ] T107 [US4] Implement POST /personalization/chapters/{chapter_id}/personalize endpoint in backend/app/routers/personalization.py (personalize content via PersonalizationService, check cache first, enforce rate limit 5/day)
- [ ] T108 [US4] Register personalization router in backend/app/main.py with /v1/personalization prefix

### Frontend Components for User Story 4

- [ ] T109 [P] [US4] Create LearningPathWidget component in src/components/Personalization/LearningPathWidget.tsx (display recommended chapters with rationale, priority indicators)
- [ ] T110 [P] [US4] Create PersonalizeButton component in src/components/Personalization/PersonalizeButton.tsx (button at start of chapter, loading state, rate limit warning)
- [ ] T111 [P] [US4] Create PersonalizedContentView component in src/components/Personalization/PersonalizedContentView.tsx (display adapted content, toggle to original)

### Frontend Services for User Story 4

- [ ] T112 [US4] Implement personalization API calls in src/services/personalization.ts (getRecommendations, regenerateRecommendations, personalizeChapter functions)
- [ ] T113 [US4] Integrate personalization API with PersonalizationContext in src/contexts/PersonalizationContext.tsx (recommendations state, personalizedChapters Map)

### Frontend Integration for User Story 4

- [ ] T114 [US4] Create learning path dashboard page in src/pages/dashboard.tsx (display LearningPathWidget, user background summary)
- [ ] T115 [US4] Add PersonalizeButton to chapter layout in src/components/Layout/ChapterLayout.tsx (inject at top of each chapter page)
- [ ] T116 [US4] Implement content toggle in PersonalizedContentView (switch between original and personalized)
- [ ] T117 [US4] Test personalization: login as beginner → verify recommendations → click personalize → verify adapted content → verify cache hit on reload → test rate limit

**Checkpoint**: User Story 4 complete - Learning path recommendations working, content personalization functional with caching. Can demo personalization independently.

---

## Phase 7: User Story 5 - Ask Questions About Selected Text (Priority: P5)

**Goal**: Allow users to highlight/select text within chapters and ask chatbot questions specifically about that selection. Chatbot focuses answer on selected context rather than entire book. Enhance deep learning experience with fine-grained Q&A.

**Independent Test**: Select paragraph in chapter → Click "Ask about this section" → Chatbot opens with selection as context → Ask "Explain this in simpler terms" → Verify answer focuses on selected text, not general topic → Deselect text → Ask new question → Verify returns to general mode.

### Backend Enhancements for User Story 5

- [ ] T118 [US5] Update ChatMessage model in backend/app/models/chat.py to include selected_text and selected_context JSONB fields (already defined in data-model.md, verify migration)
- [ ] T119 [US5] Update ChatMessageCreate schema to accept selected_text and selected_context (chapter_id, section_title, offsets)
- [ ] T120 [US5] Enhance RAGService in backend/app/services/rag_service.py with context-focused query method (prioritize selected text in prompt, filter Qdrant search to chapter_id)

### Backend API Updates for User Story 5

- [ ] T121 [US5] Update POST /chat/conversations/{id}/messages endpoint in backend/app/routers/chat.py to handle selected_text parameter (pass to RAGService context-focused query)

### Frontend Components for User Story 5

- [ ] T122 [P] [US5] Create TextSelector component in src/components/ChatBot/TextSelector.tsx (detect text selection events, show context menu or floating button)
- [ ] T123 [P] [US5] Create SelectionContextBadge component in src/components/ChatBot/SelectionContextBadge.tsx (display in chat input when text selected, clear button)

### Frontend Services for User Story 5

- [ ] T124 [US5] Implement text selection handler in src/hooks/useTextSelection.ts (detect mouse up, get selected text and range, extract chapter context)
- [ ] T125 [US5] Update sendMessage in src/services/chat.ts to include selected_text and selected_context parameters

### Frontend Integration for User Story 5

- [ ] T126 [US5] Integrate TextSelector with chapter content in src/components/Layout/ChapterLayout.tsx (add selection event listeners)
- [ ] T127 [US5] Update ChatInput component to display SelectionContextBadge when text is selected
- [ ] T128 [US5] Update MessageList component to visually indicate messages that used selected text context (show "Asked about selected text" badge)
- [ ] T129 [US5] Test selected text Q&A: select text → ask question → verify focused answer → clear selection → ask general question → verify general answer

**Checkpoint**: User Story 5 complete - Selected text Q&A working, chatbot can answer both general and context-specific questions. All user stories (P1-P5) now functional.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, deployment, and production readiness

### Documentation

- [ ] T130 [P] Create comprehensive README.md at repository root (project overview, features, tech stack, quick start link)
- [ ] T131 [P] Create backend/README.md (backend setup instructions, API documentation link)
- [ ] T132 [P] Verify quickstart.md in specs/001-physical-ai-textbook/ is up-to-date with actual setup steps

### Deployment Configuration

- [ ] T133 [US1] Create GitHub Actions workflow in .github/workflows/deploy-frontend.yml (build Docusaurus, deploy to GitHub Pages on push to main)
- [ ] T134 [P] Create GitHub Actions workflow in .github/workflows/deploy-backend.yml (deploy FastAPI to Render on push to main)
- [ ] T135 [P] Configure GitHub Pages in repository settings (source: gh-pages branch)
- [ ] T136 [P] Create Render.yaml for backend deployment configuration (Python runtime, environment variables, health check endpoint)
- [ ] T137 [US1] Test GitHub Pages deployment: push to main → verify build succeeds → verify site accessible at yourusername.github.io/physicalaibook
- [ ] T138 [P] Test Render deployment: connect Render to GitHub → trigger deploy → verify backend accessible → test health endpoint

### Environment & Secrets Management

- [ ] T139 [P] Configure GitHub Secrets for deployment (NEON_DATABASE_URL, QDRANT_API_KEY, QDRANT_URL, OPENAI_API_KEY, JWT_SECRET_KEY)
- [ ] T140 [P] Update backend .env.example with all required variables and descriptions
- [ ] T141 [P] Update frontend .env.local.example with backend API URL (production: Render URL)

### Performance & Optimization

- [ ] T142 [P] [US1] Optimize Docusaurus build (enable code splitting, minification, compression in docusaurus.config.js)
- [ ] T143 [P] [US3] Implement rate limiting for chatbot endpoints in backend/app/routers/chat.py (e.g., 10 requests per minute per user)
- [ ] T144 [P] [US4] Implement rate limiting for personalization endpoint (5 personalizations per day per user, enforced via database check)
- [ ] T145 [P] Add caching headers for static assets in frontend build configuration

### Monitoring & Logging

- [ ] T146 [P] Add health check endpoint GET /health in backend/app/main.py (return status, version, timestamp)
- [ ] T147 [P] Implement structured logging for all backend endpoints (log request ID, user ID, endpoint, duration, status code)
- [ ] T148 [P] Set up error tracking (optional: integrate Sentry or similar service for production error monitoring)

### Security Hardening

- [ ] T149 [P] Implement input sanitization for user-generated content (chat messages, profile data) in backend validators
- [ ] T150 [P] Add security headers to FastAPI responses (CORS, CSP, X-Frame-Options) via middleware in backend/app/main.py
- [ ] T151 [P] Verify password hashing uses bcrypt with appropriate cost factor (12-14 rounds)
- [ ] T152 [P] Implement JWT token expiration and refresh token mechanism (access token 7 days, refresh token 30 days)

### Database Management

- [ ] T153 [P] [US3] Set up automated chat history cleanup cron job on Render (run scripts/cleanup-chat-history.py hourly)
- [ ] T154 [P] Create database backup strategy documentation (Neon automatic backups, Qdrant snapshot process)
- [ ] T155 [P] Test database migrations rollback (alembic downgrade -1, verify schema intact)

### Final Validation

- [ ] T156 Run full quickstart.md setup from scratch to verify all instructions work
- [ ] T157 Test complete user journey: Browse content → Sign up → Ask chatbot question → Get recommendations → Personalize chapter → Ask about selected text
- [ ] T158 Verify all acceptance scenarios from spec.md pass for each user story (P1-P5)
- [ ] T159 [P] Check responsive design on real mobile devices (iOS Safari, Android Chrome)
- [ ] T160 [P] Verify page load performance targets (<3s load, <5s chatbot response) using Lighthouse or WebPageTest

**Checkpoint**: Polish complete - Application is production-ready, deployed, monitored, and fully functional across all user stories.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-7)**: All depend on Foundational phase completion
  - **US1 (P1)**: Can start after Foundational - No dependencies on other stories
  - **US2 (P2)**: Can start after Foundational - Independent (though US3/US4/US5 benefit from having auth)
  - **US3 (P3)**: Can start after Foundational - Benefits from US1 (content) but can use sample content
  - **US4 (P4)**: Depends on US2 (needs user profiles) - Can run in parallel with US3/US5
  - **US5 (P5)**: Depends on US3 (extends chatbot) - Should implement after US3 complete
- **Polish (Phase 8)**: Depends on desired user stories being complete

### User Story Dependencies

```
Foundation (Phase 2)
    ├── US1 (P1) - Browse Content [INDEPENDENT] 🎯 MVP
    ├── US2 (P2) - Authentication [INDEPENDENT]
    ├── US3 (P3) - RAG Chatbot [Soft dependency: US1 for content, US2 for user context]
    ├── US4 (P4) - Personalization [Depends on: US2 for user profiles]
    └── US5 (P5) - Selected Text Q&A [Depends on: US3 chatbot foundation]
```

**Recommended Implementation Order**:
1. Setup → Foundational (MUST complete first)
2. US1 (P1) - Deploy MVP with browsable content
3. US2 (P2) - Add authentication
4. US3 (P3) - Add chatbot (can now use authenticated user context)
5. US4 (P4) - Add personalization (uses user profiles from US2)
6. US5 (P5) - Add selected text Q&A (extends US3 chatbot)
7. Polish - Production deployment and optimization

### Within Each User Story

- Content creation tasks ([P]) can run in parallel
- Models before services
- Services before endpoints
- Frontend components ([P]) can run in parallel
- API integration after endpoints complete
- Test full story flow before moving to next priority

### Parallel Opportunities

**Phase 1 - Setup**: Tasks T001-T010 - All [P] tasks (T002, T003, T004, T005, T006, T008, T009, T010) can run in parallel

**Phase 2 - Foundational**:
- Backend foundation (T011-T020): [P] tasks (T012, T015, T016, T017, T019) can run in parallel
- Frontend foundation (T021-T029): [P] tasks (T022, T023, T024, T026, T027, T028) can run in parallel

**Phase 3 - US1**:
- All content creation tasks T030-T042 ([P]) can run in parallel
- UI tasks T044, T045 ([P]) can run in parallel

**Phase 4 - US2**:
- Models T048-T050 sequential, but schemas T051-T052 ([P]) can run in parallel
- All API endpoints T056-T059 ([P]) can run in parallel
- All frontend components T061-T064 ([P]) can run in parallel

**Phase 5 - US3**:
- Models T072-T073 ([P]), scripts T082 ([P]) can run in parallel after T077-T080 complete
- All API endpoints T084-T087 ([P]) can run in parallel
- All frontend components T089-T092 ([P]) can run in parallel

**Phase 6 - US4**:
- Models T098-T099 ([P]), schemas T102 ([P]) can run in parallel
- API endpoints T105-T107 ([P]) can run in parallel
- Frontend components T109-T111 ([P]) can run in parallel

**Phase 7 - US5**:
- Frontend components T122-T123 ([P]) can run in parallel

**Phase 8 - Polish**:
- Documentation T130-T132 ([P]), deployment T134-T138 ([P]), environment T139-T141 ([P]), performance T142-T145 ([P]), monitoring T146-T148 ([P]), security T149-T152 ([P]), database T153-T155 ([P]), validation T159-T160 ([P]) - many tasks can run in parallel

---

## Parallel Example: User Story 3 (RAG Chatbot)

```bash
# Launch all database models together:
Task: "Create ChatConversation model in backend/app/models/chat.py"
Task: "Create ChatMessage model in backend/app/models/chat.py"

# After models, launch all API endpoints together:
Task: "Implement GET /chat/conversations endpoint in backend/app/routers/chat.py"
Task: "Implement POST /chat/conversations endpoint in backend/app/routers/chat.py"
Task: "Implement GET /chat/conversations/{id}/messages endpoint"
# (T087 sequential after others due to complexity)

# Launch all frontend components together:
Task: "Create ChatBot component in src/components/ChatBot/ChatBot.tsx"
Task: "Create MessageList component in src/components/ChatBot/MessageList.tsx"
Task: "Create ChatInput component in src/components/ChatBot/ChatInput.tsx"
Task: "Create ConversationList component in src/components/ChatBot/ConversationList.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

**Minimum Viable Product** - Deliver core value fastest:

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T029) - CRITICAL BLOCKER
3. Complete Phase 3: User Story 1 (T030-T047) - **Browse Interactive Textbook**
4. **STOP and VALIDATE**: Test US1 independently
   - Verify all 4 modules accessible
   - Verify hardware requirements pages load
   - Verify responsive on mobile/tablet/desktop
   - Verify page load < 3 seconds
5. Deploy to GitHub Pages (T133, T137)
6. **Demo MVP**: "Browse our Physical AI textbook with 4 modules covering ROS 2, Gazebo, NVIDIA Isaac, and VLA/LLMs"

**MVP Task Count**: 47 tasks (Setup: 10, Foundational: 19, US1: 18)
**Estimated MVP Timeline**: 2-3 weeks for solo developer

### Incremental Delivery (Add Value Progressively)

After MVP, add one user story at a time:

1. **Iteration 1: MVP (US1)** → Deploy → Get feedback
2. **Iteration 2: +Authentication (US2)** → Deploy → Users can create accounts
3. **Iteration 3: +RAG Chatbot (US3)** → Deploy → Users can ask questions
4. **Iteration 4: +Personalization (US4)** → Deploy → Learning paths customized
5. **Iteration 5: +Selected Text Q&A (US5)** → Deploy → Fine-grained interactions
6. **Iteration 6: Polish** → Production-ready deployment

Each iteration:
- Adds independent, testable functionality
- Can be deployed and demoed standalone
- Doesn't break previous features
- Provides user value immediately

### Parallel Team Strategy

With 3+ developers, after Foundational phase completes:

**Developer A** (Frontend focus):
- Phase 3: US1 content creation and navigation (T030-T047)
- Phase 4: US2 frontend auth components (T061-T071)
- Phase 6: US4 frontend personalization UI (T109-T117)

**Developer B** (Backend focus):
- Phase 4: US2 backend auth system (T048-T060)
- Phase 5: US3 backend RAG services (T072-T088)
- Phase 6: US4 backend personalization services (T098-T108)

**Developer C** (Full-stack):
- Phase 5: US3 chatbot UI and integration (T089-T097)
- Phase 7: US5 selected text Q&A (T118-T129)
- Phase 8: Deployment and polish (T130-T160)

**Timeline with parallel work**: 4-6 weeks for all user stories + polish

---

## Task Summary

**Total Tasks**: 160 tasks across 8 phases

**Breakdown by Phase**:
- Phase 1 (Setup): 10 tasks
- Phase 2 (Foundational): 19 tasks
- Phase 3 (US1 - Browse Content): 18 tasks
- Phase 4 (US2 - Authentication): 24 tasks
- Phase 5 (US3 - RAG Chatbot): 26 tasks
- Phase 6 (US4 - Personalization): 20 tasks
- Phase 7 (US5 - Selected Text Q&A): 12 tasks
- Phase 8 (Polish): 31 tasks

**Breakdown by User Story**:
- US1 (P1): 18 tasks (+ deployment from Polish)
- US2 (P2): 24 tasks
- US3 (P3): 26 tasks (most complex - RAG system)
- US4 (P4): 20 tasks
- US5 (P5): 12 tasks (extends US3)

**Parallelizable Tasks**: ~80 tasks marked with [P] (50% can run in parallel within phases)

**MVP Scope**: 47 tasks (Setup + Foundational + US1)

---

## Notes

- All tasks follow checklist format: `- [ ] [ID] [P?] [Story] Description with file path`
- [P] marker indicates tasks that can run in parallel (different files, no blocking dependencies)
- [Story] label (US1-US5) maps tasks to user stories for traceability
- Each user story is independently implementable and testable
- Stop at any checkpoint to validate story independently before proceeding
- Tests are OPTIONAL (not explicitly requested in spec) - focus on implementation
- Commit after each task or logical group of related tasks
- Follow quickstart.md for development environment setup
- Refer to plan.md for architectural decisions and tech stack details
- Refer to data-model.md for database schema details
- Refer to contracts/openapi.yaml for API endpoint specifications

---

**Ready to implement!** Start with Phase 1 (Setup) and proceed through phases sequentially. Within each phase, leverage [P] parallelization opportunities. Focus on MVP (US1) first for fastest time-to-value.

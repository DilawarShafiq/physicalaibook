# Quickstart Development Guide

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Last Updated**: 2025-12-08

This guide will help you set up the complete development environment for the Physical AI textbook platform, including frontend (Docusaurus), backend (FastAPI), databases (Neon Postgres, Qdrant), and AI services (OpenAI).

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Project Structure Overview](#project-structure-overview)
3. [Frontend Setup (Docusaurus)](#frontend-setup-docusaurus)
4. [Backend Setup (FastAPI)](#backend-setup-fastapi)
5. [Database Setup](#database-setup)
6. [Running the Application](#running-the-application)
7. [Testing](#testing)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

Before you begin, ensure you have the following installed:

### Required Software

- **Node.js 20.x LTS** or higher
  - Download: https://nodejs.org/
  - Verify: `node --version` (should show v20.x.x)

- **Python 3.11+**
  - Download: https://www.python.org/downloads/
  - Verify: `python --version` or `python3 --version` (should show 3.11+)

- **Git**
  - Download: https://git-scm.com/
  - Verify: `git --version`

- **npm or pnpm** (package manager)
  - npm comes with Node.js
  - pnpm (recommended for faster installs): `npm install -g pnpm`
  - Verify: `npm --version` or `pnpm --version`

### Required Accounts & API Keys

1. **Neon Account** (Postgres database)
   - Sign up: https://neon.tech/
   - Create a new project: "physicalai-textbook"
   - Copy connection string (starts with `postgres://...`)

2. **Qdrant Cloud Account** (vector database)
   - Sign up: https://cloud.qdrant.io/
   - Create a free cluster: "textbook-embeddings"
   - Copy API key and cluster URL

3. **OpenAI API Key**
   - Sign up: https://platform.openai.com/
   - Create API key: https://platform.openai.com/api-keys
   - Copy API key (starts with `sk-...`)

4. **Render Account** (backend hosting - for deployment only)
   - Sign up: https://render.com/
   - Not needed for local development

---

## Project Structure Overview

```
physicalaibook/
├── docs/                    # Docusaurus content (markdown files)
│   ├── curriculum/          # Course chapters
│   ├── hardware/            # Hardware specifications
│   └── architecture/        # Architecture diagrams
├── src/                     # Docusaurus custom components (React + TypeScript)
│   ├── components/          # ChatBot, Auth, Personalization UI
│   ├── hooks/               # React hooks for API integration
│   ├── services/            # API client, auth service
│   └── types/               # TypeScript type definitions
├── backend/                 # FastAPI application (Python)
│   ├── app/                 # FastAPI app code
│   │   ├── models/          # Database models (SQLAlchemy + Pydantic)
│   │   ├── routers/         # API endpoints
│   │   ├── services/        # Business logic (RAG, auth, personalization)
│   │   └── database/        # DB connections, migrations
│   └── tests/               # Backend tests (pytest)
├── scripts/                 # Utility scripts
│   └── seed-vector-db.py    # Populate Qdrant with embeddings
├── .env.example             # Environment variables template
├── package.json             # Frontend dependencies
├── docusaurus.config.js     # Docusaurus configuration
└── README.md                # Project overview
```

---

## Frontend Setup (Docusaurus)

### Step 1: Clone the Repository

```bash
git clone https://github.com/YOUR_ORG/physicalaibook.git
cd physicalaibook
```

### Step 2: Install Frontend Dependencies

Using npm:
```bash
npm install
```

Or using pnpm (faster):
```bash
pnpm install
```

This installs:
- Docusaurus 3.x
- React 18.x
- TypeScript 5.x
- TanStack Query (for API state management)
- Other frontend dependencies

### Step 3: Configure Environment Variables

Create `.env.local` file in project root:

```bash
# Copy example file
cp .env.example .env.local
```

Edit `.env.local`:

```env
# Backend API URL (local development)
REACT_APP_API_BASE_URL=http://localhost:8000/v1

# Feature flags (optional)
REACT_APP_ENABLE_CHATBOT=true
REACT_APP_ENABLE_PERSONALIZATION=true
```

### Step 4: Start Development Server

```bash
npm start
# or
pnpm start
```

This command:
- Starts Docusaurus dev server on http://localhost:3000
- Enables hot reload (changes reflect immediately)
- Opens browser automatically

You should see the textbook homepage, but chatbot/auth features won't work until backend is running.

---

## Backend Setup (FastAPI)

### Step 1: Navigate to Backend Directory

```bash
cd backend
```

### Step 2: Create Python Virtual Environment

#### On macOS/Linux:
```bash
python3 -m venv venv
source venv/bin/activate
```

#### On Windows:
```bash
python -m venv venv
venv\Scripts\activate
```

You should see `(venv)` prefix in your terminal.

### Step 3: Install Backend Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

This installs:
- FastAPI 0.109+
- Uvicorn (ASGI server)
- LlamaIndex (RAG framework)
- OpenAI Python SDK
- Qdrant Client
- SQLAlchemy + asyncpg (Postgres)
- Alembic (database migrations)
- pytest (testing)
- Other dependencies

### Step 4: Configure Backend Environment Variables

Create `.env` file in `backend/` directory:

```bash
cp .env.example .env
```

Edit `backend/.env`:

```env
# Database
DATABASE_URL=postgres://USER:PASSWORD@HOST/DATABASE  # From Neon
DATABASE_ECHO=false  # Set to true for SQL query logging

# Qdrant Vector DB
QDRANT_URL=https://YOUR_CLUSTER.qdrant.io  # From Qdrant Cloud
QDRANT_API_KEY=your_qdrant_api_key_here

# OpenAI
OPENAI_API_KEY=sk-...  # Your OpenAI API key

# JWT Authentication
SECRET_KEY=your_secret_key_here_generate_with_openssl  # Generate with: openssl rand -hex 32
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=10080  # 7 days

# CORS (for local development)
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Application Settings
ENVIRONMENT=development
LOG_LEVEL=INFO

# Rate Limiting
MAX_PERSONALIZATIONS_PER_DAY=5
```

**Generate SECRET_KEY**:
```bash
openssl rand -hex 32
```

### Step 5: Run Database Migrations

Initialize the database schema:

```bash
cd app
alembic upgrade head
```

This creates all tables (users, chat_conversations, chat_messages, etc.) in your Neon Postgres database.

### Step 6: Seed Vector Database (Optional)

Populate Qdrant with textbook content embeddings:

```bash
cd ../..  # Return to project root
python scripts/seed-vector-db.py
```

This script:
- Reads markdown files from `docs/curriculum/`
- Chunks content using semantic chunking (1024 tokens, 15% overlap)
- Generates embeddings using OpenAI text-embedding-3-small
- Uploads vectors to Qdrant

**Note**: This requires OpenAI API credits. Initial seeding costs ~$0.50-$2.00 depending on content volume.

### Step 7: Start Backend Server

```bash
cd backend
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Flags:
- `--reload`: Auto-restart on code changes
- `--host 0.0.0.0`: Accept connections from any IP (for frontend to connect)
- `--port 8000`: Run on port 8000

The API should now be running at http://localhost:8000

**Verify**:
- API docs: http://localhost:8000/docs (Swagger UI)
- Health check: http://localhost:8000/v1/health

---

## Database Setup

### Neon Postgres Setup

1. **Create Neon Project**:
   - Go to https://console.neon.tech/
   - Click "Create Project"
   - Name: "physicalai-textbook"
   - Region: Choose closest to you
   - Postgres version: 16 (latest)

2. **Get Connection String**:
   - Copy connection string from dashboard
   - Format: `postgres://USER:PASSWORD@HOST/DATABASE?sslmode=require`
   - Add to `backend/.env` as `DATABASE_URL`

3. **Verify Connection**:
   ```bash
   psql "YOUR_CONNECTION_STRING"
   ```

   If successful, you'll see PostgreSQL prompt. Type `\q` to exit.

### Qdrant Cloud Setup

1. **Create Free Cluster**:
   - Go to https://cloud.qdrant.io/
   - Click "Create Cluster"
   - Name: "textbook-embeddings"
   - Plan: Free (1GB)
   - Region: Choose closest to you

2. **Get API Credentials**:
   - Copy Cluster URL (e.g., `https://abc123.qdrant.io`)
   - Create API Key: Settings → API Keys → Create Key
   - Add both to `backend/.env`

3. **Create Collection** (automatic on first seed):
   The collection is created automatically when you run `scripts/seed-vector-db.py`.

   Or create manually via Python:
   ```python
   from qdrant_client import QdrantClient
   from qdrant_client.models import VectorParams, Distance

   client = QdrantClient(url="YOUR_URL", api_key="YOUR_KEY")
   client.create_collection(
       collection_name="textbook_embeddings",
       vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
   )
   ```

---

## Running the Application

### Full Stack Development

You need **three terminal windows**:

**Terminal 1 - Frontend** (Docusaurus):
```bash
cd physicalaibook
npm start
# Runs on http://localhost:3000
```

**Terminal 2 - Backend** (FastAPI):
```bash
cd physicalaibook/backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn app.main:app --reload --port 8000
# Runs on http://localhost:8000
```

**Terminal 3 - Monitoring** (optional, for logs/testing):
```bash
cd physicalaibook/backend
source venv/bin/activate
# Run tests, check logs, etc.
```

### Verify Everything Works

1. **Frontend**: http://localhost:3000
   - Homepage loads
   - Navigation works
   - Can browse chapters

2. **Backend API**: http://localhost:8000/docs
   - Swagger UI loads
   - All endpoints listed
   - Can test `/v1/health` endpoint

3. **Test User Flow**:
   - Sign up: Create new account with background info
   - Sign in: Authenticate with email/password
   - Chat: Ask a question about textbook content
   - Personalize: Click "Personalize Content" button on a chapter

---

## Testing

### Frontend Tests

Run Jest + React Testing Library tests:

```bash
npm test
# or
npm run test:coverage  # With coverage report
```

Test files location: `src/**/__tests__/`

### Backend Tests

Run pytest tests:

```bash
cd backend
pytest
# or
pytest -v  # Verbose output
pytest --cov=app --cov-report=html  # With coverage report
```

Test files location: `backend/tests/`

**Test categories**:
- `tests/unit/`: Unit tests (isolated functions)
- `tests/integration/`: API integration tests
- `tests/contract/`: OpenAPI contract compliance tests

### E2E Tests (Future)

Playwright E2E tests will be added post-MVP.

---

## Troubleshooting

### Frontend Issues

**Issue**: `npm start` fails with "Port 3000 already in use"

**Solution**:
```bash
# Kill process on port 3000
# On macOS/Linux:
lsof -ti:3000 | xargs kill -9
# On Windows:
netstat -ano | findstr :3000
taskkill /PID <PID> /F
```

**Issue**: API calls fail with CORS error

**Solution**: Ensure backend `.env` has:
```env
CORS_ORIGINS=http://localhost:3000
```

---

### Backend Issues

**Issue**: `alembic upgrade head` fails with "relation already exists"

**Solution**: Database schema already exists. Run:
```bash
alembic stamp head  # Mark current version
```

**Issue**: OpenAI API calls fail with "Rate limit exceeded"

**Solution**:
- Check API key is correct
- Verify billing is set up: https://platform.openai.com/account/billing
- Reduce request frequency (use cache)

**Issue**: Qdrant connection fails

**Solution**:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test connection manually:
  ```python
  from qdrant_client import QdrantClient
  client = QdrantClient(url="YOUR_URL", api_key="YOUR_KEY")
  print(client.get_collections())
  ```

---

### Database Issues

**Issue**: Neon connection fails with "password authentication failed"

**Solution**:
- Verify connection string is correct
- Check password doesn't contain special characters (URL-encode if needed)
- Ensure IP is allowlisted (Neon allows all IPs by default on free tier)

**Issue**: Chat history not auto-deleting after 24 hours

**Solution**: Cleanup job is not running. Run manually:
```bash
python scripts/cleanup-chat-history.py
```

Set up cron job (production):
```cron
0 * * * * /path/to/venv/bin/python /path/to/scripts/cleanup-chat-history.py
```

---

## Next Steps

After successful setup:

1. **Create sample content**: Add markdown files to `docs/curriculum/`
2. **Seed vector database**: Run `python scripts/seed-vector-db.py`
3. **Test complete user flow**:
   - Sign up → Sign in → Ask chatbot question → Personalize content
4. **Read implementation tasks**: `specs/001-physical-ai-textbook/tasks.md` (created by `/sp.tasks`)
5. **Start development**: Begin implementing features from tasks.md

---

## Useful Commands Reference

### Frontend (Docusaurus)
```bash
npm start                 # Start dev server
npm run build             # Production build
npm test                  # Run tests
npm run lint              # Lint code
npm run format            # Format code (Prettier)
```

### Backend (FastAPI)
```bash
uvicorn app.main:app --reload  # Start dev server
pytest                    # Run tests
alembic revision --autogenerate -m "message"  # Create migration
alembic upgrade head      # Apply migrations
black app/                # Format code
ruff app/                 # Lint code
```

### Database
```bash
psql "CONNECTION_STRING"  # Connect to Neon Postgres
alembic current           # Show current migration version
alembic history           # Show migration history
```

---

## Additional Resources

- **Docusaurus Documentation**: https://docusaurus.io/docs
- **FastAPI Documentation**: https://fastapi.tiangolo.com/
- **LlamaIndex Documentation**: https://docs.llamaindex.ai/
- **Qdrant Documentation**: https://qdrant.tech/documentation/
- **OpenAI API Reference**: https://platform.openai.com/docs/api-reference

---

**Questions or Issues?**

- Check troubleshooting section above
- Review API documentation: http://localhost:8000/docs
- Consult architecture plan: `specs/001-physical-ai-textbook/plan.md`
- Open GitHub issue for bugs

Happy developing! 🤖📚

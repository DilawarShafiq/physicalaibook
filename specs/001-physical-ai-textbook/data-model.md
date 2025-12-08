# Data Model & Database Schema

**Feature**: Physical AI & Humanoid Robotics Interactive Textbook
**Date**: 2025-12-08
**Phase**: Phase 1 - Design

## Overview

This document defines the database schemas, entity relationships, and data structures for the textbook platform. The system uses two databases:
- **Neon Postgres**: Relational data (users, chat history, recommendations)
- **Qdrant Vector DB**: Embeddings for semantic search

## Entity Relationship Diagram

```
┌─────────────┐
│    User     │
└──────┬──────┘
       │
       │ 1:N
       ├───────────────┐
       │               │
       ▼               ▼
┌─────────────┐ ┌──────────────────┐
│ChatConversation│ │LearningPathRec│
└──────┬──────┘ └──────────────────┘
       │
       │ 1:N
       ▼
┌─────────────┐
│ChatMessage  │
└─────────────┘

┌────────────────┐
│ VectorEmbedding│ (Qdrant)
│  (per chunk)   │
└────────────────┘
```

## PostgreSQL Schema (Neon)

### Table: `users`

Stores user profiles and background information for personalization.

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,

    -- User background for personalization
    software_experience VARCHAR(20) NOT NULL CHECK (
        software_experience IN ('beginner', 'intermediate', 'advanced')
    ),
    hardware_experience VARCHAR(20) NOT NULL CHECK (
        hardware_experience IN ('none', 'hobbyist', 'professional')
    ),

    -- Profile metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login_at TIMESTAMP WITH TIME ZONE,

    -- Status
    is_active BOOLEAN DEFAULT TRUE,
    email_verified BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);
```

**Fields**:
- `id`: Primary key, UUID for user identification
- `email`: Unique user email address (used for login)
- `hashed_password`: Bcrypt-hashed password (never store plaintext)
- `software_experience`: Self-reported programming skill level
- `hardware_experience`: Self-reported robotics/hardware skill level
- `created_at`: Account creation timestamp
- `updated_at`: Last profile update timestamp
- `last_login_at`: Last successful login timestamp
- `is_active`: Account status (for soft deletion)
- `email_verified`: Email verification status (future feature)

**Validation Rules**:
- Email must be valid format (validated at API layer with Pydantic)
- Password must be >=8 characters with complexity requirements
- Experience levels constrained by CHECK constraints

---

### Table: `chat_conversations`

Stores chat sessions with 24-hour retention policy.

```sql
CREATE TABLE chat_conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,

    -- Conversation metadata
    title VARCHAR(255), -- Auto-generated from first question
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL, -- Auto-purge after 24 hours

    -- Context tracking
    last_message_at TIMESTAMP WITH TIME ZONE,
    message_count INTEGER DEFAULT 0,

    -- Status
    is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_conversations_user_id ON chat_conversations(user_id);
CREATE INDEX idx_conversations_expires_at ON chat_conversations(expires_at);
CREATE INDEX idx_conversations_created_at ON chat_conversations(created_at);

-- Automatic 24-hour expiration
CREATE OR REPLACE FUNCTION set_conversation_expiration()
RETURNS TRIGGER AS $$
BEGIN
    NEW.expires_at = CURRENT_TIMESTAMP + INTERVAL '24 hours';
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER set_expiration_on_insert
    BEFORE INSERT ON chat_conversations
    FOR EACH ROW
    EXECUTE FUNCTION set_conversation_expiration();
```

**Fields**:
- `id`: Primary key for conversation
- `user_id`: Foreign key to users table (cascade delete)
- `title`: Auto-generated conversation title (e.g., "Question about sensors")
- `created_at`: Conversation start timestamp
- `expires_at`: Automatic expiration timestamp (24 hours from creation)
- `last_message_at`: Timestamp of most recent message
- `message_count`: Total messages in conversation (for UI pagination)
- `is_active`: Conversation status

**Retention Policy**:
- Conversations automatically expire 24 hours after creation
- Cleanup job (`scripts/cleanup-chat-history.py`) runs hourly to delete expired conversations
- User can manually close conversations (sets `is_active = FALSE`)

---

### Table: `chat_messages`

Stores individual messages within conversations.

```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID REFERENCES chat_conversations(id) ON DELETE CASCADE,

    -- Message content
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,

    -- Context for selected text Q&A
    selected_text TEXT, -- NULL for general questions
    selected_context JSONB, -- Metadata about selected text (chapter, section)

    -- RAG metadata (for assistant messages only)
    sources JSONB, -- Array of source citations [{chapter, section, chunk_id}]

    -- Timestamps
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

    -- Ordering
    sequence_number INTEGER NOT NULL
);

CREATE INDEX idx_messages_conversation_id ON chat_messages(conversation_id);
CREATE INDEX idx_messages_created_at ON chat_messages(created_at);
CREATE INDEX idx_messages_sequence ON chat_messages(conversation_id, sequence_number);
```

**Fields**:
- `id`: Primary key for message
- `conversation_id`: Foreign key to chat_conversations (cascade delete)
- `role`: Message sender (`user` or `assistant`)
- `content`: Message text content
- `selected_text`: User-selected text for context (if applicable)
- `selected_context`: Metadata about selection (chapter ID, section title)
- `sources`: RAG sources cited in assistant response (JSON array)
- `created_at`: Message timestamp
- `sequence_number`: Message order within conversation

**JSON Schema Examples**:

`selected_context` structure:
```json
{
  "chapter_id": "module-1/ros2-basics",
  "section_title": "ROS 2 Communication Patterns",
  "start_offset": 120,
  "end_offset": 350
}
```

`sources` structure (assistant messages):
```json
[
  {
    "chapter_id": "module-1/ros2-basics",
    "section_title": "Publishers and Subscribers",
    "chunk_id": "chunk_42",
    "relevance_score": 0.89
  },
  {
    "chapter_id": "module-2/gazebo-intro",
    "section_title": "Simulation Basics",
    "chunk_id": "chunk_103",
    "relevance_score": 0.76
  }
]
```

---

### Table: `learning_path_recommendations`

Stores personalized chapter recommendations for users.

```sql
CREATE TABLE learning_path_recommendations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,

    -- Recommendation data
    recommended_chapters JSONB NOT NULL, -- Ordered list of chapter IDs
    rationale TEXT, -- Why these chapters were recommended

    -- Background snapshot (for tracking changes)
    software_experience VARCHAR(20) NOT NULL,
    hardware_experience VARCHAR(20) NOT NULL,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP + INTERVAL '7 days',

    -- Status
    is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_recommendations_user_id ON learning_path_recommendations(user_id);
CREATE INDEX idx_recommendations_expires_at ON learning_path_recommendations(expires_at);

-- Unique active recommendation per user
CREATE UNIQUE INDEX idx_active_recommendation_per_user
    ON learning_path_recommendations(user_id)
    WHERE is_active = TRUE;
```

**Fields**:
- `id`: Primary key for recommendation
- `user_id`: Foreign key to users table
- `recommended_chapters`: Ordered list of chapter IDs to read
- `rationale`: Explanation of why these chapters match user's background
- `software_experience`: Snapshot of user's software level (for tracking changes)
- `hardware_experience`: Snapshot of user's hardware level
- `created_at`: Recommendation generation timestamp
- `expires_at`: Recommendation validity period (7 days)
- `is_active`: Whether recommendation is current

**Business Rules**:
- Only one active recommendation per user at a time
- Regenerate recommendations when user updates background profile
- Expire recommendations after 7 days (prompt re-generation based on progress)

**JSON Schema for `recommended_chapters`**:
```json
[
  {
    "chapter_id": "curriculum/introduction",
    "module": "introduction",
    "priority": 1,
    "reason": "Essential foundation for all learners"
  },
  {
    "chapter_id": "curriculum/module-1/ros2-basics",
    "module": "module-1",
    "priority": 2,
    "reason": "Beginner-friendly ROS 2 introduction"
  },
  {
    "chapter_id": "hardware/workstation",
    "module": "hardware",
    "priority": 3,
    "reason": "Critical hardware setup for development environment"
  }
]
```

---

### Table: `personalized_content_cache`

Caches AI-generated personalized content to reduce API costs.

```sql
CREATE TABLE personalized_content_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,

    -- Personalized content
    original_content TEXT NOT NULL,
    personalized_content TEXT NOT NULL,

    -- User background snapshot (cache key)
    software_experience VARCHAR(20) NOT NULL,
    hardware_experience VARCHAR(20) NOT NULL,

    -- Metadata
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP + INTERVAL '30 days',
    access_count INTEGER DEFAULT 0,
    last_accessed_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_personalized_cache_lookup
    ON personalized_content_cache(user_id, chapter_id, software_experience, hardware_experience);
CREATE INDEX idx_personalized_cache_expires_at ON personalized_content_cache(expires_at);

-- Composite unique constraint for cache lookup
CREATE UNIQUE INDEX idx_personalized_cache_unique
    ON personalized_content_cache(user_id, chapter_id, software_experience, hardware_experience)
    WHERE expires_at > CURRENT_TIMESTAMP;
```

**Fields**:
- `id`: Primary key
- `user_id`: Foreign key to users table
- `chapter_id`: Chapter identifier (e.g., "curriculum/module-1/ros2-basics")
- `original_content`: Original chapter content (for comparison)
- `personalized_content`: AI-generated adapted content
- `software_experience`: User's software level when content was generated
- `hardware_experience`: User's hardware level when content was generated
- `created_at`: Cache entry creation timestamp
- `expires_at`: Cache expiration (30 days)
- `access_count`: Number of times cached content was served
- `last_accessed_at`: Last access timestamp

**Cache Invalidation**:
- Expires after 30 days
- Invalidated when user updates background profile
- Invalidated when chapter content is updated (content hash mismatch)

---

## Qdrant Vector Database Schema

### Collection: `textbook_embeddings`

Stores vector embeddings of textbook content chunks for semantic search.

**Collection Configuration**:
```python
{
    "vectors": {
        "size": 1536,  # OpenAI text-embedding-3-small dimension
        "distance": "Cosine"  # Cosine similarity for semantic search
    },
    "optimizers_config": {
        "indexing_threshold": 20000  # Start indexing after 20k vectors
    }
}
```

**Payload Schema** (metadata for each vector):
```json
{
    "chunk_id": "string (unique)",
    "chapter_id": "string",
    "module_number": "integer (1-4)",
    "section_title": "string",
    "content": "string (actual text chunk)",
    "chunk_index": "integer (sequence within chapter)",
    "difficulty_level": "string (beginner|intermediate|advanced)",
    "topics": ["array", "of", "topics"],
    "word_count": "integer",
    "created_at": "timestamp",
    "content_hash": "string (MD5 of content for change detection)"
}
```

**Payload Example**:
```json
{
    "chunk_id": "module-1-ros2-basics-chunk-5",
    "chapter_id": "curriculum/module-1/ros2-basics",
    "module_number": 1,
    "section_title": "Publishers and Subscribers",
    "content": "In ROS 2, the publisher-subscriber pattern is a fundamental communication mechanism...",
    "chunk_index": 5,
    "difficulty_level": "beginner",
    "topics": ["ros2", "communication", "publisher", "subscriber"],
    "word_count": 342,
    "created_at": "2025-12-08T10:30:00Z",
    "content_hash": "a1b2c3d4e5f6..."
}
```

**Indexing Strategy**:
- Create index after 20,000 vectors for optimal search performance
- Use cosine distance for semantic similarity
- Filter queries by metadata (module, difficulty_level, topics)

**Search Filters** (example):
```python
# Filter by module for targeted search
{
    "must": [
        {"key": "module_number", "match": {"value": 1}}
    ]
}

# Filter by difficulty level
{
    "must": [
        {"key": "difficulty_level", "match": {"value": "beginner"}}
    ]
}

# Combine filters
{
    "must": [
        {"key": "module_number", "range": {"gte": 1, "lte": 2}},
        {"key": "difficulty_level", "match": {"any": ["beginner", "intermediate"]}}
    ]
}
```

---

## Data Validation & Constraints

### User Input Validation (Pydantic Models)

**User Creation**:
```python
from pydantic import BaseModel, EmailStr, constr, Field

class UserCreate(BaseModel):
    email: EmailStr
    password: constr(min_length=8)
    software_experience: Literal["beginner", "intermediate", "advanced"]
    hardware_experience: Literal["none", "hobbyist", "professional"]

class UserUpdate(BaseModel):
    software_experience: Optional[Literal["beginner", "intermediate", "advanced"]]
    hardware_experience: Optional[Literal["none", "hobbyist", "professional"]]
```

**Chat Message**:
```python
class ChatMessageCreate(BaseModel):
    content: constr(min_length=1, max_length=5000)
    selected_text: Optional[str] = None
    selected_context: Optional[dict] = None
```

### Database Constraints Summary

1. **Referential Integrity**:
   - All foreign keys with `ON DELETE CASCADE` for automatic cleanup
   - Orphaned records automatically removed when parent is deleted

2. **Uniqueness**:
   - Users: unique email
   - Active recommendations: one per user
   - Personalized content cache: unique per (user, chapter, experience levels)

3. **Data Quality**:
   - CHECK constraints on enum fields (experience levels, message roles)
   - NOT NULL constraints on required fields
   - Timestamp defaults for audit trail

4. **Performance**:
   - Indexes on foreign keys, frequently queried fields
   - Composite indexes for common query patterns
   - Partial indexes for filtered queries

---

## State Transitions

### Chat Conversation Lifecycle

```
[Created] → [Active] → [Inactive/Expired] → [Deleted]
    ↓           ↓              ↓
  (New)     (Messages)    (24h passed)
```

States:
- **Created**: New conversation initialized
- **Active**: User actively sending messages
- **Inactive**: User stopped interacting (still within 24h)
- **Expired**: 24 hours passed since creation
- **Deleted**: Purged by cleanup job

### Learning Path Recommendation Lifecycle

```
[Generated] → [Active] → [Expired/Invalidated] → [Archived]
     ↓           ↓              ↓
  (First)  (User views)  (7d or profile change)
```

States:
- **Generated**: New recommendation created based on user profile
- **Active**: Current recommendation shown to user
- **Expired**: 7 days passed or user updated profile
- **Archived**: Marked inactive, new recommendation generated

---

## Migration Strategy

**Initial Setup** (Alembic migrations):

1. **Migration 001**: Create users table
2. **Migration 002**: Create chat_conversations and chat_messages tables
3. **Migration 003**: Create learning_path_recommendations table
4. **Migration 004**: Create personalized_content_cache table
5. **Migration 005**: Create indexes and triggers

**Qdrant Setup** (script-based):

- Script: `scripts/seed-vector-db.py`
- Creates collection with proper configuration
- Indexes initial textbook content (markdown files)

---

## Data Retention & Cleanup

### Automated Cleanup Jobs

1. **Chat History Purge** (hourly):
   ```sql
   DELETE FROM chat_conversations WHERE expires_at < CURRENT_TIMESTAMP;
   ```

2. **Expired Recommendations** (daily):
   ```sql
   UPDATE learning_path_recommendations
   SET is_active = FALSE
   WHERE expires_at < CURRENT_TIMESTAMP AND is_active = TRUE;
   ```

3. **Stale Content Cache** (daily):
   ```sql
   DELETE FROM personalized_content_cache WHERE expires_at < CURRENT_TIMESTAMP;
   ```

### Backup Strategy

- **Neon Postgres**: Automatic daily backups (managed by Neon)
- **Qdrant**: Periodic snapshots of vector collection (Qdrant Cloud managed)
- **Critical data**: Users table backed up before major migrations

---

## Security Considerations

1. **Password Storage**: Never store plaintext passwords, use bcrypt with salt
2. **User Data**: Email and profile information encrypted at rest (Neon encryption)
3. **Chat History**: Automatically purged after 24 hours (privacy by design)
4. **Access Control**: Row-level security on sensitive tables (future enhancement)
5. **API Keys**: Never store OpenAI API keys in database (use environment variables)

---

## Next Steps

1. ✅ Data model complete
2. **Create API contracts**: `contracts/openapi.yaml` and `contracts/types.ts`
3. **Generate quickstart guide**: `quickstart.md` for development setup
4. **Implement models**: Create SQLAlchemy ORM models and Pydantic schemas

# 🚀 Quick Start Guide

## What's Been Built

✅ **Beautiful Docusaurus Textbook** with modern AI-themed design
✅ **Chapter-based structure** (not week-based)
✅ **Better-auth authentication** configured
✅ **Personalization feature** using OpenAI Agents SDK + Gemini
✅ **Urdu translation feature** using Gemini
✅ **ChapterTools component** with beautiful UI
✅ **FastAPI backend** with all endpoints ready
✅ **Google Gemini integration** via OpenAI SDK

## How to Run

### Step 1: Start the Frontend

```bash
# In the project root
npm start
```

Open http://localhost:3000

### Step 2: Start the Backend

```bash
# Open a new terminal
cd backend

# Install dependencies (first time only)
pip install -r requirements.txt

# Start server
python -m uvicorn app.main:app --reload
```

API runs at http://localhost:8000

## Using the Features

### 1. View the Textbook
- Navigate to any chapter
- Beautiful indigo/purple theme
- Dark mode available

### 2. Sign Up / Sign In
- Use better-auth for authentication
- Provide your software/hardware background

### 3. Personalize Content
- Click "Personalize for Me" button at chapter start
- Content adapts to your experience level
- Uses Gemini 2.0 Flash via OpenAI SDK

### 4. Translate to Urdu
- Click "Translate to Urdu" button
- Get chapter in proper Urdu script
- Right-to-left formatting automatically applied

### 5. AI Chatbot (Coming Soon)
- RAG chatbot using Qdrant
- Ask questions about textbook
- Highlight text and ask specific questions

## API Endpoints

### Authentication
- POST `/api/v1/auth/signup` - Create account
- POST `/api/v1/auth/signin` - Login
- GET `/api/v1/auth/me` - Get current user

### Personalization
- POST `/api/v1/personalization/chapters/personalize` - Personalize content
- POST `/api/v1/personalization/chapters/translate` - Translate content

### Chat (Coming Soon)
- POST `/api/v1/chat/conversations` - Create conversation
- POST `/api/v1/chat/conversations/{id}/messages` - Send message

## Configuration

Your API key is already set in `backend/.env`:
```env
GOOGLE_API_KEY=AIzaSyAV9ynWcw7Z3554ovVYMalpX_fEuhnY-lg
GEMINI_MODEL=gemini-2.0-flash-exp
```

Using OpenAI SDK with Gemini:
```python
from openai import OpenAI

client = OpenAI(
    api_key=settings.GOOGLE_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

response = client.chat.completions.create(
    model="gemini-2.0-flash-exp",
    messages=[...]
)
```

## What's Next?

1. **Test personalization**: Sign up and click "Personalize for Me"
2. **Test translation**: Click "Translate to Urdu"
3. **Add more chapters**: Create content in `docs/curriculum/`
4. **Deploy**: Push to GitHub and deploy to GitHub Pages

## Troubleshooting

### Frontend won't start
```bash
rm -rf node_modules package-lock.json
npm install
npm start
```

### Backend errors
```bash
cd backend
pip install --upgrade openai
python -m uvicorn app.main:app --reload
```

### API key issues
- Check `backend/.env` has correct GOOGLE_API_KEY
- Verify Gemini API is enabled in Google Cloud Console

## Beautiful Design Features

✨ **Colors**
- Primary: Indigo (#6366f1)
- Accent Cyan: #06b6d4
- Accent Purple: #a855f7
- Stunning dark mode with deep blues

✨ **Components**
- Gradient buttons with hover effects
- Smooth animations
- Shadow depth effects
- Beautiful typography (Inter + JetBrains Mono)

✨ **Chapter Tools**
- Personalize button (gradient purple)
- Translate button (gradient cyan)
- Show Original button (gradient gray)
- Loading spinners
- Success badges

---

**You're all set! Start exploring the textbook! 📚🤖**

# Physical AI & Humanoid Robotics Interactive Textbook

An AI-enhanced interactive textbook for learning Physical AI, built with Docusaurus, FastAPI, and Google Gemini.

## ✨ Features

- 📚 **Interactive Textbook**: Comprehensive curriculum on ROS 2, Gazebo, NVIDIA Isaac, and VLA
- 🤖 **AI-Powered RAG Chatbot**: Ask questions about textbook content
- 👤 **User Authentication**: Better-auth integration with background collection
- 🎯 **Content Personalization**: Adapt chapters to your experience level
- 🌍 **Urdu Translation**: Translate chapters to Urdu with one click
- 💬 **Selected Text Q&A**: Highlight text and ask questions about it

## 🚀 Quick Start

### Prerequisites

- Node.js 18+
- Python 3.11+
- Google Gemini API Key

### Frontend Setup

```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build
```

The site will open at `http://localhost:3000`

### Backend Setup

```bash
# Navigate to backend
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
# Edit backend/.env with your API keys

# Run migrations
alembic upgrade head

# Start server
uvicorn app.main:app --reload
```

The API will be available at `http://localhost:8000`

## 🔑 Environment Variables

### Frontend (`.env.local`)

```env
REACT_APP_API_BASE_URL=http://localhost:8000/api/v1
```

### Backend (`backend/.env`)

```env
# Database
DATABASE_URL=sqlite:///./physical_ai_textbook.db

# Qdrant Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=

# Google Gemini
GOOGLE_API_KEY=AIzaSyAV9ynWcw7Z3554ovVYMalpX_fEuhnY-lg
GEMINI_MODEL=gemini-2.0-flash-exp

# JWT
SECRET_KEY=your-super-secret-dev-key-change-in-production

# App Settings
ENVIRONMENT=development
FRONTEND_URL=http://localhost:3000
```

## 📖 Curriculum Structure

### Module 1: ROS 2 Fundamentals
- Chapter 1.1: ROS 2 Basics
- Chapter 1.2: Advanced Communication
- Chapter 1.3: Navigation & Transforms

### Module 2: Digital Twin & Gazebo
- Chapter 2.1: Gazebo Fundamentals
- Chapter 2.2: Sim-to-Real Transfer
- Chapter 2.3: Multi-Robot Systems

### Module 3: AI-Robot Brain & NVIDIA Isaac
- Chapter 3.1: Computer Vision
- Chapter 3.2: Isaac Sim
- Chapter 3.3: Reinforcement Learning
- Chapter 3.4: Perception-Action Pipelines

### Module 4: VLA & LLMs for Robotics
- Chapter 4.1: LLMs for Robotics
- Chapter 4.2: Vision-Language-Action Models
- Chapter 4.3: Capstone Project

## 🛠️ Tech Stack

### Frontend
- **Framework**: Docusaurus 3.0
- **Auth**: Better-auth
- **Styling**: Custom CSS with beautiful gradients
- **State Management**: React Context + TanStack Query

### Backend
- **API**: FastAPI
- **AI**: OpenAI Agents SDK + Google Gemini
- **Vector DB**: Qdrant
- **Database**: SQLite (dev) / PostgreSQL (prod)
- **Auth**: Better-auth sessions

## 🎨 Beautiful Design

The textbook features:
- Modern indigo/purple AI-themed color palette
- Stunning dark mode
- Smooth animations and transitions
- Readable typography (Inter + JetBrains Mono)
- Gradient buttons and surfaces
- Shadow effects for depth

## 📱 Features in Detail

### Personalization
Click "Personalize for Me" at the start of any chapter to adapt the content based on your:
- Software experience (beginner/intermediate/advanced)
- Hardware experience (none/hobbyist/professional)

### Translation
Click "Translate to Urdu" to get the chapter in Urdu with:
- Technical terms kept in English
- Proper right-to-left formatting
- Beautiful Urdu typography

### RAG Chatbot
- Ask questions about the textbook
- Get answers grounded in course content
- Highlight text and ask specific questions
- Conversation history maintained

## 🚀 Deployment

### GitHub Pages

```bash
# Update docusaurus.config.js with your GitHub info
# Build the site
npm run build

# Deploy
GIT_USER=<your-username> npm run deploy
```

### Vercel (Alternative)

```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
vercel
```

## 📝 License

MIT License - feel free to use for education!

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines first.

## 📧 Contact

Questions? Open an issue or reach out to the team!

---

Built with ❤️ using Docusaurus, FastAPI, and Google Gemini

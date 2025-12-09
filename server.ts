/**
 * Better Auth Server
 * Provides authentication endpoints and custom student progress APIs
 */

import express from 'express';
import cors from 'cors';
import { auth } from './src/auth';
import { toNodeHandler } from 'better-auth/node';
import Database from 'better-sqlite3';

const app = express();
const PORT = process.env.AUTH_SERVER_PORT || 3001;

// Middleware
app.use(cors({
  origin: ['http://localhost:3000', 'http://127.0.0.1:3000'],
  credentials: true,
}));
app.use(express.json());

// Better Auth routes - handles all /api/auth/* endpoints
app.use('/api/auth', toNodeHandler(auth));

// Custom student progress endpoints
const db = new Database(process.env.DATABASE_URL?.replace('file:', '') || './auth.db');

// Update student progress for a module
app.post('/api/auth/update-progress', async (req, res) => {
  try {
    const { moduleId, progress } = req.body;
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    // Get session from Better Auth
    const sessionToken = authHeader.replace('Bearer ', '');
    const session = db.prepare(`
      SELECT * FROM session WHERE id = ?
    `).get(sessionToken) as any;

    if (!session) {
      return res.status(401).json({ error: 'Invalid session' });
    }

    // Update user progress
    db.prepare(`
      UPDATE user
      SET progress = ?, currentModule = ?
      WHERE id = ?
    `).run(progress, moduleId, session.userId);

    res.json({ success: true, moduleId, progress });
  } catch (error) {
    console.error('Error updating progress:', error);
    res.status(500).json({ error: 'Failed to update progress' });
  }
});

// Mark module as completed
app.post('/api/auth/complete-module', async (req, res) => {
  try {
    const { moduleId } = req.body;
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const sessionToken = authHeader.replace('Bearer ', '');
    const session = db.prepare(`
      SELECT * FROM session WHERE id = ?
    `).get(sessionToken) as any;

    if (!session) {
      return res.status(401).json({ error: 'Invalid session' });
    }

    // Get current completed modules
    const user = db.prepare(`
      SELECT completedModules FROM user WHERE id = ?
    `).get(session.userId) as any;

    const completedModules = JSON.parse(user.completedModules || '[]');

    if (!completedModules.includes(moduleId)) {
      completedModules.push(moduleId);

      db.prepare(`
        UPDATE user
        SET completedModules = ?
        WHERE id = ?
      `).run(JSON.stringify(completedModules), session.userId);
    }

    res.json({ success: true, moduleId, completedModules });
  } catch (error) {
    console.error('Error completing module:', error);
    res.status(500).json({ error: 'Failed to complete module' });
  }
});

// Mark lab as completed
app.post('/api/auth/complete-lab', async (req, res) => {
  try {
    const { labId } = req.body;
    const authHeader = req.headers.authorization;

    if (!authHeader) {
      return res.status(401).json({ error: 'Unauthorized' });
    }

    const sessionToken = authHeader.replace('Bearer ', '');
    const session = db.prepare(`
      SELECT * FROM session WHERE id = ?
    `).get(sessionToken) as any;

    if (!session) {
      return res.status(401).json({ error: 'Invalid session' });
    }

    // Get current completed labs
    const user = db.prepare(`
      SELECT completedLabs FROM user WHERE id = ?
    `).get(session.userId) as any;

    const completedLabs = JSON.parse(user.completedLabs || '[]');

    if (!completedLabs.includes(labId)) {
      completedLabs.push(labId);

      db.prepare(`
        UPDATE user
        SET completedLabs = ?
        WHERE id = ?
      `).run(JSON.stringify(completedLabs), session.userId);
    }

    res.json({ success: true, labId, completedLabs });
  } catch (error) {
    console.error('Error completing lab:', error);
    res.status(500).json({ error: 'Failed to complete lab' });
  }
});

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', timestamp: new Date().toISOString() });
});

// Start server
app.listen(PORT, () => {
  console.log(`✅ Better Auth server running on http://localhost:${PORT}`);
  console.log(`📚 API endpoints:`);
  console.log(`   - POST /api/auth/update-progress`);
  console.log(`   - POST /api/auth/complete-module`);
  console.log(`   - POST /api/auth/complete-lab`);
  console.log(`   - All /api/auth/* (Better Auth)`);
});

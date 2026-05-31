/**
 * expand-pilot.ts
 *
 * Pilot for the Claude lesson-authoring pass. Takes ONE lesson from the
 * Physical AI book and expands its outline (overview + key insights + cited
 * papers) into a full, written lesson — generating it with BOTH Sonnet 4.6
 * and Opus 4.8 so the quality/cost trade-off can be compared side by side.
 *
 * The generation is GROUNDED: Claude is instructed to teach only from the
 * provided facts and citations and not invent numbers, claims, or papers.
 *
 * Setup:  create  physicalaibook/.env.pilot  containing:
 *           ANTHROPIC_API_KEY=sk-ant-...
 * Run:    npx tsx scripts/expand-pilot.ts [lessonId]   (default 2.1)
 * Output: scripts/pilot/<lessonId>-sonnet-4-6.md
 *         scripts/pilot/<lessonId>-opus-4-8.md
 */
import * as fs from 'fs'
import * as path from 'path'
import { fileURLToPath } from 'url'
import Anthropic from '@anthropic-ai/sdk'
import { allBooks } from '../../autosap_website/src/data/allBooks'
import type { AcademyModule, Lesson } from '../../autosap_website/src/data/academyData'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const OUT_DIR = path.join(__dirname, 'pilot')

const MODELS = [
  { id: 'claude-sonnet-4-6', label: 'sonnet-4-6' },
  { id: 'claude-opus-4-8', label: 'opus-4-8' },
]

/** Minimal .env loader (no dependency) — reads KEY=VALUE lines. */
function loadEnv(file: string) {
  if (!fs.existsSync(file)) return
  for (const line of fs.readFileSync(file, 'utf8').split('\n')) {
    const m = line.match(/^\s*([A-Z0-9_]+)\s*=\s*(.*)\s*$/)
    if (m && !process.env[m[1]]) process.env[m[1]] = m[2].replace(/^["']|["']$/g, '')
  }
}

function findLesson(lessonId: string): { lesson: Lesson; module: AcademyModule; bookTitle: string } | null {
  const book = allBooks.find((b) => b.id === 'physical-ai')
  if (!book) return null
  for (const module of book.modules) {
    const lesson = module.lessons.find((l) => l.id === lessonId)
    if (lesson) return { lesson, module, bookTitle: book.title }
  }
  return null
}

function buildPrompt(lesson: Lesson, module: AcademyModule, bookTitle: string): string {
  const papers =
    lesson.papers && lesson.papers.length
      ? lesson.papers.map((p) => `- "${p.title}" — ${p.authors} (${p.year}), ${p.venue}`).join('\n')
      : '(none provided)'
  return [
    `BOOK: ${bookTitle}`,
    `MODULE ${module.number}: ${module.title} — ${module.subtitle}`,
    `MODULE CONTEXT: ${module.description}`,
    ``,
    `LESSON ${lesson.id}: ${lesson.title}`,
    `TARGET DURATION: ${lesson.duration}`,
    ``,
    `OVERVIEW (the seed prose):`,
    lesson.overview,
    ``,
    `KEY INSIGHTS (the factual backbone — every technical claim you make must trace to these or the papers):`,
    lesson.keyInsights.map((k) => `- ${k}`).join('\n'),
    ``,
    `HANDS-ON LAB (to expand into a worked, step-by-step exercise):`,
    lesson.lab || '(none provided)',
    ``,
    `CITED PAPERS (cite these by name where relevant; do not invent others):`,
    papers,
  ].join('\n')
}

const SYSTEM = `You are a senior author writing for Autosapien Academy, a research-grade technical textbook on physical AI and humanoid robotics, read by robotics engineers and ML researchers.

Your job: expand a lesson OUTLINE into a complete, genuinely instructive WRITTEN LESSON in Markdown.

Hard rules:
- GROUND everything in the provided overview, key insights, and cited papers. Do NOT invent numbers, benchmarks, product specs, dates, or citations beyond what is given. You may add intuition, explanation, analogies, reasoning, and pedagogical structure — but every concrete technical claim must trace to the supplied material.
- Teach, don't summarize. Explain WHY things are true, build intuition, connect ideas, and walk through reasoning.
- Length: roughly 1,800-2,400 words.

Structure (Markdown, start at level-2 headings — no H1, no frontmatter):
- A short motivating introduction (no heading) that frames why this lesson matters.
- 3-5 thematic "## " sections that teach the concepts in depth, in a logical build.
- Concrete examples or worked reasoning where it aids understanding.
- A "## Putting it into practice" section that expands the hands-on lab into clear, numbered steps with what to observe and why.
- A "## Key takeaways" section: 4-6 tight bullets.
Cite papers inline by name in prose (e.g., "the approach introduced in *Expressive Whole-Body Control* ..."). Output ONLY the Markdown lesson body.`

async function main() {
  loadEnv(path.join(__dirname, '..', '.env.pilot'))
  const apiKey = process.env.ANTHROPIC_API_KEY
  if (!apiKey) {
    console.error(
      '\n✗ ANTHROPIC_API_KEY not found.\n  Create physicalaibook/.env.pilot with:\n    ANTHROPIC_API_KEY=sk-ant-...\n',
    )
    process.exit(1)
  }

  const lessonId = process.argv[2] || '2.1'
  const found = findLesson(lessonId)
  if (!found) {
    console.error(`✗ Lesson ${lessonId} not found in the Physical AI book.`)
    process.exit(1)
  }

  const { lesson, module, bookTitle } = found
  const prompt = buildPrompt(lesson, module, bookTitle)
  const client = new Anthropic({ apiKey })
  fs.mkdirSync(OUT_DIR, { recursive: true })

  console.log(`Lesson ${lesson.id}: ${lesson.title}\n`)
  for (const model of MODELS) {
    process.stdout.write(`Generating with ${model.id} ... `)
    const start = Date.now()
    const msg = await client.messages.create({
      model: model.id,
      max_tokens: 5000,
      system: SYSTEM,
      messages: [{ role: 'user', content: prompt }],
    })
    const text = msg.content
      .filter((b): b is Anthropic.TextBlock => b.type === 'text')
      .map((b) => b.text)
      .join('')
    const words = text.split(/\s+/).length
    const secs = ((Date.now() - start) / 1000).toFixed(1)
    const header = `<!-- model: ${model.id} | lesson ${lesson.id} | ${words} words | in ${msg.usage.input_tokens}t out ${msg.usage.output_tokens}t -->\n\n`
    const outFile = path.join(OUT_DIR, `${lessonId}-${model.label}.md`)
    fs.writeFileSync(outFile, header + text + '\n', 'utf8')
    console.log(`${words} words, ${secs}s → scripts/pilot/${lessonId}-${model.label}.md`)
  }
  console.log('\nDone. Compare the two files to choose the authoring model.')
}

main().catch((e) => {
  console.error(e)
  process.exit(1)
})

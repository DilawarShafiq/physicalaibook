/**
 * gen-books.ts
 *
 * Faithfully converts the Autosapien Academy book data (TypeScript) into
 * Docusaurus markdown docs in the Panaversity "Agent Factory" textbook style —
 * one doc per lesson, grouped into per-module folders, with a module index
 * page and a per-book intro page.
 *
 * Each lesson page follows a consistent pedagogical structure:
 *   1. Learning objectives (info admonition)
 *   2. Why this matters
 *   3. Overview
 *   4. Key concepts (first concept highlighted as a note)
 *   5. Hands-on lab (tip admonition)
 *   6. Check your understanding (active-recall flashcards)
 *   7. References
 *   8. Module footer / navigation context
 *
 * Source of truth: ../../autosapien/src/data/allBooks.ts
 * Output:          ./books/<book-id>/...
 *
 * Run: npx tsx scripts/gen-books.ts
 */
import * as fs from 'fs'
import * as path from 'path'
import { fileURLToPath } from 'url'
import { allBooks, type Book } from '../../autosap_website/src/data/allBooks'
import type { AcademyModule, Lesson, Paper } from '../../autosap_website/src/data/academyData'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const REPO_ROOT = path.resolve(__dirname, '..')
const BOOKS_DIR = path.join(REPO_ROOT, 'books')

/** Escape characters that confuse the MDX compiler in prose text. */
function mdxSafe(text: string): string {
  return text
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/\{/g, '&#123;')
    .replace(/\}/g, '&#125;')
}

/** Produce a valid double-quoted YAML scalar from any JS string. */
function yamlString(text: string): string {
  return JSON.stringify(text)
}

/** Produce a valid YAML flow array of strings. */
function yamlArray(items: string[]): string {
  return '[' + items.map((i) => JSON.stringify(i)).join(', ') + ']'
}

function pad2(n: number): string {
  return String(n).padStart(2, '0')
}

/** Sanitize a lesson id like "1.1" or "C1.1" → "1-1" / "c1-1". */
function lessonIdSlug(id: string): string {
  return id.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '')
}

function slugifyTitle(title: string): string {
  return title
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-|-$/g, '')
    .slice(0, 60)
}

function lessonFileName(lesson: Lesson): string {
  return `lesson-${lessonIdSlug(lesson.id)}-${slugifyTitle(lesson.title)}.md`
}

/**
 * Derive a short topic "cue" from a key insight. Insights are typically
 * formatted "Topic: detail" or "Topic — detail"; we use the left side as a
 * recall cue. Falls back to a truncated prefix.
 */
function deriveCue(insight: string): string {
  // Prefer the "Topic: detail" / "Topic — detail" left-hand side as a clean label.
  const m = insight.match(/^(.{4,80}?)(:\s| — | - | – )/)
  if (m) return m[1].trim().replace(/[.,;:]$/, '')
  // Otherwise use the first natural clause/sentence — never truncate mid-phrase with an ellipsis.
  const comma = insight.indexOf(', ')
  if (comma > 12 && comma <= 80) return insight.slice(0, comma).trim()
  const period = insight.search(/\.\s/)
  if (period > 12 && period <= 90) return insight.slice(0, period).trim()
  return insight.trim()
}

/** Split an insight into a (cue, detail) pair for flashcards. */
function splitInsight(insight: string): { cue: string; detail: string } {
  const m = insight.match(/^(.{4,72}?)(:\s| — | - | – )(.*)$/)
  if (m) return { cue: m[1].trim().replace(/[.,;]$/, ''), detail: m[3].trim() }
  return { cue: deriveCue(insight), detail: insight }
}

function renderPapers(papers: Paper[]): string {
  const lines = papers.map(
    (p) => `- **${mdxSafe(p.title)}** — ${mdxSafe(p.authors)} (${p.year}). *${mdxSafe(p.venue)}*`,
  )
  return ['## References', '', ...lines, ''].join('\n')
}

/** First sentence of a block of prose, for "why this matters" framing. */
function firstSentence(text: string): string {
  const idx = text.search(/\.\s/)
  return idx === -1 ? text : text.slice(0, idx + 1)
}

function renderLesson(
  lesson: Lesson,
  module: AcademyModule,
  position: number,
  prev: Lesson | null,
  next: Lesson | null,
): string {
  const fm: string[] = ['---']
  fm.push(`title: ${yamlString(lesson.title)}`)
  fm.push(`sidebar_label: ${yamlString(`${lesson.id} ${lesson.title}`)}`)
  fm.push(`sidebar_position: ${position}`)
  fm.push(`description: ${yamlString(lesson.overview.split('. ')[0].slice(0, 180))}`)
  if (lesson.tags && lesson.tags.length) {
    fm.push(`tags: ${yamlArray(lesson.tags)}`)
  }
  fm.push('---')

  const body: string[] = []
  body.push('')
  body.push(`# ${mdxSafe(lesson.title)}`)
  body.push('')

  // Metadata line (duration + level + module + tags surfaced visibly)
  const meta = [
    `**Duration:** ${mdxSafe(lesson.duration)}`,
    `**Level:** ${mdxSafe(module.level)}`,
    `**Module:** ${module.number}. ${mdxSafe(module.title)}`,
  ]
  if (lesson.tags && lesson.tags.length) {
    meta.push(`**Focus:** ${lesson.tags.map((t) => `\`${mdxSafe(t)}\``).join(', ')}`)
  }
  body.push(meta.join(' · '))
  body.push('')

  // Learning objectives — derived from the topics covered by the key insights.
  if (lesson.keyInsights && lesson.keyInsights.length) {
    const cues: string[] = []
    const seen = new Set<string>()
    for (const insight of lesson.keyInsights) {
      const cue = deriveCue(insight)
      const key = cue.toLowerCase()
      if (!seen.has(key)) {
        seen.add(key)
        cues.push(cue)
      }
      if (cues.length >= 5) break
    }
    body.push(':::info Learning objectives')
    body.push('')
    body.push('By the end of this lesson you will be able to explain and apply:')
    body.push('')
    for (const cue of cues) {
      body.push(`- ${mdxSafe(cue)}`)
    }
    if (lesson.lab) {
      body.push('')
      body.push('You will then consolidate these ideas in the hands-on lab below.')
    }
    body.push(':::')
    body.push('')
  }

  // Overview as the main exposition.
  body.push('## Overview')
  body.push('')
  body.push(mdxSafe(lesson.overview))
  body.push('')

  // Key concepts — first insight highlighted as a "key idea" note.
  if (lesson.keyInsights && lesson.keyInsights.length) {
    body.push('## Key concepts')
    body.push('')
    const [first, ...rest] = lesson.keyInsights
    body.push(':::note Key idea')
    body.push('')
    body.push(mdxSafe(first))
    body.push('')
    body.push(':::')
    body.push('')
    if (rest.length) {
      for (const insight of rest) {
        body.push(`- ${mdxSafe(insight)}`)
      }
      body.push('')
    }
  }

  // Hands-on lab as a tip admonition.
  if (lesson.lab) {
    body.push(':::tip Hands-on lab')
    body.push('')
    body.push(mdxSafe(lesson.lab))
    body.push('')
    body.push(':::')
    body.push('')
  }

  // Check your understanding — active-recall flashcards built from insights.
  if (lesson.keyInsights && lesson.keyInsights.length) {
    body.push('## Check your understanding')
    body.push('')
    body.push('Cover the answers and try to recall each point before expanding it.')
    body.push('')
    lesson.keyInsights.slice(0, 5).forEach((insight, i) => {
      const { cue, detail } = splitInsight(insight)
      const prompt =
        cue && cue.toLowerCase() !== detail.toLowerCase() ? cue : `Key point ${i + 1}`
      body.push('<details>')
      body.push(`<summary>${mdxSafe(prompt)}</summary>`)
      body.push('')
      body.push(mdxSafe(detail))
      body.push('')
      body.push('</details>')
      body.push('')
    })
  }

  // Papers as a References list.
  if (lesson.papers && lesson.papers.length) {
    body.push(renderPapers(lesson.papers))
  }

  // Navigation context footer.
  body.push('---')
  body.push('')
  const navBits: string[] = []
  if (prev) navBits.push(`← Previous: **${mdxSafe(prev.id)} ${mdxSafe(prev.title)}**`)
  if (next) navBits.push(`Next: **${mdxSafe(next.id)} ${mdxSafe(next.title)}** →`)
  if (navBits.length) {
    body.push(navBits.join(' · '))
    body.push('')
  }
  body.push(`*Part of Module ${module.number}: ${mdxSafe(module.title)}.*`)
  body.push('')

  return fm.join('\n') + '\n' + body.join('\n') + '\n'
}

function renderModuleIndex(module: AcademyModule, book: Book): string {
  const fm: string[] = ['---']
  fm.push(`title: ${yamlString(module.title)}`)
  fm.push(`sidebar_label: ${yamlString('Overview')}`)
  fm.push(`sidebar_position: 1`)
  fm.push(`description: ${yamlString(module.description)}`)
  fm.push('---')

  const body: string[] = []
  body.push('')
  body.push(`# Module ${module.number}: ${mdxSafe(module.title)}`)
  body.push('')
  body.push(`*${mdxSafe(module.subtitle)}*`)
  body.push('')
  body.push(
    [
      `**Duration:** ${mdxSafe(module.duration)}`,
      `**Level:** ${mdxSafe(module.level)}`,
      `**Lessons:** ${module.lessons.length}`,
    ].join(' · '),
  )
  body.push('')
  body.push(mdxSafe(module.description))
  body.push('')

  if (module.prerequisites && module.prerequisites.length) {
    body.push('## Prerequisites')
    body.push('')
    for (const pre of module.prerequisites) {
      const target = book.modules.find((m) => m.slug === pre)
      if (target) {
        body.push(
          `- [Module ${target.number}: ${mdxSafe(target.title)}](../module-${pad2(
            target.number,
          )}-${target.slug}/index.md)`,
        )
      } else {
        body.push(`- ${mdxSafe(pre)}`)
      }
    }
    body.push('')
  } else {
    body.push('## Prerequisites')
    body.push('')
    body.push('None — this is an entry point into the book.')
    body.push('')
  }

  if (module.outcomes && module.outcomes.length) {
    body.push(':::info Learning outcomes')
    body.push('')
    body.push('By the end of this module you will be able to:')
    body.push('')
    for (const out of module.outcomes) {
      body.push(`- ${mdxSafe(out)}`)
    }
    body.push(':::')
    body.push('')
  }

  body.push('## Lessons in this module')
  body.push('')
  module.lessons.forEach((lesson, idx) => {
    const file = lessonFileName(lesson)
    body.push(
      `${idx + 1}. [${mdxSafe(lesson.id)} — ${mdxSafe(lesson.title)}](./${file}) · *${mdxSafe(lesson.duration)}*`,
    )
  })
  body.push('')

  const firstLesson = module.lessons[0]
  if (firstLesson) {
    body.push('---')
    body.push('')
    body.push(
      `👉 **Start here:** [${mdxSafe(firstLesson.id)} — ${mdxSafe(firstLesson.title)}](./${lessonFileName(firstLesson)})`,
    )
    body.push('')
  }

  return fm.join('\n') + '\n' + body.join('\n') + '\n'
}

function renderBookIntro(book: Book): string {
  const fm: string[] = ['---']
  fm.push(`title: ${yamlString(book.title)}`)
  fm.push(`sidebar_label: ${yamlString('Introduction')}`)
  fm.push(`sidebar_position: 0`)
  fm.push(`slug: /`)
  fm.push(`description: ${yamlString(book.description)}`)
  fm.push('---')

  const totalLessons = book.modules.reduce((acc, m) => acc + m.lessons.length, 0)

  const body: string[] = []
  body.push('')
  body.push(`# ${mdxSafe(book.title)}`)
  body.push('')
  body.push(`### ${mdxSafe(book.subtitle)}`)
  body.push('')
  body.push(`> ${mdxSafe(book.hook)}`)
  body.push('')
  body.push(mdxSafe(book.description))
  body.push('')
  body.push(
    [
      `**Modules:** ${book.modules.length}`,
      `**Lessons:** ${totalLessons}`,
      `**Total hours:** ${mdxSafe(book.totalHours)}`,
      `**Author:** ${mdxSafe(book.author)}`,
    ].join(' · '),
  )
  body.push('')

  body.push('## Who this book is for')
  body.push('')
  for (const a of book.audience) {
    body.push(`- ${mdxSafe(a)}`)
  }
  body.push('')

  body.push(':::tip How to use this book')
  body.push('')
  body.push(
    'Each lesson opens with **learning objectives**, builds the ideas in **Overview** and **Key concepts**, then asks you to apply them in a **hands-on lab** and test recall with **Check your understanding**. Work the labs — they are where the learning sticks.',
  )
  body.push(':::')
  body.push('')

  body.push('## Modules')
  body.push('')
  book.modules.forEach((m) => {
    const dir = `module-${pad2(m.number)}-${m.slug}`
    body.push(`### [Module ${m.number}: ${mdxSafe(m.title)}](./${dir}/index.md)`)
    body.push('')
    body.push(`*${mdxSafe(m.subtitle)}* — ${mdxSafe(m.description)}`)
    body.push('')
    body.push(`\`${mdxSafe(m.duration)}\` · \`${mdxSafe(m.level)}\` · ${m.lessons.length} lessons`)
    body.push('')
  })

  const qs = book.quickStart
  const qsModule = book.modules.find((m) => m.slug === qs.slug)
  if (qsModule) {
    const dir = `module-${pad2(qsModule.number)}-${qsModule.slug}`
    body.push('---')
    body.push('')
    body.push(`👉 **${mdxSafe(qs.label)}:** [${mdxSafe(qsModule.title)}](./${dir}/index.md)`)
    body.push('')
  }

  return fm.join('\n') + '\n' + body.join('\n') + '\n'
}

function writeFile(filePath: string, content: string) {
  fs.mkdirSync(path.dirname(filePath), { recursive: true })
  fs.writeFileSync(filePath, content, 'utf8')
}

function generateBook(book: Book) {
  const bookDir = path.join(BOOKS_DIR, book.id)
  // clean
  fs.rmSync(bookDir, { recursive: true, force: true })
  fs.mkdirSync(bookDir, { recursive: true })

  // intro
  writeFile(path.join(bookDir, 'intro.md'), renderBookIntro(book))

  let lessonCount = 0
  for (const module of book.modules) {
    const dir = path.join(bookDir, `module-${pad2(module.number)}-${module.slug}`)
    fs.mkdirSync(dir, { recursive: true })

    // _category_.json
    const category = {
      label: `Module ${module.number}: ${module.title}`,
      position: module.number + 1, // intro is position 0
      link: {
        type: 'doc',
        id: `module-${pad2(module.number)}-${module.slug}/index`,
      },
    }
    writeFile(path.join(dir, '_category_.json'), JSON.stringify(category, null, 2) + '\n')

    // module index
    writeFile(path.join(dir, 'index.md'), renderModuleIndex(module, book))

    // lessons
    module.lessons.forEach((lesson, idx) => {
      const prev = idx > 0 ? module.lessons[idx - 1] : null
      const next = idx < module.lessons.length - 1 ? module.lessons[idx + 1] : null
      writeFile(
        path.join(dir, lessonFileName(lesson)),
        renderLesson(lesson, module, idx + 2, prev, next),
      )
      lessonCount++
    })
  }

  console.log(
    `✓ ${book.id}: ${book.modules.length} modules, ${lessonCount} lessons → books/${book.id}/`,
  )
}

function main() {
  fs.mkdirSync(BOOKS_DIR, { recursive: true })
  for (const book of allBooks) {
    generateBook(book)
  }
  console.log('Done.')
}

main()

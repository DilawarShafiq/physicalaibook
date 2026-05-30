/**
 * gen-books.ts
 *
 * Faithfully converts the Autosapien Academy book data (TypeScript) into
 * Docusaurus markdown docs — one doc per lesson, grouped into per-module
 * folders, with a module index page and a per-book intro page.
 *
 * Source of truth: ../../autosapien/src/data/allBooks.ts
 * Output:          ./books/<book-id>/...
 *
 * Run: npx tsx scripts/gen-books.ts
 */
import * as fs from 'fs'
import * as path from 'path'
import { fileURLToPath } from 'url'
import { allBooks, type Book } from '../../autosapien/src/data/allBooks'
import type { AcademyModule, Lesson, Paper } from '../../autosapien/src/data/academyData'

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

function renderPapers(papers: Paper[]): string {
  const lines = papers.map(
    (p) => `- **${mdxSafe(p.title)}** — ${mdxSafe(p.authors)} (${p.year}). *${mdxSafe(p.venue)}*`,
  )
  return ['## References', '', ...lines, ''].join('\n')
}

function renderLesson(
  lesson: Lesson,
  module: AcademyModule,
  position: number,
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
  // Metadata line (duration + level + tags surfaced visibly)
  const meta = [
    `**Duration:** ${mdxSafe(lesson.duration)}`,
    `**Level:** ${mdxSafe(module.level)}`,
    `**Module:** ${module.number}. ${mdxSafe(module.title)}`,
  ]
  if (lesson.tags && lesson.tags.length) {
    meta.push(`**Tags:** ${lesson.tags.map((t) => `\`${mdxSafe(t)}\``).join(', ')}`)
  }
  body.push(meta.join(' · '))
  body.push('')

  // Overview as intro prose
  body.push('## Overview')
  body.push('')
  body.push(mdxSafe(lesson.overview))
  body.push('')

  // Key insights as bulleted list
  if (lesson.keyInsights && lesson.keyInsights.length) {
    body.push('## Key Insights')
    body.push('')
    for (const insight of lesson.keyInsights) {
      body.push(`- ${mdxSafe(insight)}`)
    }
    body.push('')
  }

  // Lab as a tip admonition
  if (lesson.lab) {
    body.push(':::tip Lab')
    body.push('')
    body.push(mdxSafe(lesson.lab))
    body.push('')
    body.push(':::')
    body.push('')
  }

  // Papers as a References list
  if (lesson.papers && lesson.papers.length) {
    body.push(renderPapers(lesson.papers))
  }

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
    body.push('## Learning Outcomes')
    body.push('')
    body.push('By the end of this module you will be able to:')
    body.push('')
    for (const out of module.outcomes) {
      body.push(`- ${mdxSafe(out)}`)
    }
    body.push('')
  }

  body.push('## Lessons')
  body.push('')
  module.lessons.forEach((lesson) => {
    const file = `lesson-${lessonIdSlug(lesson.id)}-${slugifyTitle(lesson.title)}.md`
    body.push(`- [${mdxSafe(lesson.id)} — ${mdxSafe(lesson.title)}](./${file}) · *${mdxSafe(lesson.duration)}*`)
  })
  body.push('')

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

  body.push('## Modules')
  body.push('')
  book.modules.forEach((m) => {
    const dir = `module-${pad2(m.number)}-${m.slug}`
    body.push(
      `### [Module ${m.number}: ${mdxSafe(m.title)}](./${dir}/index.md)`,
    )
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
      const file = `lesson-${lessonIdSlug(lesson.id)}-${slugifyTitle(lesson.title)}.md`
      writeFile(path.join(dir, file), renderLesson(lesson, module, idx + 2))
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

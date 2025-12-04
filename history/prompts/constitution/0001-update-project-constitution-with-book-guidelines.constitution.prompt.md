---
id: 0001
title: Update Project Constitution with Book Guidelines
stage: constitution
date: 2025-12-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution 1. Writing Tone & Style\n\nThe book must use a friendly, simple, and approachable tone.\n\nAvoid overly academic or complex language.\n\nUse short paragraphs only (2–4 sentences max).\n\nExplanations should be clear, human-readable, and easy for beginners.\n\n2. Structure & File Organization\n\nAll book content must be stored in:\n/content/chapters\n\nEach chapter file must follow a consistent structure:\n\nTitle\n\nShort introduction\n\nMain content in small sections\n\nExamples (optional)\n\nSummary\n\n3. AI Usage & Verification\n\nNo AI-generated text may be used without human verification.\n\nEvery generated paragraph must be reviewed for correctness, clarity, and tone.\n\nAny claims or facts must be checked manually before acceptance.\n\nAI drafts are allowed, but humans must approve all final text.\n\n4. Consistency Rules\n\nMaintain consistent terminology throughout the book.\n\nStyle must remain friendly and simple across all chapters.\n\nFormatting must follow the same pattern:\n\nHeadings use H1–H3\n\nBullet points for lists\n\nShort paragraphs only\n\nNo long blocks of text\n\n5. Quality Requirements\n\nContent must be original and not copied from external sources.\n\nAll examples must be correct and tested (if applicable).\n\nThe content must be free of factual errors.\n\nThe tone, structure, and clarity must match this constitution before a chapter is accepted.\n\n6. Editorial Guidelines\n\nAvoid unnecessary jargon.\n\nUse examples and analogies whenever helpful.\n\nKeep sections focused — no off-topic content.\n\nEach chapter should flow logically from the previous one.
labels: ["constitution", "guidelines", "book"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
- .specify/templates/plan-template.md
tests: null
---

## Prompt

1. Writing Tone & Style

The book must use a friendly, simple, and approachable tone.

Avoid overly academic or complex language.

Use short paragraphs only (2–4 sentences max).

Explanations should be clear, human-readable, and easy for beginners.

2. Structure & File Organization

All book content must be stored in:
/content/chapters

Each chapter file must follow a consistent structure:

Title

Short introduction

Main content in small sections

Examples (optional)

Summary

3. AI Usage & Verification

No AI-generated text may be used without human verification.

Every generated paragraph must be reviewed for correctness, clarity, and tone.

Any claims or facts must be checked manually before acceptance.

AI drafts are allowed, but humans must approve all final text.

4. Consistency Rules

Maintain consistent terminology throughout the book.

Style must remain friendly and simple across all chapters.

Formatting must follow the same pattern:

Headings use H1–H3

Bullet points for lists

Short paragraphs only

No long blocks of text

5. Quality Requirements

Content must be original and not copied from external sources.

All examples must be correct and tested (if applicable).

The content must be free of factual errors.

The tone, structure, and clarity must match this constitution before a chapter is accepted.

6. Editorial Guidelines

Avoid unnecessary jargon.

Use examples and analogies whenever helpful.

Keep sections focused — no off-topic content.

Each chapter should flow logically from the previous one.

## Response snapshot

Updated the project constitution (`.specify/memory/constitution.md`) with the new book writing and content guidelines. Also, updated the `plan-template.md` to reflect the constitution check based on these new principles.

## Outcome

- ✅ Impact: Project constitution updated with book writing and content guidelines.
- 🧪 Tests: No automated tests were run, as this was a documentation update.
- 📁 Files: Updated project constitution and adjusted plan template.
- 🔁 Next prompts: None.
- 🧠 Reflection: Successfully updated project constitution. Ensured consistency across relevant templates.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null

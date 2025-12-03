<!--
Version change: [CONSTITUTION_VERSION_OLD] -> 1.0.0
Modified principles:
- [PRINCIPLE_1_NAME] -> 1. Writing Tone & Style
- [PRINCIPLE_2_NAME] -> 2. Docusaurus Structure & Formatting Rules
- [PRINCIPLE_3_NAME] -> 3. File Naming Rules
- [PRINCIPLE_4_NAME] -> 4. Sidebar & Navigation Conventions
- [PRINCIPLE_5_NAME] -> 5. AI Usage & Verification Rules
- [PRINCIPLE_6_NAME] -> 6. Quality & Editorial Standards
- [SECTION_2_NAME] -> 7. Consistency Rules
- [SECTION_3_NAME] -> 8. Prohibited Content
Removed sections:
- None explicitly removed, principles were replaced with concrete content.
Added sections:
- None explicitly added, principles were replaced with concrete content.
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated (Constitution Check needs manual review)
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
Follow-up TODOs:
- Manually review and update the "Constitution Check" section in .specify/templates/plan-template.md to reflect Docusaurus-specific rules.
-->
# Docusaurus Book Project Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### 1. Writing Tone & Style
- Use a friendly, simple, beginner-friendly writing style.
- Keep paragraphs short (2–4 sentences).
- Avoid academic or complex language.
- Maintain a consistent voice throughout all chapters.

### 2. Docusaurus Structure & Formatting Rules
- All book content MUST be stored inside: /content/chapters
- Every chapter must be a Markdown (.md or .mdx) file.
- Use valid Docusaurus-compatible Markdown syntax.
- Required chapter structure:
  - H1 Title at the top (`# Chapter Title`)
  - Short introduction section
  - Main content divided into H2 or H3 sections
  - Summary section at the end
- Use fenced code blocks ``` for examples.
- Use only Markdown features supported by Docusaurus.

### 3. File Naming Rules
- File names must be kebab-case, e.g., `introduction.md`, `chapter-1-getting-started.md`.
- No spaces, no uppercase letters.
- Each chapter file must start with a clear, descriptive name.

### 4. Sidebar & Navigation Conventions
- Every chapter must include proper frontmatter:
  ```yaml
  sidebar_label: Chapter Title
  sidebar_position: <number>
  description: A short, friendly description of the chapter.
  ```
- Titles must be clear and concise.
- Descriptions must be 1–2 friendly sentences.

### 5. AI Usage & Verification Rules
- No AI-generated text may be used without human review.
- All content must be fact-checked before acceptance.
- All claims must be accurate and clear.
- AI drafts must be improved and verified by a human before being marked final.

### 6. Quality & Editorial Standards
- All content must be original (no copying from external sources).
- Avoid jargon unless defined.
- Use examples and analogies when helpful.
- Keep content tightly focused on the chapter goal.
- Avoid filler, repetition, and long blocks of text.

### 7. Consistency Rules
- Use consistent terminology across all chapters.
- Follow the same formatting pattern everywhere.
- Maintain the same tone and voice from chapter to chapter.

### 8. Prohibited Content
- No overly long paragraphs.
- No unsupported claims.
- No broken Markdown syntax.
- No content that violates Docusaurus formatting capabilities.

## Governance
This Constitution defines strict global rules that the AI must follow for ALL tasks, including specification, planning, writing, editing, and implementation. All rules are unambiguous and enforceable across the entire writing project. Amendments to this constitution require documentation and approval.

**Version**: 1.0.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-03
<!-- Example: Version: 2.1.1 | Ratified: 2025-06-13 | Last Amended: 2025-07-16 -->

---
id: 1701700000000
title: create-docusaurus-book-tasks
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: docusaurus-book-spec
branch: 1-docusaurus-book-spec
user: user
command: /sp.tasks
labels: ["docusaurus","book","tasks","generation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- history/prompts/docusaurus-book-spec/1701700000000-create-docusaurus-book-tasks.tasks.prompt.md
tests:

---

## Prompt

i meant check if requirements are meant down below Using MCP context7. Output ONLY a set of /sp.task items (task list in JSON array and readable markdown). These tasks must be executable by an LLM worker or a developer.

INPUT: spec_id = {{SPEC_ID}} and optionally focus = {{FOCUS}} (e.g., "chapter", "example", "ci").

TASK ITEM FORMAT (for each task):
- id: sp.task.<NNNN>
- title: short title
- purpose: 1–2 sentences
- inputs: list of required inputs (context7 files, assets)
- steps: numbered step-by-step actions (each step must be atomic)
- expected_output: exact filename(s) and content summary
- path: exact repo path where file should be created
- token_chunk_hint: recommended max output chunk size for an LLM (e.g., 800–1200 tokens)
- dependencies: list of task ids
- acceptance_tests: 1–3 automated or manual checks
- owner: "LLM" or "Developer"

REQUIREMENTS:
- Break the SPEC into tasks that are each small enough for an LLM to complete (<=1200 tokens).
- Prioritize parallelizable tasks where possible.
- For code examples tasks include "run-instruction" and expected CLI commands to validate.
- Include at least 6 tasks for a typical Module SPEC (example IDs sp.task.0001..0006).
- Provide a JSON array of all tasks as the first output block, followed by a readable markdown table of tasks.

OUTPUT:
- Produce JSON array of tasks and then markdown table.
- No additional commentary.

## Response snapshot

Generated 6 tasks for creating a Docusaurus book, including initialization, content integration, configuration, sample chapter creation, building, and local testing. Tasks are provided in JSON and markdown table format.

## Outcome

- ✅ Impact: Tasks for Docusaurus book generation created.
- 🧪 Tests: No specific tests were run for task generation itself.
- 📁 Files: history/prompts/docusaurus-book-spec/1701700000000-create-docusaurus-book-tasks.tasks.prompt.md
- 🔁 Next prompts: The user can now proceed with implementing the generated tasks.
- 🧠 Reflection: The bash script for PHR creation failed, so agent-native tools were used. The task generation followed the specified format and requirements.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation failed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why the bash script failed or update the default PHR creation to use agent-native tools first.
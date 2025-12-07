---
id: 001
title: Update project constitution based on requirements
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-docusaurus-book-spec
user: user
command: /sp.constitution check if the file is meeting this requirement

1. Document metadata (frontmatter):
   - title: "/sp.constitution — Physical AI & Humanoid Robotics"
   - version: "1.0.0"
   - author: "Spec-Agent"

2. Sections required (exact headings):
   - Purpose & Scope
   - Writing Tone & Voice
   - Document Architecture & Docusaurus Rules
   - Module & Chapter Format (exact template)
   - Code & Example Rules (language, snippet size, runnable flags)
   - Diagrams & Assets (naming rules, alt-text, formats)
   - Specification & Plan Format (what /sp.specify, /sp.plan, /sp.task must include)
   - Acceptance Criteria & QA checks
   - Banned Behaviors & Hallucination Rules
   - Update / Versioning Policy

3. Style constraints (must appear verbatim in the constitution):
   - "Tone: formal, technical, modular, hands-on; no marketing or storytelling."
   - "Always reference MCP context7 for course facts before adding new facts."
   - "Never invent hardware or prices — use only items listed in context7."
   - "Each code block must include 'run-instruction' comment explaining how to execute."

4. Formatting constraints:
   - Use Markdown compatible with Docusaurus.
   - Provide a one-line YAML frontmatter example for Docusaurus pages.
   - Include at least one small example of the Module template (as markdown).

5. Output rules:
   - Produce the full constitution markdown document.
   - No extra explanation. If context7 is missing, include an ERROR header with instructions to attach context7.

if no generate the constitution now.
labels: ["governance", "constitution", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md (modified)
tests:
null
---

## Prompt

```text
check if the file is meeting this requirement

1. Document metadata (frontmatter):
   - title: "/sp.constitution — Physical AI & Humanoid Robotics"
   - version: "1.0.0"
   - author: "Spec-Agent"

2. Sections required (exact headings):
   - Purpose & Scope
   - Writing Tone & Voice
   - Document Architecture & Docusaurus Rules
   - Module & Chapter Format (exact template)
   - Code & Example Rules (language, snippet size, runnable flags)
   - Diagrams & Assets (naming rules, alt-text, formats)
   - Specification & Plan Format (what /sp.specify, /sp.plan, /sp.task must include)
   - Acceptance Criteria & QA checks
   - Banned Behaviors & Hallucination Rules
   - Update / Versioning Policy

3. Style constraints (must appear verbatim in the constitution):
   - "Tone: formal, technical, modular, hands-on; no marketing or storytelling."
   - "Always reference MCP context7 for course facts before adding new facts."
   - "Never invent hardware or prices — use only items listed in context7."
   - "Each code block must include 'run-instruction' comment explaining how to execute."

4. Formatting constraints:
   - Use Markdown compatible with Docusaurus.
   - Provide a one-line YAML frontmatter example for Docusaurus pages.
   - Include at least one small example of the Module template (as markdown).

5. Output rules:
   - Produce the full constitution markdown document.
   - No extra explanation. If context7 is missing, include an ERROR header with instructions to attach context7.

if no generate the constitution now.
```

## Response snapshot

Updated the `.specify/memory/constitution.md` file to meet the specified requirements, including frontmatter, section headings, style constraints, and formatting.

## Outcome

- ✅ Impact: Project constitution updated with new guidelines and formatting.
- 🧪 Tests: No tests were run for this action.
- 📁 Files: The constitution file was updated.
- 🔁 Next prompts: Ensure dependent templates are updated to align with the new constitution.
- 🧠 Reflection: Successfully updated the project constitution. The previous `create-phr.sh` script failed, so the PHR was created using agent-native tools. Need to ensure dependent templates are updated.

## Evaluation notes (flywheel)

- Failure modes observed: Failed to execute `create-phr.sh` script.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null

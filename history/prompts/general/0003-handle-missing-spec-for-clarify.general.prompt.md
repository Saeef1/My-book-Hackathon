---
id: 0003
title: Handle Missing Spec for Clarify
stage: general
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-python-data-science-book
user: unknown
command: /sp.clarify
labels: ["clarification", "error_handling"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:

tests:

---

## Prompt

/sp.clarify

## Response snapshot

Informed the user that the `spec.md` file is missing and instructed them to run `/sp.specify` first, as per the `/sp.clarify` tool's guidelines for handling missing spec files.

## Outcome

- ✅ Impact: Correctly handled a prerequisite failure for the `/sp.clarify` command, guiding the user on the next necessary step.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: User to run `/sp.specify`.
- 🧠 Reflection: Successfully followed the explicit instructions for a missing spec file within the `/sp.clarify` workflow, preventing further errors.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

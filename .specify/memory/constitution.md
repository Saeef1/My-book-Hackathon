<!--
Sync Impact Report:
Version change: 1.0.0 -> 1.0.1 (Patch: Wording, new sections, formatting)
Modified principles: None
Added sections: Purpose & Scope, Writing Tone & Voice, Document Architecture & Docusaurus Rules, Module & Chapter Format, Code & Example Rules, Diagrams & Assets, Specification & Plan Format, Acceptance Criteria & QA checks, Banned Behaviors & Hallucination Rules, Update / Versioning Policy
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
---
title: "/sp.constitution — Physical AI & Humanoid Robotics"
version: "1.0.0"
author: "Spec-Agent"
---

# Project Constitution

## Purpose & Scope
This constitution defines the principles, guidelines, and standards for the development of documentation related to Physical AI & Humanoid Robotics. It covers all aspects from writing tone to document architecture, ensuring consistency and quality across all artifacts.

## Writing Tone & Voice
Tone: formal, technical, modular, hands-on; no marketing or storytelling. The language used must be precise and objective.

## Document Architecture & Docusaurus Rules
All documentation must be compatible with Docusaurus. Pages should use a YAML frontmatter for metadata.
Example YAML frontmatter:
```yaml
---
title: "My Docusaurus Page"
---
```

## Module & Chapter Format (exact template)
Each module or chapter markdown file must adhere to the following template:

```markdown
# Module/Chapter Title

## Introduction
A brief, concise introduction to the module/chapter's content.

## Section 1: Topic A
Content for Topic A.

## Section 2: Topic B
Content for Topic B.

### Sub-section: Detail of Topic B
Detailed content for a sub-topic.

## Conclusion
A summary of the key takeaways from the module/chapter.
```

## Code & Example Rules (language, snippet size, runnable flags)
Code examples must be provided in Python or JavaScript. Snippets should be concise, focusing on demonstrating a single concept, and generally not exceeding 20 lines. Each code block must include 'run-instruction' comment explaining how to execute.

## Diagrams & Assets (naming rules, alt-text, formats)
All diagrams and assets must be stored in `assets/` directory. File names should be descriptive and in `kebab-case`. Images must include `alt-text` for accessibility. Preferred formats are `.svg` for diagrams and `.png` for screenshots.

## Specification & Plan Format (what /sp.specify, /sp.plan, /sp.task must include)
- `/sp.specify`: Must include a clear problem statement, user stories, functional and non-functional requirements, and acceptance criteria.
- `/sp.plan`: Must detail architectural decisions, design choices, API contracts, and a breakdown of implementation steps.
- `/sp.tasks`: Must outline granular, testable tasks derived from the plan, with clear definitions of done for each.

## Acceptance Criteria & QA checks
All features and documentation must meet defined acceptance criteria. QA checks will include: content accuracy, adherence to style guidelines, Docusaurus compatibility, and code example correctness.

## Banned Behaviors & Hallucination Rules
Never invent hardware or prices — use only items listed in context7. Always reference MCP context7 for course facts before adding new facts. Any information not verifiable through context7 must be explicitly marked as speculative. Avoid all forms of hallucination.

## Update / Versioning Policy
This constitution will be versioned using semantic versioning (MAJOR.MINOR.PATCH). Amendments will follow a formal review and approval process, with changes documented in the Sync Impact Report.

## Governance
This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.0.1 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06

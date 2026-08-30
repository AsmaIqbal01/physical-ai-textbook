---
id: 0001
title: Repo-wide error audit
stage: general
date: 2026-08-30
surface: agent
model: claude-sonnet-5
feature: none
branch: main
user: azeecreations1@gmail.com
command: N/A
labels: [audit, broken-links, tooling-bug, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - README.md (created)
 - .specify/templates/plan-template.md (fixed broken reference)
 - .claude/commands/sp.constitution.md (fixed broken reference)
tests:
 - bash -n on all .specify/scripts/bash/*.sh (all pass)
 - ran .specify/scripts/bash/check-prerequisites.sh --json (reproduced failure)
 - ran .specify/scripts/bash/create-phr.sh --stage spec (reproduced failure)
---

## Prompt

Analyze this entire repo and find all errors:- Broken links- Missing files referenced in code- Syntax errors- Incomplete content- Any README issues- Any config/build errorsList everything you find, fix what you can automatically, then tell me what needs manual attention.Do not touch anything outside this folder.

## Response snapshot

Audited the full repository (no code yet — this is an SDD spec/tooling scaffold for a Docusaurus textbook). Findings:

1. **Broken doc references (fixed)**: `.specify/templates/plan-template.md` and `.claude/commands/sp.constitution.md` pointed to a nonexistent `.specify/templates/commands/*.md` path; corrected to the real location `.claude/commands/sp.*.md`.
2. **Missing README (fixed)**: repo had no README.md at all; added one summarizing project purpose, structure, and status.
3. **Tooling bug (verified by running scripts, not auto-fixed — needs a decision)**: `.specify/scripts/bash/common.sh`'s `check_feature_branch` requires branch names matching `^[0-9]{3}-`, and `create-phr.sh`'s feature auto-detection requires `specs/<NNN-name>` directories. This repo's actual feature lives at `specs/book-skeleton/` on branch `main` (no numeric prefix), so `check-prerequisites.sh` (used by `/sp.plan`, `/sp.tasks`, `/sp.clarify`, `/sp.analyze`, `/sp.checklist`, `/sp.implement`, `/sp.adr`) and `create-phr.sh` for feature-scoped stages both hard-fail (exit 1), reproduced directly. This is why no `history/prompts/` records existed despite CLAUDE.md mandating them for every prompt.
4. **Missing referenced files**: `.specify/memory/constitution.md` references `specs/chapter-template.md`, which does not exist. `decisions.md` marks "Create appendices.md outline" as done (✅) but no `appendices.md` exists anywhere in the repo.
5. **Content inconsistencies**: `decisions.md` checks off "Update spec.md" and "Update book-structure.md" with the approved tiered Module 3 (Basic/Advanced) and dual-LLM (Ollama/OpenAI) decisions, but neither `spec.md` nor `book-structure.md` actually reflects those decisions — Module 3 and the FR list are still single-path. `book-structure.md` sets "Final Submission: November 30, 2025" despite the spec/decisions/clarifications all being dated December 5–6, 2025 (the deadline predates the planning docs).
6. **Expected-but-not-yet-built structure**: `spec.md`/`constitution.md` describe a full Docusaurus skeleton (`docs/`, `static/`, `sidebars.js`, `docusaurus.config.js`, `package.json`) — none of it exists yet, and no `plan.md`/`tasks.md` exist for the `book-skeleton` feature either. This isn't a bug so much as the project simply not having progressed past the spec stage.
7. No syntax errors (`bash -n` clean on all scripts), no JSON/YAML config files exist to check, and no markdown-style relative links (`[text](path)`) were found anywhere to check for 404s.

## Outcome

- ✅ Impact: Two broken doc cross-references fixed, root README added; five categories of pre-existing issues identified and clearly attributed for manual follow-up.
- 🧪 Tests: `bash -n` on all shell scripts (pass); reproduced `check-prerequisites.sh` and `create-phr.sh` failures live.
- 📁 Files: README.md (new), .specify/templates/plan-template.md, .claude/commands/sp.constitution.md.
- 🔁 Next prompts: Decide on numbered vs. unnumbered feature directories to unblock `/sp.plan`/`/sp.tasks`; reconcile spec.md/book-structure.md with decisions.md's approved tiered-path decisions; create or drop the appendices.md/chapter-template.md commitments; correct or reissue the November 30, 2025 deadline.
- 🧠 Reflection: The project's own tooling assumes a numbered-feature convention that its first real feature violated, silently breaking the PHR mandate from day one — worth surfacing to the user as the highest-leverage fix.

## Evaluation notes (flywheel)

- Failure modes observed: none in this session; audit was read-mostly with two safe, verified doc fixes.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

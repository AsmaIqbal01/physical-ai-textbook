# Physical AI & Humanoid Robotics Textbook

A spec-first textbook project for Physical AI and Humanoid Robotics (ROS 2, digital twins, NVIDIA Isaac, and Vision-Language-Action integration), built using the [SpecifyPlus](.specify/) spec-driven development workflow.

## What exists now

- **`.specify/memory/constitution.md`** — the project's governing principles (educational clarity, theory + practice, spec-driven content, etc.)
- **`specs/book-skeleton/spec.md`** — the feature spec for the book's skeleton structure, plus supporting `clarifications.md`, `decisions.md`, and `book-structure.md`
- **`.specify/scripts/bash/`** — shell scripts that power the spec-driven workflow (creating features, prerequisite checks, PHR records, etc.)
- **`.specify/templates/`** — templates for specs, plans, tasks, ADRs, and Prompt History Records
- **`.claude/commands/`** — the `/sp.*` slash commands (`/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`, etc.) that drive the workflow
- **`history/prompts/`** — Prompt History Records (PHRs) capturing prompts and outcomes as work progresses

## What is planned

- A Docusaurus-based website (`docs/`, `static/`, `sidebars.js`, `docusaurus.config.js`, `package.json`) generated from the book-skeleton spec
- Actual chapter content across the book's modules (ROS 2, digital twins, NVIDIA Isaac, Vision-Language-Action integration) and the capstone project
- An implementation plan and task list (`plan.md`, `tasks.md`) for the `book-skeleton` feature, to be generated via `/sp.plan` and `/sp.tasks`

## Workflow

This project follows Spec-Driven Development: a feature starts as a spec under `specs/`, moves through clarification and planning, and is implemented only once approved. See `.specify/memory/constitution.md` for the full principles and `.claude/commands/` for the available `/sp.*` commands.

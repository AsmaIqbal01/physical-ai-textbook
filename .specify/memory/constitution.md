# Physical AI & Humanoid Robotics Textbook Constitution

<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Refocused constitution from 7 detailed principles to 5 streamlined core principles
- Modified principles:
  - Collapsed "Educational Clarity First" + "Progressive Skill Building" + "Multi-Modal Learning Support" → "Educational Clarity"
  - Collapsed "Theory-Practice Integration" + "Industry-Relevant Content" → "Theory + Practice"
  - Removed "Industry-Relevant Content" as standalone (integrated into Theory + Practice)
  - Removed "Progressive Skill Building" as standalone (integrated into Educational Clarity)
  - Removed "Multi-Modal Learning Support" as standalone (integrated into Educational Clarity)
  - Renamed "Rigor and Correctness" → "Quality Standards" section
  - Renamed "Open and Extensible" → "Consistency & Modularity"
  - Added "AI-Assisted, Human-Supervised" principle
  - Added "Version-Controlled" principle
- Added sections:
  - Simplified Quality Standards section
  - Simplified Structure section with explicit Docusaurus integration
  - Simplified Constraints section
  - Simplified Success Criteria section
- Removed sections:
  - Content Quality Standards (depth/coverage details removed)
  - Review and Validation (simplified into Quality Standards)
  - Development Workflow (streamlined)
  - Collaboration and Version Control (merged into Version-Controlled principle)
  - Testing and Validation (merged into Quality Standards)
- Templates requiring updates:
  ✅ constitution.md - updated
  ⚠ plan-template.md - Review "Constitution Check" section to align with new 5 principles
  ⚠ spec-template.md - Verify requirements sections align with educational content focus
  ⚠ tasks-template.md - Verify task categorization reflects educational content development
- Follow-up TODOs:
  - Review plan-template.md Constitution Check gates
  - Ensure templates support Docusaurus-specific needs (docs/, sidebars.js, static/)
-->

## Core Principles

### I. Specs First

All textbook content MUST originate from an approved specification document. No chapter, section, or exercise MUST be created without a corresponding spec that defines its purpose, learning objectives, scope, and success criteria.

**Rationale**: Spec-driven development ensures intentional content design, prevents scope creep, maintains consistency, and provides clear acceptance criteria for content validation.

**Non-negotiable rules**:
- Every chapter has a spec file in `specs/<chapter-name>/spec.md`
- Spec MUST define learning objectives, prerequisites, scope, and exercises
- Content implementation follows spec exactly; deviations require spec amendment
- No content merges without spec approval

### II. Educational Clarity

All content MUST prioritize student understanding. Explanations MUST be clear, concise, and structured for progressive learning—starting with intuitive examples before introducing mathematical formulations or complex implementations.

**Rationale**: Students come from diverse backgrounds. Content must scaffold learning from fundamentals to advanced topics while maintaining technical rigor and supporting multiple learning modalities.

**Non-negotiable rules**:
- Introduce concepts with real-world examples before formal definitions
- Include visual aids (diagrams, mermaid charts, illustrations) for complex topics
- Provide worked examples for every major concept
- Use consistent terminology throughout all chapters
- Support multiple learning modalities (text, visuals, code, exercises)
- Ensure accessibility (alt text, clear diagrams, color-blind-safe palettes)

### III. Theory + Practice

Each chapter MUST balance theoretical foundations with hands-on practical applications. Theory sections MUST be accompanied by practical exercises, code examples, simulations, or lab activities that reinforce concepts.

**Rationale**: Physical AI and robotics require both conceptual understanding and practical skills. Students need to see how theory translates to working systems.

**Non-negotiable rules**:
- Every theoretical section includes at least one practical application
- Lab exercises reference specific theoretical concepts
- Code examples MUST be tested, functional, and include explanatory comments
- Simulations MUST be reproducible with documented environments (Python versions, dependencies)
- Use pseudocode for algorithms before implementation details
- Include real-world examples from robotics and AI applications

### IV. AI-Assisted, Human-Supervised

AI tools (including Spec-Kit Plus and Claude Code) MUST be used to accelerate content creation, but all outputs MUST be validated by subject matter experts. AI-generated content serves as a draft; human review ensures accuracy, coherence, and educational quality.

**Rationale**: AI dramatically speeds content generation, but educational materials require expert validation to ensure technical accuracy, pedagogical soundness, and alignment with learning objectives.

**Non-negotiable rules**:
- All AI-generated content flagged for human review before finalization
- Subject matter experts validate technical accuracy (math, algorithms, code)
- Educators validate pedagogical approach and clarity
- All code examples tested in clean environments before publication
- Sources cited; no unverified claims; no AI hallucinations allowed

### V. Consistency & Modularity

Content MUST maintain uniform structure, tone, and formatting across all chapters. Each chapter MUST be modular—independently understandable yet coherent with the full textbook. Content MUST be version-controlled and easy to update.

**Rationale**: Consistency enhances learning; modularity enables flexible course design and incremental updates. Version control provides traceability and collaboration support.

**Non-negotiable rules**:
- All chapters follow the same structure template
- Consistent heading hierarchy, terminology, and notation throughout
- Chapters are self-contained with clear prerequisites listed
- Modular design allows instructors to reorder or skip chapters
- All content in Git; branching strategy for chapters; pull requests for major changes
- Commit messages reference spec IDs and follow convention

## Quality Standards

### Accuracy and Rigor

All technical content—mathematical formulations, algorithms, code implementations—MUST be technically accurate and verified. Sources MUST be cited; claims MUST be evidence-based.

**Standards**:
- Mathematical derivations reviewed for correctness
- Code tested in documented environments (Python versions, dependency manifests)
- Research claims cite peer-reviewed sources or authoritative industry references
- No AI hallucinations; validate all generated content
- Errata process established for post-publication corrections

### Markdown and Docusaurus Compatibility

All content MUST be written in Markdown compatible with Docusaurus. Formatting MUST render correctly in the static site build.

**Standards**:
- Use Docusaurus-compatible Markdown syntax
- Diagrams use mermaid syntax or SVG/PNG in `static/` directory
- Code blocks specify language for syntax highlighting
- Clear headings, logical flow, production-ready formatting
- All internal links use relative paths; external links validated

### Readability and Structure

Content MUST have clear headings, logical flow, and be production-ready. Use pseudocode for algorithms, diagrams for systems, and examples for concepts.

**Standards**:
- Clear section headings following logical progression
- Introductory paragraph for each major section
- Examples accompany every major concept
- Diagrams include descriptive captions and alt text
- Mathematical notation accompanied by plain-language descriptions

## Structure

### Chapter Organization

Chapters are developed as separate specs under `specs/<chapter-name>/`. Each spec defines:
- **Purpose**: What topic is covered and why it matters
- **Learning Outcomes**: What students will be able to do after this chapter
- **Content Outline**: Sections, subsections, key concepts
- **Examples and Exercises**: Practical applications and assessments
- **References**: Academic papers, textbooks, industry resources

### Docusaurus Integration

The textbook is built with Docusaurus and deployed to GitHub Pages. Content structure:

```
docs/                  # Markdown chapter files
├── intro.md
├── chapter-01/
│   ├── index.md
│   └── exercises.md
├── chapter-02/
│   ├── index.md
│   └── exercises.md
sidebars.js            # Navigation structure
static/                # Images, diagrams, assets
├── img/
└── diagrams/
```

**Requirements**:
- Each chapter has a directory under `docs/`
- `sidebars.js` defines navigation and ordering
- Images and diagrams stored in `static/` with descriptive filenames
- Build tested locally before deployment (`npm run build`)
- Deployed to GitHub Pages on main branch merges

## Constraints

### Spec-Driven Only

Content MUST only be generated from approved specs. No ad-hoc content creation.

**Enforcement**:
- Spec file MUST exist before content implementation begins
- Spec reviewed and approved by project lead or SME before development
- Content implementation references spec ID in commit messages

### No Unverified Claims

All technical claims MUST be verified. No reliance on AI-generated content without validation.

**Enforcement**:
- All code examples tested and functional
- All mathematical derivations reviewed
- All research claims cited with sources
- Human review required before publication

### Structure and Tone Consistency

All content MUST follow the established structure and tone defined in templates.

**Enforcement**:
- Chapter template defined in `specs/chapter-template.md`
- Tone: clear, professional, educational—avoid colloquialisms
- Formatting guidelines in `.specify/templates/`

## Success Criteria

The constitution is successfully applied when:

- **Specs Implemented into Markdown**: All approved specs have corresponding Markdown chapters in `docs/`
- **Content Passes Readability and Accuracy Checks**: SME review confirms technical accuracy; educator review confirms pedagogical quality
- **Examples and Diagrams Included**: Every chapter has worked examples, code snippets, and visual aids
- **Docusaurus Build Works**: `npm run build` succeeds without errors; site renders correctly
- **Deployed to GitHub Pages**: Main branch automatically deploys to live site; links functional
- **Maintainable and Extensible**: Modular chapters allow updates without breaking other content; version-controlled with clear commit history

## Governance

### Amendment Process

This constitution governs textbook development practices. Amendments require:
1. Proposal with rationale and impact analysis
2. Review by project lead and core contributors
3. Consensus approval (or 2/3 majority if consensus not reached)
4. Documentation in ADR (Architecture Decision Record) if architecturally significant
5. Update to dependent templates and workflows

### Compliance and Quality Assurance

- All chapter development MUST follow the Spec-Driven process
- All content MUST meet Quality Standards before publication
- Principle violations MUST be justified in writing and documented
- Regular retrospectives to assess adherence and refine practices

### Version Control

Constitution versioning follows semantic versioning:

- **MAJOR**: Backward incompatible changes (removed principles, restructured governance)
- **MINOR**: New principles, expanded guidance, additional sections
- **PATCH**: Corrections, clarifications, wording improvements

**Version**: 1.1.0
**Ratified**: 2025-12-05
**Last Amended**: 2025-12-06

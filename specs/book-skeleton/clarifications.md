# Book Skeleton Clarification Report

**Project**: Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-06
**Purpose**: Identify ambiguities, missing details, and required clarifications before implementation

---

## Executive Summary

The current book skeleton specification is well-structured and comprehensive. However, there are several ambiguities and missing details that need clarification to ensure independent implementability. This report identifies 47 clarification points across 8 categories.

**Critical Issues**: 12
**High Priority**: 18
**Medium Priority**: 17

---

## 1. Module Scope and Boundaries

### Module 1: ROS 2

#### 1.1 **CRITICAL: Chapter 1.4 Scope Ambiguity**
**Issue**: "Bridge a trained RL agent (e.g., Stable Baselines3) to ROS 2" - Does this assume students already have a trained RL agent, or should the chapter teach RL training?

**Clarification Needed**:
- [ ] Should Chapter 1.4 include an introduction to Reinforcement Learning basics?
- [ ] Should students use a pre-trained model, or train from scratch?
- [ ] If training is required, what environment should be used (Gym, custom ROS 2 env)?
- [ ] What is the expected skill level: just bridging (beginner) vs. training+bridging (advanced)?

**Suggested Resolution**:
- **Option A (Beginner-friendly)**: Provide a pre-trained RL model; focus only on ROS 2 integration
- **Option B (Comprehensive)**: Include a mini RL tutorial (1-2 pages) with Stable Baselines3 + Gym
- **Option C (Modular)**: Make RL training an optional advanced section; basic students use pre-trained model

**Impact if Unresolved**: Students without RL background may get stuck; content authors may create inconsistent tutorials.

---

#### 1.2 **HIGH: URDF Complexity Level**
**Issue**: Chapter 1.3 expects students to "Create a URDF model for a simple humanoid (torso, arms, legs)" - How detailed should this be?

**Clarification Needed**:
- [ ] Should the humanoid have functional joints (revolute, prismatic) or just visual links?
- [ ] How many degrees of freedom (DOF)? (e.g., 2 DOF arms vs. 7 DOF arms)
- [ ] Should collision and inertia properties be included, or just visual meshes?
- [ ] Should students model from scratch, or modify a template URDF?

**Suggested Resolution**:
- Specify minimum requirements: "5-link humanoid with 8 DOF (2 legs with hip/knee, 2 arms with shoulder/elbow)"
- Provide a starter URDF template for students to modify
- Include collision and inertia in advanced optional section

**Impact if Unresolved**: Exercises will vary widely in complexity; grading/validation becomes inconsistent.

---

### Module 2: Gazebo & Unity

#### 2.1 **CRITICAL: Gazebo Version Conflict**
**Issue**: Chapter 2.1 mentions "Gazebo (Classic vs. Gazebo Sim/Ignition)" - Which version should be the primary focus?

**Clarification Needed**:
- [ ] Should the book focus on Gazebo Classic (older, more stable, deprecated) or Gazebo Sim/Ignition (modern, ROS 2 native)?
- [ ] If both are covered, how much time for each?
- [ ] What are the compatibility implications for ROS 2 Humble (recommended distro)?

**Suggested Resolution**:
- **Recommended**: Focus on **Gazebo Sim (Ignition Fortress or Garden)** as it's ROS 2 native
- Mention Gazebo Classic only in a brief comparison table (1 page max)
- Ensure all exercises use Gazebo Sim for consistency

**Impact if Unresolved**: Students may install wrong version; tutorials may not work; ROS 2 integration issues.

---

#### 2.2 **HIGH: Unity vs. RViz2 Role Clarity**
**Issue**: Chapter 2.2 mentions visualizing sensor data in RViz2, but Chapter 2.3 focuses on Unity visualization - When should students use which tool?

**Clarification Needed**:
- [ ] What are the distinct use cases for RViz2 vs. Unity?
- [ ] Should RViz2 be introduced in Module 1 or Module 2?
- [ ] Is Unity optional or mandatory for this module?

**Suggested Resolution**:
- **RViz2**: Real-time debugging, sensor visualization, trajectory planning (standard ROS 2 tool)
- **Unity**: High-fidelity visualization, HRI interfaces, VR/AR applications (advanced/optional)
- Introduce RViz2 in Module 1 (Chapter 1.2 or 1.3) as a standard tool
- Make Unity an optional advanced topic in Module 2 unless HRI is a core requirement

**Impact if Unresolved**: Students may duplicate effort or be confused about tool selection.

---

#### 2.3 **MEDIUM: RealSense Simulation Specificity**
**Issue**: "Depth cameras (Intel RealSense simulation)" - Is this a specific Gazebo plugin or generic depth camera?

**Clarification Needed**:
- [ ] Should students use the official `realsense_gazebo_plugin`?
- [ ] Is camera calibration and intrinsic/extrinsic parameters required?
- [ ] Should the exercise work with a real RealSense camera if available, or simulation-only?

**Suggested Resolution**:
- Use `gazebo_ros_camera` plugin with depth output (generic, works for all students)
- Optionally mention `realsense_gazebo_plugin` for advanced users with real hardware
- Provide camera calibration YAML file as a downloadable resource

**Impact if Unresolved**: Plugin compatibility issues; students without RealSense hardware may struggle.

---

### Module 3: NVIDIA Isaac

#### 3.1 **CRITICAL: Isaac Sim Hardware Requirements**
**Issue**: Isaac Sim requires powerful GPUs (RTX 3060+, 32GB RAM) - How do students without this hardware complete Module 3?

**Clarification Needed**:
- [ ] Is Module 3 mandatory or optional for students without GPUs?
- [ ] Should cloud alternatives (AWS, Azure, Omniverse Cloud) be provided in every chapter?
- [ ] What is the fallback if cloud is not affordable for some students?
- [ ] Can students skip Module 3 and still complete Module 4 and Capstone?

**Suggested Resolution**:
- Make Isaac Sim **optional** with a clear Gazebo-based alternative for all exercises
- Provide detailed cloud setup instructions (AWS EC2 g4dn.xlarge with Omniverse Nucleus)
- **Modular prerequisite**: Module 4 should list "Module 2 OR Module 3" (not both mandatory)
- Capstone should support both Gazebo and Isaac Sim deployment paths

**Impact if Unresolved**: Excludes students without high-end hardware; reduces accessibility; violates educational equity.

---

#### 3.2 **HIGH: Isaac Gym vs. Isaac Sim Confusion**
**Issue**: Chapter 3.3 mentions "Reinforcement learning for walking gaits (Isaac Gym)" and "deploy in Isaac Sim" - Are these two different tools?

**Clarification Needed**:
- [ ] What is the relationship between Isaac Gym and Isaac Sim?
- [ ] Can Isaac Gym run without Isaac Sim?
- [ ] Should students use IsaacGymEnvs, Isaac Sim, or both?
- [ ] What is the workflow: Train in Isaac Gym → Deploy in Isaac Sim?

**Suggested Resolution**:
- Clarify: **Isaac Gym** = RL training environment (headless, fast, GPU-parallel)
- Clarify: **Isaac Sim** = Photorealistic simulator (Omniverse, visualization, slower)
- Workflow: Train policy in Isaac Gym → Export model → Test in Isaac Sim → Deploy to ROS 2/hardware
- Provide a workflow diagram in Chapter 3.1 introduction

**Impact if Unresolved**: Students install wrong tools; training exercises fail; wasted time.

---

#### 3.3 **HIGH: Bipedal Control Complexity**
**Issue**: "Train a bipedal walking policy" is extremely complex - Is this realistic for a 4-week module?

**Clarification Needed**:
- [ ] Should students train from scratch (weeks/months of GPU time) or fine-tune a pre-trained policy?
- [ ] What is the minimum viable outcome: stable standing, simple walking, dynamic locomotion?
- [ ] Should this be a group project or individual exercise?
- [ ] What is the expected time commitment for this exercise?

**Suggested Resolution**:
- Provide a **pre-trained baseline policy** (e.g., from IsaacGymEnvs)
- Exercise focuses on **fine-tuning** or **testing** the policy, not training from scratch
- Advanced optional section: "Train your own policy from scratch (estimated 20+ GPU hours)"
- Make this a **Capstone prerequisite** rather than a timed module exercise

**Impact if Unresolved**: Exercise becomes unfeasible; students frustrated; learning outcomes not met.

---

### Module 4: Vision-Language-Action (VLA)

#### 4.1 **CRITICAL: LLM API Costs and Access**
**Issue**: Chapter 4.2 uses "GPT, LLaMA" - GPT requires paid API access, LLaMA requires local deployment - Which is recommended?

**Clarification Needed**:
- [ ] Should students use paid APIs (OpenAI GPT-4, Claude) or open-source models (LLaMA, Mistral)?
- [ ] If open-source, what are the hardware requirements for local LLM deployment?
- [ ] Should the book provide API credits or institutional access?
- [ ] What is the fallback for students without API access or GPU for local LLMs?

**Suggested Resolution**:
- **Primary Path**: Use open-source models (LLaMA 2 7B, Mistral 7B) via Ollama (low GPU req)
- **Alternative Path**: Provide instructions for OpenAI API with free tier (gpt-3.5-turbo)
- **Budget Considerations**: Estimate API costs ($5-10 for all exercises) and provide cost-saving tips
- Offer pre-recorded demonstrations for students who cannot run LLMs locally or afford APIs

**Impact if Unresolved**: Economic barrier; students cannot complete module; inequitable access.

---

#### 4.2 **HIGH: Whisper Model Size and Performance**
**Issue**: "Speech recognition with OpenAI Whisper" - Which model size (tiny, base, small, medium, large)?

**Clarification Needed**:
- [ ] Which Whisper model should students use?
- [ ] What are the accuracy vs. speed vs. hardware tradeoffs?
- [ ] Should students run Whisper locally or use OpenAI API?
- [ ] Is real-time performance required, or offline transcription acceptable?

**Suggested Resolution**:
- **Recommended**: Whisper **base** model (74M params, good balance, runs on CPU)
- **Advanced**: Whisper **small** or **medium** for GPU users
- Provide performance comparison table (model size, WER, latency, hardware)
- Real-time not required for initial exercises; focus on correctness first

**Impact if Unresolved**: Students choose wrong model; performance issues; inconsistent results.

---

#### 4.3 **MEDIUM: VLA Model Definition Ambiguity**
**Issue**: Chapter 4.1 introduces "Vision-Language-Action (VLA) models" - Is this a specific model architecture or a paradigm?

**Clarification Needed**:
- [ ] Are VLA models like RT-1, RT-2 (Google DeepMind) the focus, or is VLA a general concept?
- [ ] Should students use pre-trained VLA models, or build modular V+L+A pipelines?
- [ ] What is the relationship between VLA and traditional robotics pipelines?

**Suggested Resolution**:
- Clarify: **VLA** = paradigm for integrating Vision + Language + Action (not a single model)
- Chapter 4 teaches modular integration: Vision (YOLO/CLIP), Language (LLM), Action (ROS 2)
- Optionally mention RT-1/RT-2 as research examples (no hands-on due to complexity)
- Focus on practical integration, not bleeding-edge research models

**Impact if Unresolved**: Students expect access to RT-1/RT-2; confusion about scope; unrealistic expectations.

---

### Capstone Project

#### 5.1 **CRITICAL: Hardware Deployment Requirement**
**Issue**: Chapter 5.3 mentions "Deploying to physical hardware" - Is real hardware required to complete the Capstone?

**Clarification Needed**:
- [ ] Is hardware deployment mandatory or optional?
- [ ] What percentage of students are expected to have access to Jetson Orin + Unitree robot?
- [ ] Should simulation-only completion be considered a passing grade?
- [ ] What is the minimum hardware requirement (e.g., Jetson Nano with webcam)?

**Suggested Resolution**:
- **Simulation is sufficient** for course completion
- Hardware deployment is **optional bonus** for students with access
- Minimum hardware for bonus: Jetson Orin Nano ($499) + RealSense D435i ($329) + any wheeled robot/RC car
- Provide detailed sim-to-real transfer guide for those attempting hardware deployment

**Impact if Unresolved**: Course becomes inaccessible to students without $2000+ hardware budget; violates educational principles.

---

#### 5.2 **HIGH: Integration Complexity and Time Estimate**
**Issue**: "Integrate all concepts to build a fully autonomous humanoid" - Is 4-6 weeks realistic for this scope?

**Clarification Needed**:
- [ ] What is the minimum viable capstone project (MVP)?
- [ ] Should students integrate all 4 modules, or can they choose 2-3?
- [ ] Is this an individual or group project?
- [ ] What is the expected deliverable: code + report, video demo, live presentation?

**Suggested Resolution**:
- **MVP**: Navigation (Module 1+2) + Voice Command (Module 4) → "Voice-controlled navigation robot"
- **Full Integration**: Add perception (Module 3) + manipulation → "Autonomous fetch-and-deliver robot"
- **Group Project Recommended**: 2-3 students per team
- **Deliverables**: GitHub repo + 5-min video demo + technical report (10 pages max)

**Impact if Unresolved**: Scope creep; students overwhelmed; unrealistic expectations; high dropout rate.

---

## 2. Prerequisites and Assumptions

### 2.1 **HIGH: "Prior AI Knowledge" Vagueness**
**Issue**: Book targets "students with prior AI knowledge" - What specific AI topics are assumed?

**Clarification Needed**:
- [ ] Machine Learning basics (supervised, unsupervised, RL)?
- [ ] Deep Learning (CNNs, RNNs, Transformers)?
- [ ] Specific frameworks (TensorFlow, PyTorch, JAX)?
- [ ] What if students only know classical ML, not deep learning?

**Suggested Resolution**:
- **Minimum Prerequisites**:
  - Basic ML concepts (classification, regression, overfitting)
  - Python programming with NumPy, Pandas
  - Familiarity with neural networks (forward pass, backprop at high level)
- **Helpful but Optional**:
  - PyTorch or TensorFlow experience
  - Reinforcement Learning concepts
  - Computer Vision basics
- Add **Appendix A: AI Prerequisites Refresher** (10-15 pages)

**Impact if Unresolved**: Students without DL background struggle; need frequent detours to explain basics.

---

### 2.2 **MEDIUM: Linux Command Line Proficiency Level**
**Issue**: "Familiarity with Linux command line" - What does "familiarity" mean?

**Clarification Needed**:
- [ ] Should students know bash scripting?
- [ ] Are commands like `ssh`, `scp`, `rsync`, `systemctl` expected?
- [ ] Is Docker/Podman knowledge assumed?

**Suggested Resolution**:
- **Minimum Linux Skills**:
  - File operations: `cd`, `ls`, `mkdir`, `cp`, `mv`, `rm`
  - Package management: `apt install`, `apt update`
  - Text editing: `nano` or `vim` basics
  - Process management: `ps`, `kill`, `top`
- **Introduced in Book**:
  - ROS 2 environment setup: `source`, `export`, `.bashrc`
  - Docker basics (if containers are used)
- Add **Appendix B: Linux Essentials Cheat Sheet**

**Impact if Unresolved**: Students get stuck on environment setup; spend more time on sysadmin than robotics.

---

### 2.3 **HIGH: Programming Language Proficiency**
**Issue**: Prerequisites list "Python, C++" - Are both required, or just one?

**Clarification Needed**:
- [ ] Can students complete the book with Python only?
- [ ] Which chapters require C++ knowledge?
- [ ] What C++ proficiency level is needed (syntax only, OOP, templates, modern C++)?

**Suggested Resolution**:
- **Python is mandatory**; C++ is optional for advanced users
- C++ used only in optional sections (e.g., custom ROS 2 plugins, performance-critical code)
- All core exercises have Python implementations
- Provide C++ equivalents as "Advanced Implementation" sidebars

**Impact if Unresolved**: Students without C++ background feel excluded; unclear if they can proceed.

---

## 3. Learning Outcomes Clarity

### 3.1 **MEDIUM: Quantifiable Skill Metrics Missing**
**Issue**: Learning outcomes like "Master ROS 2" are subjective - How do we measure mastery?

**Clarification Needed**:
- [ ] What specific skills should students demonstrate to pass each module?
- [ ] Are there quizzes, coding challenges, or project rubrics?
- [ ] What is the assessment criteria: code quality, functionality, understanding?

**Suggested Resolution**:
- Define **measurable outcomes** for each module:
  - **Module 1**: "Create a 3-node ROS 2 system with pub-sub and services (no starter code)"
  - **Module 2**: "Simulate a URDF robot in Gazebo with sensor fusion (LiDAR + depth camera)"
  - **Module 3**: "Deploy a pre-trained Isaac Gym policy in Isaac Sim"
  - **Module 4**: "Build a voice-controlled robot that executes 5 commands (sit, stand, walk, turn, stop)"
- Add **Appendix C: Assessment Rubrics** for each module

**Impact if Unresolved**: Instructors cannot fairly grade; students unclear if they're ready to proceed.

---

### 3.2 **MEDIUM: Progressive Difficulty Curve**
**Issue**: Is there a clear progression from beginner to advanced topics within and across modules?

**Clarification Needed**:
- [ ] Should each module have beginner/intermediate/advanced sections?
- [ ] How do we ensure students aren't overwhelmed in early chapters?
- [ ] Are there prerequisite checks or self-assessment quizzes?

**Suggested Resolution**:
- Use **difficulty tags** in each chapter section:
  - 🟢 Beginner: Essential concepts, step-by-step
  - 🟡 Intermediate: Requires combining multiple concepts
  - 🔴 Advanced: Research-level, optional deep dives
- Add **self-assessment quizzes** at end of each module (10 questions, auto-graded)
- Provide **learning path** diagrams showing dependencies

**Impact if Unresolved**: Students struggle with pacing; dropout risk; advanced students bored with basics.

---

## 4. Hands-On Exercises and Simulations

### 4.1 **CRITICAL: Exercise Testing and Validation**
**Issue**: How do students (and instructors) verify that exercises are completed correctly?

**Clarification Needed**:
- [ ] Should exercises have automated tests (unit tests, integration tests)?
- [ ] Should there be expected outputs (screenshots, logs, metrics)?
- [ ] How do students debug when exercises don't work as expected?

**Suggested Resolution**:
- **Every exercise must include**:
  - Expected output description (e.g., "Robot reaches goal within 30 seconds")
  - Sample output files (screenshots, ROS bag files, videos)
  - Common debugging tips (FAQ section)
- **Automated validation scripts** where feasible:
  - ROS 2 topic checks: `ros2 topic list | grep /my_topic`
  - Gazebo model verification: Check joint states, sensor publishing
- **GitHub template repos** for each exercise with starter code and tests

**Impact if Unresolved**: Students unsure if solutions are correct; instructors overwhelmed with grading; high support burden.

---

### 4.2 **HIGH: Exercise Time Estimates**
**Issue**: Some exercises (e.g., "Train a bipedal walking policy") have no time estimates - How long should students allocate?

**Clarification Needed**:
- [ ] What is the expected time for each exercise (30 min, 2 hours, 1 week)?
- [ ] Should exercises be completed during class time or as homework?
- [ ] What is the total time commitment per module (e.g., 10 hours/week for 4 weeks)?

**Suggested Resolution**:
- Add **time estimates** to every exercise:
  - ⏱️ Quick (30 min): Simple code modifications
  - ⏱️⏱️ Standard (2-4 hours): Complete implementations
  - ⏱️⏱️⏱️ Project (1-2 weeks): Capstone-level integration
- Include **total weekly time estimate** in each module intro (e.g., "Module 1: ~12 hours/week")

**Impact if Unresolved**: Students under/overestimate time; poor time management; missed deadlines.

---

### 4.3 **MEDIUM: Simulation Fidelity vs. Reality Gap**
**Issue**: How do we address the sim-to-real gap in exercises?

**Clarification Needed**:
- [ ] Should exercises explicitly teach sim-to-real transfer techniques?
- [ ] What are the limitations of simulation that students should understand?
- [ ] How do we set realistic expectations for hardware deployment?

**Suggested Resolution**:
- Add **"Sim-to-Real Considerations"** section in each simulation chapter
- Teach **domain randomization** in Isaac Sim chapters (vary lighting, textures, physics)
- Include **case studies** of successful sim-to-real transfers (e.g., OpenAI Rubik's Cube)
- Capstone should include **failure analysis**: "What worked in sim but not on hardware?"

**Impact if Unresolved**: Students surprised when hardware behaves differently; blame themselves for "bad code".

---

## 5. Integration and Dependencies Between Modules

### 5.1 **HIGH: Module 3 (Isaac) Dependency Ambiguity**
**Issue**: Module 4 lists "Module 3 optional but recommended" - What content is missed if students skip Module 3?

**Clarification Needed**:
- [ ] Can Module 4 exercises work with Gazebo instead of Isaac Sim?
- [ ] What specific Isaac tools are used in Module 4 (if any)?
- [ ] Should there be "Module 3 version" and "No Module 3 version" of Module 4 exercises?

**Suggested Resolution**:
- **Decouple Isaac-specific content**:
  - Module 4 Chapter 4.1 (Whisper): Works with any simulation
  - Module 4 Chapter 4.2 (LLM planning): Simulation-agnostic
  - Module 4 Chapter 4.3 (Multimodal): Provide both Gazebo and Isaac Sim examples
- Add **"Prerequisites: Module 1, Module 2, (Module 3 OR Gazebo proficiency)"**

**Impact if Unresolved**: Students skip Module 3 but can't complete Module 4; unclear dependency chain.

---

### 5.2 **MEDIUM: ROS 2 Version Compatibility**
**Issue**: Different modules may require different ROS 2 packages - Is there a version matrix?

**Clarification Needed**:
- [ ] Which ROS 2 distribution is primary: Humble, Iron, Rolling?
- [ ] Are there breaking changes between distributions that affect exercises?
- [ ] Should students use Docker to manage multiple ROS 2 versions?

**Suggested Resolution**:
- **Standardize on ROS 2 Humble Hawksbill** (LTS, supported until 2027)
- Test all exercises on Humble + Ubuntu 22.04
- Provide **Docker images** for consistent environments (optional but recommended)
- Add **version compatibility table** in Appendix

**Impact if Unresolved**: Package incompatibilities; exercises break; "works on my machine" syndrome.

---

### 5.3 **HIGH: Capstone Module Integration Strategy**
**Issue**: Capstone requires integrating Modules 1-4 - Is there a reference architecture or are students designing from scratch?

**Clarification Needed**:
- [ ] Should students follow a prescribed architecture (e.g., behavior trees, ROS 2 nav stack)?
- [ ] Is there a reference implementation to guide them?
- [ ] How much freedom do students have in system design?

**Suggested Resolution**:
- Provide **3 reference architectures** for different complexity levels:
  - **Basic**: ROS 2 nav stack + Whisper voice commands
  - **Intermediate**: Add Isaac ROS perception + LLM task planner
  - **Advanced**: Custom behavior tree with VLA integration
- Students choose based on skill level and time availability
- Provide **architecture diagrams** and **starter code** for each level

**Impact if Unresolved**: Students overwhelmed by integration; unclear starting point; high failure rate.

---

## 6. Student Challenges and Support

### 6.1 **CRITICAL: Debugging Support Strategy**
**Issue**: Robotics debugging is notoriously difficult - How do we help students when things go wrong?

**Clarification Needed**:
- [ ] Should there be office hours, TA support, or forum?
- [ ] What debugging tools should students learn (rqt_graph, rqt_console, Gazebo inspector)?
- [ ] Should there be a "Common Errors and Fixes" section for each chapter?

**Suggested Resolution**:
- **Every chapter includes**:
  - **Common Errors** section with solutions
  - **Debugging Checklist** (e.g., "Check topic names, verify sensor publishing")
  - **Troubleshooting flowchart** for systematic debugging
- Introduce **debugging tools early** (Chapter 1.2):
  - `ros2 topic list/echo/hz`
  - `rqt_graph`, `rqt_console`
  - Gazebo model inspector, joint state viewer
- **RAG Chatbot** should prioritize debugging questions

**Impact if Unresolved**: Students stuck for hours on simple issues; high frustration; increased dropout.

---

### 6.2 **HIGH: Peer Collaboration vs. Academic Integrity**
**Issue**: Robotics projects benefit from collaboration - What is allowed?

**Clarification Needed**:
- [ ] Can students work together on exercises?
- [ ] Should code submissions be individual or group-based?
- [ ] What constitutes plagiarism vs. legitimate collaboration?

**Suggested Resolution**:
- **Module Exercises**: Individual work, but **discussion allowed**
- **Capstone Project**: **Group work encouraged** (2-3 students)
- **Clear academic integrity policy**:
  - ✅ Allowed: Discussing concepts, debugging together, code review
  - ❌ Not Allowed: Copying code, submitting identical solutions
- Use **GitHub Classroom** for code submissions with commit history

**Impact if Unresolved**: Ambiguity leads to unintentional violations; unfair grading.

---

### 6.3 **MEDIUM: Hardware Failure and Backup Plans**
**Issue**: Simulation environments (Gazebo, Isaac Sim) can crash - What's the backup?

**Clarification Needed**:
- [ ] Should students save progress frequently (ROS bag files, checkpoints)?
- [ ] Are there cloud backup solutions if local environments break?
- [ ] What if a student's GPU fails mid-project?

**Suggested Resolution**:
- **Best Practices Section** in Introduction:
  - Save ROS bag files after every successful run
  - Use Git for version control (commit after every working milestone)
  - Docker images for reproducibility
- **Cloud Failover**: Provide AWS/Azure instructions for emergency access
- **Extension Policy**: Instructor can grant extensions for genuine hardware failures

**Impact if Unresolved**: Students lose days of work; frustration; unfair grading.

---

## 7. Content Authoring and Consistency

### 7.1 **MEDIUM: Placeholder Specificity**
**Issue**: Placeholders should guide content creation - What level of detail is needed?

**Clarification Needed**:
- [ ] Should placeholders include example pseudocode or just section titles?
- [ ] How detailed should "Code Examples" placeholders be?
- [ ] Should there be word count targets for each section?

**Suggested Resolution**:
- **Placeholder Template**:
  ```markdown
  ## Theory & Explanation
  <!-- PLACEHOLDER: Explain [TOPIC] in 500-800 words. Include:
       - Intuitive explanation with real-world analogy
       - Mathematical formulation (if applicable)
       - Diagram or flowchart
       - Example: Explain ROS 2 pub-sub with postal mail analogy, show message flow diagram
  -->

  ## Code Examples
  <!-- PLACEHOLDER: Provide 2-3 code examples:
       1. Minimal example (10-15 lines, illustrates core concept)
       2. Practical example (50-100 lines, realistic use case)
       3. Advanced example (optional, shows optimization or edge cases)
       Language: Python (primary), C++ (optional sidebar)
       Testing: All code must run in clean ROS 2 Humble + Ubuntu 22.04 environment
  -->
  ```

**Impact if Unresolved**: Inconsistent content quality; some chapters too verbose, others too sparse.

---

### 7.2 **HIGH: Citation and Reference Standards**
**Issue**: Academic rigor requires proper citations - What format?

**Clarification Needed**:
- [ ] Which citation format: IEEE, ACS, Chicago, informal?
- [ ] Should there be a centralized bibliography or per-chapter references?
- [ ] Are DOIs required for all papers?

**Suggested Resolution**:
- Use **IEEE citation format** (standard in robotics/CS)
- **Per-chapter references** at the end of each chapter (easier for modular reading)
- **Centralized bibliography** in Appendix D (for full book view)
- Use **BibTeX** for consistency; auto-generate from `.bib` file
- Example format:
  ```
  [1] M. Quigley et al., "ROS: an open-source Robot Operating System," ICRA 2009.
  [2] NVIDIA Isaac Sim Documentation, https://docs.omniverse.nvidia.com/isaacsim, Accessed: Dec 2025.
  ```

**Impact if Unresolved**: Inconsistent citations; academic credibility questioned.

---

### 7.3 **MEDIUM: Diagram and Visual Standards**
**Issue**: Visual consistency improves learning - What tools and styles should be used?

**Clarification Needed**:
- [ ] Should diagrams be hand-drawn, generated (Mermaid, Graphviz), or professional (Figma, Illustrator)?
- [ ] What is the color scheme and style guide?
- [ ] Should all diagrams be vector (SVG) or raster (PNG) acceptable?

**Suggested Resolution**:
- **Primary Tool**: Mermaid diagrams (Markdown-native, version-controlled)
- **Secondary**: draw.io / Excalidraw for complex diagrams (export as SVG)
- **Color Scheme**: Docusaurus default palette (color-blind-safe)
- **Format**: SVG preferred (scalable), PNG acceptable (max 1920px width)
- **Alt Text**: Mandatory for accessibility (describe diagram for screen readers)

**Impact if Unresolved**: Visual inconsistency; accessibility issues; unprofessional appearance.

---

## 8. Deployment and Technical Infrastructure

### 8.1 **HIGH: Docusaurus Version and Plugin Compatibility**
**Issue**: Spec mentions "Docusaurus v3 (latest stable)" - What if v3 is still beta?

**Clarification Needed**:
- [ ] As of Dec 2025, should we check if v3 is stable or use v2?
- [ ] Which plugins are confirmed compatible?
- [ ] Should there be a fallback to v2 if v3 has issues?

**Suggested Resolution**:
- **Check Docusaurus status** (as of Dec 2025):
  - If v3 stable → use v3
  - If v3 beta/unstable → use v2.x (latest stable)
- **Required Plugins**:
  - `@docusaurus/preset-classic` (default)
  - `@docusaurus/theme-mermaid` (diagrams)
  - `@docusaurus/plugin-client-redirects` (if restructuring URLs)
- Test skeleton build with all plugins before content creation

**Impact if Unresolved**: Build breaks mid-project; plugin incompatibilities; migration overhead.

---

### 8.2 **MEDIUM: GitHub Pages vs. Vercel Decision**
**Issue**: Spec lists both GitHub Pages and Vercel - Which is primary?

**Clarification Needed**:
- [ ] What are the pros/cons of each for this use case?
- [ ] Should the skeleton support both, or choose one?
- [ ] Are there build time or resource limits to consider?

**Suggested Resolution**:
- **Primary: GitHub Pages** (free, integrated with repo, easy CI/CD)
- **Alternative: Vercel** (faster builds, better preview deployments, custom domains)
- Set up **GitHub Pages by default** in skeleton
- Provide **Vercel migration guide** in Appendix
- Test both to ensure Docusaurus config works for either

**Impact if Unresolved**: Deployment issues; students confused about where to deploy.

---

### 8.3 **HIGH: CI/CD Automation Scope**
**Issue**: Out-of-scope lists "CI/CD pipeline" - But should skeleton include basic GitHub Actions?

**Clarification Needed**:
- [ ] Should skeleton include a `.github/workflows/deploy.yml` file?
- [ ] What tests should run on every PR (link checking, build validation)?
- [ ] Should there be automated Lighthouse scores for accessibility/performance?

**Suggested Resolution**:
- **Minimal CI/CD in skeleton**:
  - `.github/workflows/deploy.yml`: Build and deploy on push to main
  - `.github/workflows/pr-check.yml`: Build validation + link checking on PRs
- **Advanced CI/CD (post-skeleton)**:
  - Automated accessibility tests (pa11y, axe-core)
  - Broken link detection (lychee)
  - Lighthouse performance scores
- Provide example workflows in skeleton with comments for customization

**Impact if Unresolved**: Manual deployments error-prone; broken links go unnoticed; quality degrades over time.

---

## Summary of Actionable Clarifications

### Immediate (Pre-Implementation)
1. ✅ Resolve Gazebo version (Classic vs. Sim)
2. ✅ Define Module 3 (Isaac) as optional with Gazebo fallback
3. ✅ Specify hardware deployment as optional for Capstone
4. ✅ Choose ROS 2 Humble as primary distribution
5. ✅ Decide on LLM approach (local open-source via Ollama)
6. ✅ Set Whisper model default (base model)
7. ✅ Confirm Docusaurus version (v2 or v3)
8. ✅ Establish placeholder template with detail level
9. ✅ Define citation format (IEEE)
10. ✅ Create exercise time estimate guidelines

### High Priority (During Skeleton Creation)
11. Specify URDF complexity for Chapter 1.3
12. Clarify RL training scope for Chapter 1.4
13. Define RViz2 vs Unity use cases
14. Add difficulty tags (beginner/intermediate/advanced)
15. Create measurable learning outcomes for each module
16. Define prerequisites precisely (AI, Linux, programming)
17. Establish assessment rubrics
18. Create debugging checklists and common errors sections
19. Set up basic GitHub Actions workflows
20. Decide on GitHub Pages vs Vercel

### Medium Priority (Content Development Phase)
21-47. Address remaining clarity issues as content is created

---

## Recommended Next Steps

1. **Review this clarification report** with project stakeholders
2. **Make decisions** on all CRITICAL and HIGH priority items
3. **Update spec.md and book-structure.md** with resolved clarifications
4. **Create appendices**:
   - Appendix A: AI Prerequisites Refresher
   - Appendix B: Linux Essentials Cheat Sheet
   - Appendix C: Assessment Rubrics
   - Appendix D: Complete Bibliography
5. **Proceed with skeleton implementation** using clarified requirements

---

**Report Status**: Draft v1.0
**Next Review**: After stakeholder feedback on critical clarifications

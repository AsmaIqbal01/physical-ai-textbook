# Critical Design Decisions - Physical AI Textbook

**Date**: 2025-12-06
**Status**: Approved
**Version**: 1.0

---

## Decision Summary

This document records all critical design decisions made to resolve ambiguities in the book skeleton specification. These decisions affect the entire book structure, content, and student experience.

---

## 1. Simulation Platform

**Decision**: **Gazebo Sim (Ignition Fortress/Garden)**

**Rationale**:
- ROS 2 native integration (no bridge required)
- Modern architecture with better performance
- Active development and long-term support
- Compatible with ROS 2 Humble (our target distribution)

**Implications**:
- All simulation exercises use Gazebo Sim
- Chapter 2.1 introduces Gazebo Sim (brief mention of Classic for historical context)
- Content authors must use Ignition Fortress or Garden
- Docker images will include Gazebo Sim pre-installed

**Alternatives Considered**:
- ❌ Gazebo Classic: Deprecated, requires ros_gz_bridge
- ❌ Both equally: Doubles content scope unnecessarily

**References**:
- Gazebo Sim docs: https://gazebosim.org
- ROS 2 + Gazebo integration: https://github.com/ros-simulation/gz_ros2_control

---

## 2. Module 3 (NVIDIA Isaac) Structure

**Decision**: **Tiered Learning Paths - Basic (Gazebo) + Advanced (Isaac)**

**Rationale**:
- Inclusive: Students without RTX GPUs can complete the course
- Aspirational: Advanced students can learn cutting-edge Isaac tools
- Flexible: Instructors can choose which path to emphasize
- Reduces financial barrier ($0 for Basic path vs. $1500+ for Isaac hardware)

**Implementation**:

### Basic Path (Gazebo-based)
- **Module 3 Basic**: Advanced Gazebo techniques (sensor fusion, navigation, motion planning)
- **Exercises**: Gazebo-native implementations
- **Capstone**: Fully functional in Gazebo Sim
- **Hardware**: CPU sufficient, GPU optional for faster rendering

### Advanced Path (Isaac-based)
- **Module 3 Advanced**: Isaac Sim + Isaac ROS + Isaac Gym
- **Exercises**: GPU-accelerated perception, photorealistic simulation, RL training
- **Capstone**: Enhanced with Isaac features (synthetic data, advanced perception)
- **Hardware**: RTX 3060+ or cloud (AWS g4dn.xlarge ~$0.50/hr)

### Course Completion Requirements
- **To pass**: Complete Basic path OR Advanced path (student's choice)
- **Prerequisites**:
  - Module 4: Requires Modules 1, 2, and (Module 3 Basic OR Module 3 Advanced)
  - Capstone: Requires all modules (either path)

**Implications**:
- Content authors create parallel exercises for each path
- Sidebar indicators: 🟢 Basic Path | 🔵 Advanced Path
- Estimated content increase: +30% (but avoids excluding students)
- Assessment rubrics for both paths (equivalent difficulty)

**Alternatives Considered**:
- ❌ Mandatory Isaac: Excludes students without GPUs
- ❌ Optional Isaac only: Devalues Isaac, unclear requirements

---

## 3. LLM Integration Approach

**Decision**: **Both Local (Ollama) and API (OpenAI) - Student's Choice**

**Rationale**:
- **Local (Ollama + LLaMA/Mistral)**:
  - Pros: Free, private, no API limits, works offline
  - Cons: Requires 8-16GB RAM, slower on CPU, larger downloads (~4-7GB models)
- **Cloud APIs (OpenAI/Claude)**:
  - Pros: Easy setup, best quality, faster responses
  - Cons: Costs $10-30 per student, requires credit card, internet dependency

**Implementation**:

### Module 4 Structure (Dual Path)
Each chapter provides **two implementation guides**:

#### Option A: Local LLM (Ollama)
- Installation: `curl -fsSL https://ollama.com/install.sh | sh`
- Models: LLaMA 2 7B, Mistral 7B Instruct
- Integration: Python client (`ollama.Client()`)
- Pros: Free, unlimited usage
- Cons: 4-7GB download, slower inference

#### Option B: Cloud API (OpenAI)
- Setup: API key from OpenAI (requires account + payment)
- Models: gpt-3.5-turbo (cheaper), gpt-4o-mini (better)
- Integration: `openai` Python library
- Pros: Fast, high quality
- Cons: ~$0.50-2.00 per exercise (est. $10-30 total)

### Default Recommendation
- **Primary**: Ollama (accessible to all students, no cost barrier)
- **Alternative**: OpenAI API (for students preferring speed/quality)
- **Code Examples**: All use abstraction layer (works with either backend)

**Budget Estimate**:
- Local path: $0 (free, only disk space + compute)
- API path: ~$10-30 per student for all Module 4 exercises

**Implications**:
- Tutorial sections duplicated for both approaches (~20% content increase)
- Code examples use abstraction: `LLMClient` class with `OllamaBackend` or `OpenAIBackend`
- Students declare choice in Chapter 4.1 setup

**Alternatives Considered**:
- ❌ Local only: Slow on older laptops, worse quality than GPT-4
- ❌ API only: Creates financial barrier, excludes students without credit cards

---

## 4. Physical Hardware Deployment

**Decision**: **Simulation Sufficient - Hardware Deployment Optional Bonus**

**Rationale**:
- **Accessibility**: Not all students can afford $2000+ hardware kits
- **Educational Goals**: Core learning outcomes achievable in simulation
- **Equity**: Course should not have financial barriers beyond standard laptop
- **Practicality**: Sim-to-real transfer is complex; simulation validates understanding

**Implementation**:

### Core Capstone (Simulation-based)
- **Requirement**: Deploy autonomous humanoid in Gazebo Sim or Isaac Sim
- **Deliverables**:
  - Fully functional simulation demo (video recording)
  - Code repository (GitHub)
  - Technical report (architecture, results, lessons learned)
- **Grading**: 100% achievable with simulation only

### Optional Hardware Bonus (+10-20% extra credit)
- **Requirement**: Deploy Capstone project on physical robot
- **Minimum Hardware**:
  - Compute: Jetson Orin Nano ($499) or Raspberry Pi 4 (cheaper, slower)
  - Sensors: Webcam (~$30) or RealSense D435i ($329)
  - Robot: Any wheeled platform (RC car, TurtleBot3, Unitree Go2)
- **Deliverables**:
  - Real-world demo video
  - Sim-to-real transfer report (what changed, why)
  - Failure analysis (what worked in sim but not hardware)
- **Grading**: Extra credit for attempting; full credit even if deployment partially succeeds

### Institutional Support (Optional)
- Universities can provide loaner kits (Jetson + RealSense + robot)
- Group projects: 3-4 students share one kit
- Lab hours: Scheduled hardware access for testing

**Implications**:
- Capstone chapter includes **two completion paths**:
  - Chapter 5.3A: Simulation Deployment (mandatory)
  - Chapter 5.3B: Hardware Deployment (optional bonus)
- Sim-to-real techniques taught in bonus section (domain randomization, calibration)
- Success criteria focused on simulation fidelity, not hardware

**Alternatives Considered**:
- ❌ Hardware required: Excludes students, expensive ($2000+ per student)
- ❌ Group kits mandatory: Coordination overhead, unequal contribution

---

## 5. Reinforcement Learning Training Scope

**Decision**: **Pre-trained Model Provided - Focus on ROS 2 Bridge Only**

**Rationale**:
- **Module 1 Focus**: ROS 2 fundamentals, not deep RL
- **Time Constraint**: Training RL from scratch adds 5-10 hours to one chapter
- **Prerequisites**: RL theory not assumed; students come from diverse backgrounds
- **Learning Outcome**: Objective is "bridge AI agents to ROS 2", not "train RL policies"

**Implementation**:

### Chapter 1.4: Python Agents to ROS Bridge

#### Core Content (All Students)
1. **Introduction**: What are AI agents? (ML models, RL policies, planners)
2. **Pre-trained Model**: Download a Stable Baselines3 PPO agent (robotic arm reaching task)
3. **ROS 2 Bridge**:
   - Load model in Python script
   - Subscribe to sensor topics (joint states, camera feed)
   - Publish actions to control topics (joint commands)
4. **Exercise**: Run pre-trained agent in Gazebo, observe behavior, modify reward shaping

#### Advanced Optional Section (RL Enthusiasts)
- **Appendix 1.4A: Training Your Own RL Agent** (5-10 pages)
  - Gym environment basics
  - Stable Baselines3 PPO training loop
  - Hyperparameter tuning tips
  - Expected time: 3-8 hours (training + tuning)

**Pre-trained Model Details**:
- **Task**: Robotic arm reaching target position
- **Algorithm**: PPO (Proximal Policy Optimization)
- **Framework**: Stable Baselines3
- **Environment**: Custom Gym environment (included in course repo)
- **Training Time**: Pre-trained for 1M steps (~4 hours on RTX 3060)
- **Performance**: ~90% success rate in simulation

**Implications**:
- Students focus on **integration** (ROS 2 bridge) rather than **training** (RL theory)
- Keeps Chapter 1.4 within 2-3 hour time estimate
- Advanced students can explore RL in appendix (optional deep dive)
- Prepares students for Module 3 Advanced (Isaac Gym RL training)

**Alternatives Considered**:
- ❌ Train from scratch: Too time-consuming for Module 1
- ❌ Skip RL entirely: Misses important AI-robotics integration concept

---

## 6. Exercise Validation Method

**Decision**: **Expected Output Samples (Screenshots, Logs, Videos)**

**Rationale**:
- **Simplicity**: Easy for students to understand ("does my output match this?")
- **Visual Learning**: Screenshots/videos are intuitive validation
- **Debugging Aid**: Students can compare their outputs to expected results
- **No Infrastructure**: No need for autograder servers or complex test frameworks

**Implementation**:

### Exercise Template Structure
Each exercise includes:

#### 1. Objective (What to build)
Example: "Create a ROS 2 publisher that sends velocity commands to a simulated robot."

#### 2. Instructions (Step-by-step)
- Step 1: Create package `my_robot_control`
- Step 2: Write publisher node `velocity_publisher.py`
- Step 3: Launch Gazebo simulation
- Step 4: Run your node and observe robot movement

#### 3. Expected Outputs (Validation)
- **Terminal Output**:
  ```
  [INFO] [velocity_publisher]: Publishing to /cmd_vel at 10Hz
  [INFO] [velocity_publisher]: Linear: 0.5 m/s, Angular: 0.2 rad/s
  ```
- **Screenshot**: Gazebo window showing robot moving forward and turning
- **ROS 2 Topic Check**:
  ```bash
  ros2 topic list
  # Expected: /cmd_vel appears in the list

  ros2 topic echo /cmd_vel
  # Expected: Twist messages with linear.x=0.5, angular.z=0.2
  ```
- **Video** (for complex exercises): 10-30 second clip of correct behavior

#### 4. Common Errors and Fixes
- **Error**: "No such package 'my_robot_control'"
  - **Fix**: Run `colcon build` in workspace root
- **Error**: "Robot not moving in Gazebo"
  - **Fix**: Check topic name (should be `/cmd_vel`, not `/velocity`)

#### 5. Debugging Checklist
- [ ] Package built successfully (`colcon build`)
- [ ] Node running without errors (`ros2 run my_robot_control velocity_publisher`)
- [ ] Topic exists (`ros2 topic list | grep /cmd_vel`)
- [ ] Messages publishing (`ros2 topic echo /cmd_vel`)
- [ ] Gazebo simulation running

### Validation File Structure (in course repo)
```
exercises/
├── module-01/
│   ├── chapter-1.2-exercise/
│   │   ├── README.md (instructions)
│   │   ├── expected-outputs/
│   │   │   ├── terminal-output.txt
│   │   │   ├── gazebo-screenshot.png
│   │   │   └── demo-video.mp4
│   │   ├── starter-code/ (optional templates)
│   │   └── solution/ (reference implementation, released after deadline)
```

**Implications**:
- Content authors must create expected outputs for every exercise
- Screenshots use consistent Gazebo/RViz2 themes for clarity
- Videos hosted on GitHub repo (< 10MB) or YouTube (linked in README)
- Students self-validate before submitting to instructors

**Alternatives Considered**:
- ❌ Automated tests: Complex to maintain, requires CI/CD infrastructure
- ❌ Rubrics only: Subjective, slower feedback, instructor bottleneck
- ✅ **Future Enhancement**: Add optional automated tests for motivated students

---

## 7. Additional Resolved Decisions

### 7.1 ROS 2 Distribution
- **Decision**: ROS 2 Humble Hawksbill (LTS)
- **Rationale**: Long-term support until 2027, stable, Ubuntu 22.04 compatible
- **Alternatives**: Iron (shorter support), Rolling (unstable)

### 7.2 Citation Format
- **Decision**: IEEE citation style
- **Rationale**: Standard in CS/robotics, concise, widely recognized
- **Implementation**: BibTeX files in `references/`, auto-generate per chapter

### 7.3 Diagram Tool
- **Decision**: Mermaid (primary), draw.io/Excalidraw (secondary)
- **Rationale**: Mermaid is Markdown-native, version-controlled, Docusaurus-integrated
- **Format**: SVG preferred (scalable), PNG acceptable (max 1920px width)

### 7.4 Docusaurus Version
- **Decision**: Check in Dec 2025; use latest stable (likely v3.x)
- **Fallback**: If v3 unstable, use v2.4+ (latest v2 stable)
- **Plugins**: `@docusaurus/preset-classic`, `@docusaurus/theme-mermaid`

---

## 8. Impact Summary

### Accessibility Improvements
- ✅ No mandatory GPU requirement (Basic path uses Gazebo)
- ✅ No mandatory hardware purchase (simulation sufficient)
- ✅ No mandatory API costs (local LLMs available)
- ✅ Tiered learning (accommodates different skill/resource levels)

### Content Scope Increase
- +30%: Dual path for Module 3 (Gazebo Basic + Isaac Advanced)
- +20%: Dual LLM tutorials (Ollama + OpenAI API)
- +10%: Optional hardware deployment guide
- **Total**: ~60% more content than single-path approach
- **Justification**: Inclusivity and flexibility outweigh authoring cost

### Student Outcomes
- **Minimum Viable Path**: Modules 1, 2, 3 Basic, 4 (local LLM), Capstone (sim only)
  - Cost: $0 (beyond laptop)
  - Hardware: Mid-range laptop (16GB RAM, no GPU required)
  - Time: ~14 weeks (4+3+4+3 weeks modules)
- **Advanced Path**: Modules 1, 2, 3 Advanced, 4 (API), Capstone (hardware bonus)
  - Cost: ~$30 (API) + $2000 (hardware, optional)
  - Hardware: RTX GPU or cloud access
  - Time: +20% (more complex exercises)

---

## 9. Next Steps

1. ✅ **Update `spec.md`** with decisions (FR-001 to FR-022 amendments)
2. ✅ **Update `book-structure.md`** with tiered paths (Module 3 split into Basic/Advanced)
3. ✅ **Create `appendices.md`** outline:
   - Appendix A: AI Prerequisites Refresher
   - Appendix B: Linux Essentials Cheat Sheet
   - Appendix C: Assessment Rubrics (Basic vs Advanced paths)
   - Appendix D: Complete Bibliography
   - Appendix 1.4A: Training Your Own RL Agent
4. ⏳ **Proceed to Planning Phase** (`/sp.plan book-skeleton`)

---

**Approval**: User-approved on 2025-12-06
**Implementation**: Ready for skeleton creation
**Review Date**: Before content authoring begins (verify all decisions still valid)

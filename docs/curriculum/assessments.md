# Assessments & Grading

This course uses a project-based assessment model where hands-on work is prioritized over traditional exams.

## Grading Breakdown

| Component | Weight | Description |
|-----------|--------|-------------|
| Lab Assignments | 20% | Weekly hands-on exercises (Weeks 1-12) |
| Module 1 Project | 15% | ROS 2 autonomous navigation system |
| Module 2 Project | 15% | Gazebo simulation with sim-to-real validation |
| Module 3 Project | 20% | AI-powered perception-action pipeline |
| Final Capstone | 30% | End-to-end Physical AI system |

**Total: 100%**

---

## Lab Assignments (20%)

### Format
- **Quantity**: 12 labs (1 per week, Weeks 1-12)
- **Submission**: Code + README + demo video
- **Duration**: 3-5 hours per lab
- **Grading**: Pass/Fail (must complete 10/12 to earn full credit)

### Requirements
Each lab must include:
1. **Functional Code**: Runs without errors on specified hardware
2. **Documentation**: README with setup instructions and usage
3. **Demo Video**: 2-3 minute recording showing working system
4. **Reflection**: 1-paragraph summary of challenges and learnings

### Example Labs
- Lab 1.2: Publisher/Subscriber Nodes
- Lab 4.2: Gazebo Sensor Integration
- Lab 7.1: YOLOv8 Object Detection
- Lab 11.1: GPT-4 Robot Command Interface

---

## Module 1 Project: ROS 2 Navigation System (15%)

**Due**: End of Week 3

### Objective
Build an autonomous navigation system where a robot navigates a Gazebo warehouse while avoiding obstacles and reaching target waypoints.

### Requirements
- **Navigation**: Robot reaches 5 waypoints in under 5 minutes
- **Obstacle Avoidance**: Uses laser scan data to avoid collisions
- **URDF Model**: Custom robot with differential drive and sensors
- **Launch Files**: Single command to start entire system
- **Documentation**: System architecture diagram + README

### Grading Rubric
| Criterion | Points | Description |
|-----------|--------|-------------|
| Navigation Accuracy | 40 | All waypoints reached successfully |
| Code Quality | 25 | Clean, modular, well-documented code |
| System Integration | 20 | Seamless ROS 2 component integration |
| Documentation | 15 | Clear architecture and usage docs |

---

## Module 2 Project: Digital Twin Validation (15%)

**Due**: End of Week 6

### Objective
Create a Gazebo digital twin of a real environment, validate sim-to-real transfer by testing the same code in both simulation and reality.

### Requirements
- **Custom World**: Gazebo environment modeled from real space
- **Domain Randomization**: Randomized lighting/textures/physics
- **Sim-to-Real Test**: Same navigation code runs in sim and reality
- **Performance Report**: Compare success rates, timing, and failure modes
- **Multi-Robot**: Coordinate 2+ robots in shared workspace

### Grading Rubric
| Criterion | Points | Description |
|-----------|--------|-------------|
| Simulation Fidelity | 30 | Realistic physics and sensor models |
| Sim-to-Real Transfer | 30 | <20% performance drop on real hardware |
| Domain Randomization | 20 | Effective randomization strategy |
| Analysis Report | 20 | Thorough comparison and insights |

---

## Module 3 Project: AI Perception-Action Pipeline (20%)

**Due**: End of Week 10

### Objective
Deploy an end-to-end AI pipeline where a robot detects objects, plans grasps, and executes manipulation using vision models and learned policies.

### Requirements
- **Vision Model**: Real-time object detection (≥10 FPS on Jetson)
- **Manipulation**: Pick-and-place 3+ object types
- **RL Training**: Policy trained in Isaac Gym with ≥80% success rate
- **Edge Deployment**: Entire pipeline runs on Jetson Orin
- **Failure Recovery**: Handles at least 2 failure modes gracefully

### Grading Rubric
| Criterion | Points | Description |
|-----------|--------|-------------|
| Perception Accuracy | 25 | Detection mAP ≥0.75 on test set |
| Manipulation Success | 30 | ≥80% pick-place success on 20 trials |
| Real-Time Performance | 20 | End-to-end latency <500ms |
| Edge Optimization | 15 | Efficient use of Jetson resources |
| Robustness | 10 | Handles occlusion, lighting changes |

---

## Final Capstone Project (30%)

**Due**: End of Week 13

### Objective
Design and build a complete Physical AI system that solves a real-world problem, integrating all course concepts: ROS 2, simulation, AI, and hardware.

### Project Scope
Students choose one of the following tracks:

#### Track A: Assistive Robotics
Build a home assistant robot that:
- Responds to voice commands (LLM integration)
- Navigates multi-room environments
- Performs object retrieval and delivery
- Adapts to user preferences over time

#### Track B: Warehouse Automation
Deploy a fleet of robots that:
- Coordinate to move inventory
- Optimize traffic flow to minimize collisions
- Use VLA models for flexible object manipulation
- Report progress via cloud dashboard

#### Track C: Custom Proposal
Propose and implement a novel Physical AI application (requires instructor approval by Week 11)

### Deliverables
1. **Working System**: Live demo during final presentation
2. **Code Repository**: Complete source code with CI/CD
3. **Technical Report**: 10-page document covering:
   - Problem statement and motivation
   - System architecture
   - Implementation details
   - Experimental results
   - Lessons learned and future work
4. **15-Minute Presentation**: Technical talk with Q&A

### Grading Rubric
| Criterion | Points | Description |
|-----------|--------|-------------|
| Functionality | 40 | System works as specified |
| Technical Depth | 25 | Advanced integration of course concepts |
| Innovation | 15 | Novel approach or application |
| Documentation | 10 | Clear, comprehensive technical report |
| Presentation | 10 | Effective communication of results |

---

## Late Policy

- **Labs**: No credit for submissions >7 days late
- **Module Projects**: -10% per day late (max -50%)
- **Final Capstone**: No late submissions accepted

## Academic Integrity

All code must be your own work. You may:
- Use official documentation and tutorials
- Discuss concepts with classmates (but not share code)
- Use ChatGPT/Claude for debugging (must cite usage)

You may **not**:
- Copy code from classmates or online sources without attribution
- Submit work you did not personally write
- Use AI to generate entire solutions without understanding

Violations result in zero credit and referral to academic misconduct office.

---

## Grading Scale

| Letter Grade | Percentage | Description |
|-------------|-----------|-------------|
| A | 93-100% | Exceptional work, exceeds expectations |
| A- | 90-92% | Excellent work, meets all criteria |
| B+ | 87-89% | Very good work, minor issues |
| B | 83-86% | Good work, some missing elements |
| B- | 80-82% | Satisfactory work, needs improvement |
| C+ | 77-79% | Adequate work, significant gaps |
| C | 73-76% | Minimal acceptable work |
| C- | 70-72% | Below expectations |
| D | 60-69% | Failing work, major deficiencies |
| F | <60% | Unacceptable work |

---

**Next**: Explore the course modules to begin learning!

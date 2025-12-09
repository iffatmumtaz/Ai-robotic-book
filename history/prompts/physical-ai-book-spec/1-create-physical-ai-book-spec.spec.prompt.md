---
id: 1
title: Create Physical AI Book Spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-book-spec
branch: 001-physical-ai-book-spec
user: unknown
command: /sp.specify
labels: ["specification", "book-development"]
links:
  spec: specs/001-physical-ai-book-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book-spec/spec.md
  - specs/001-physical-ai-book-spec/checklists/requirements.md
tests:
  - none
---

## Prompt

/sp.specify Create a full technical book specification for the capstone course
“Physical AI & Humanoid Robotics.”

Target Audience:
• Students, educators, and robotics developers learning embodied AI systems.
• Readers using ROS 2, Gazebo, Unity, and NVIDIA Isaac for humanoid robotics.

Project Goal:
Write a complete textbook for the Physical AI course using Docusaurus and Spec-Kit Plus.
The book must teach how AI systems interact with the physical world and how humanoid robots
use sensing, perception, planning, and action to operate in real environments.

Scope Requirements:
• Cover all 4 modules:
  1. ROS 2 – The Robotic Nervous System
  2. Gazebo + Unity – The Digital Twin
  3. NVIDIA Isaac – The AI-Robot Brain
  4. Vision-Language-Action (VLA)

• Include weekly learning topics, assessments, labs, and hardware requirements.
• Include simulation workflows (Gazebo, Isaac Sim, Unity) and deployment workflows
  for Jetson Orin + RealSense.
• Include Capstone Project: Autonomous Humanoid with voice command → planning → navigation → object detection → manipulation.

Book Structure Requirements:
• 1 chapter per module.
• Each chapter must contain 3 lessons.
• Each lesson must include:
  – Title
  – Description
  – Learning goals
  – Hands-on lab or coding activity
  – Required tools/software
  – Expected outcome for learners

Content Guidelines:
• Teach Physical AI foundations and embodied intelligence clearly.
• Explain interfaces between LLMs, ROS 2, and robot actions.
• Provide practical examples: robot control, SLAM, digital twin, path planning, RL, VLA.
• Include diagrams/tables where relevant (described in text).

Technical Requirements:
• Docusaurus-ready structure with:
  – Sidebar hierarchy
  – Clean folder naming
  – Lesson markdown layout
• Must be compatible with GitHub Pages publishing.
• Must reference required hardware + cloud options:
  – RTX workstation requirements
  – Jetson Orin Kit
  – RealSense cameras
  – Unitree Go2 / Unitree G1 / proxy robots

Success Criteria:
• 12 total lessons (4 chapters × 3 lessons each).
• Each module contains at least:
  – 2 practical labs
  – 1 simulation workflow
  – 1 evaluation activity
• Capstone project fully defined with architecture, input/output flow, and testing stages.
• A reader should be able to:
  – Understand Physical AI end-to-end
  – Build, simulate, and deploy a humanoid robot pipeline
  – Integrate LLMs with ROS 2
  – Operate a Jetson-based physical AI stack
• All explanations must be technically accurate and actionable.

## Response snapshot

Feature branch: `001-physical-ai-book-spec`
Specification file: `specs/001-physical-ai-book-spec/spec.md`
Checklist file: `specs/001-physical-ai-book-spec/checklists/requirements.md`

All specification quality checklist items passed.

## Outcome

- ✅ Impact: A comprehensive technical book specification for "Physical AI & Humanoid Robotics" was successfully created, outlining target audience, project goal, scope, structure, content guidelines, technical requirements, and success criteria.
- 🧪 Tests: No automated tests were executed as part of spec creation. The spec was validated against a quality checklist.
- 📁 Files: The `spec.md` and `requirements.md` checklist files were created in `specs/001-physical-ai-book-spec/`.
- 🔁 Next prompts: The specification is ready for the next phase, `/sp.clarify` or `/sp.plan`.
- 🧠 Reflection: The spec was generated accurately from the user's detailed description, and all checklist items were addressed.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
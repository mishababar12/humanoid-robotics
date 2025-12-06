<!-- Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles: All principles updated or new.
Added sections: Technical Requirements & Infrastructure, Course Structure & Assessment
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .claude/commands/sp.adr.md ✅ updated (no specific changes needed, general guidance)
- .claude/commands/sp.analyze.md ✅ updated
- .claude/commands/sp.checklist.md ✅ updated
- .claude/commands/sp.clarify.md ✅ updated
- .claude/commands/sp.constitution.md ✅ updated
- .claude/commands/sp.git.commit_pr.md ✅ updated
- .claude/commands/sp.implement.md ✅ updated
- .claude/commands/sp.phr.md ✅ updated
- .claude/commands/sp.plan.md ✅ updated
- .claude/commands/sp.specify.md ✅ updated
- .claude/commands/sp.tasks.md ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics — Technical Textbook Constitution

## Core Principles

### I. AI/Spec-Driven Book Creation & Deployment
Every feature, including the book's content, must be developed following Spec-Driven Development (SDD) principles. The textbook itself will be created using Docusaurus, deployed to GitHub Pages, and leverage Claude Code and Spec-Kit Plus for its development lifecycle. This ensures a structured, verifiable, and automated approach to content generation and deployment.

### II. Integrated RAG Chatbot Development
The published textbook must incorporate an embedded Retrieval-Augmented Generation (RAG) chatbot. This chatbot will be built using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier. Its core functionality must include answering user questions about the book's content and providing context-aware responses based on user-selected text within the book.

### III. Reusable Intelligence with Subagents & Skills
The project must prioritize the creation and utilization of reusable intelligence components. This involves developing Claude Code Subagents and Agent Skills to automate complex tasks, enhance development efficiency, and demonstrate advanced AI integration within the book project. This approach promotes modularity and extensibility.

### IV. Secure & Personalized User Experience
User authentication and authorization must be implemented using Better-Auth. Upon signup, the system will collect information about the user's software and hardware background. This data will be used to personalize the content of the textbook, ensuring that learning materials are tailored to individual user needs and technical capabilities.

### V. Multi-lingual Content Support
The textbook must include functionality that allows logged-in users to translate chapter content into Urdu. This feature enhances accessibility and caters to a broader audience, demonstrating a commitment to inclusive learning experiences.

### VI. Test-First Development (NON-NEGOTIABLE)
Test-Driven Development (TDD) is a mandatory practice for all code development. Tests must be written and approved by stakeholders before implementation begins. The Red-Green-Refactor cycle will be strictly enforced to ensure code quality, reliability, and maintainability throughout the project lifecycle.

## Technical Requirements & Infrastructure

The "Physical AI & Humanoid Robotics" course is computationally demanding, sitting at the intersection of Physics Simulation (Isaac Sim/Gazebo), Visual Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA).

### Digital Twin Workstation (Required per Student)
-   **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher (Ideal: RTX 3090/4090 with 24GB VRAM)
-   **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9
-   **RAM**: 64 GB DDR5 (32 GB absolute minimum)
-   **OS**: Ubuntu 22.04 LTS (native Linux for ROS 2)

### Physical AI Edge Kit
-   **Brain**: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
-   **Eyes**: Intel RealSense D435i or D455 (RGB and Depth data, includes IMU)
-   **Inner Ear**: Generic USB IMU (BNO055)
-   **Voice Interface**: Simple USB Microphone/Speaker array (e.g., ReSpeaker)

### Robot Lab Options
-   **Option A (Proxy)**: Unitree Go2 Edu (quadruped)
-   **Option B (Miniature Humanoid)**: Unitree G1 or Robotis OP3 (or Hiwonder TonyPi Pro for kinematics only)
-   **Option C (Premium)**: Unitree G1 Humanoid

### Cloud-Native Lab (High OpEx Alternative)
-   **Cloud Workstations**: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge with NVIDIA Isaac Sim on Omniverse Cloud.
-   **Local "Bridge" Hardware**: Jetson Kit and one physical robot still required for physical deployment phase.

## Course Structure & Assessment

The course is structured into modules covering essential aspects of Physical AI and Humanoid Robotics, with practical assessments.

### Modules
-   **Module 1**: The Robotic Nervous System (ROS 2)
-   **Module 2**: The Digital Twin (Gazebo & Unity)
-   **Module 3**: The AI-Robot Brain (NVIDIA Isaac™)
-   **Module 4**: Vision-Language-Action (VLA)

### Learning Outcomes
-   Understand Physical AI principles and embodied intelligence
-   Master ROS 2 for robotic control
-   Simulate robots with Gazebo and Unity
-   Develop with NVIDIA Isaac AI robot platform
-   Design humanoid robots for natural interactions
-   Integrate GPT models for conversational robotics

### Assessments
-   ROS 2 package development project
-   Gazebo simulation implementation
-   Isaac-based perception pipeline
-   Capstone Project: Simulated humanoid robot with conversational AI

## Governance

This constitution serves as the foundational document for the "Physical AI & Humanoid Robotics — Technical Textbook" project. It supersedes all other project practices, guidelines, and agreements. Any amendments to this constitution require a formal proposal, thorough documentation of rationale and potential impacts, and explicit approval from the project leads. A detailed migration plan must accompany any backward-incompatible changes. All Pull Requests and code reviews must rigorously verify compliance with the principles and requirements outlined herein. Justification is required for any introduced complexity.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04

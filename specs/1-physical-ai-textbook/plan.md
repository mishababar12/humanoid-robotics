# Implementation Plan: Physical AI & Humanoid Robotics — Technical Textbook

**Branch**: `1-physical-ai-textbook` | **Date**: 2025-12-04 | **Spec**: [specs/1-physical-ai-textbook/spec.md](specs/1-physical-ai-textbook/spec.md)
**Input**: Feature specification from `/specs/1-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the development of a technical textbook for "Physical AI & Humanoid Robotics," leveraging Spec-Driven Development (SDD) for content creation and deployment via Docusaurus and GitHub Pages. It integrates a RAG chatbot using OpenAI Agents, FastAPI, Neon, and Qdrant, along with user personalization and Urdu translation features. The core technical approach involves Python-based robotics middleware (ROS 2), physics simulation (Gazebo, Unity), advanced AI perception (NVIDIA Isaac), and large language model integration (VLA systems).

## Technical Context

**Language/Version**: Python 3.x (for ROS 2 rclpy, FastAPI, OpenAI Agents/ChatKit SDKs)
**Primary Dependencies**: ROS 2 (Humble/Iron), Gazebo, Unity, NVIDIA Isaac SDK (Isaac Sim, Isaac ROS), OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier, Docusaurus, Better-Auth.
**Storage**: Neon Serverless Postgres (for RAG chatbot knowledge base, user profiles); Docusaurus content stored as Markdown files.
**Testing**: Pytest (for Python code), Docusaurus built-in testing (for documentation rendering), unit/integration tests for RAG chatbot API, end-to-end tests for personalization/translation features, reproducibility checks for all code examples/simulations.
**Target Platform**: Ubuntu 22.04 LTS (for local high-performance workstations and NVIDIA Jetson Edge AI kits); potential cloud instances (AWS g5.2xlarge / g6e.xlarge with NVIDIA Omniverse Cloud for Isaac Sim).
**Project Type**: Technical textbook (Docusaurus) with integrated web services (FastAPI RAG chatbot, Better-Auth).
**Performance Goals**: RAG chatbot response time < 3 seconds (as clarified in spec).
**Constraints**: 20,000–35,000 words for textbook content, Markdown source for Docusaurus, diagrams with open tools, 13-week project timeline, all practical work confined to simulations and edge computing kits (no physical robot deployment for student exercises).
**Scale/Scope**: Four core modules covering Physical AI, ROS 2, Digital Twins, AI-Robot Brain, and VLA systems across 13 weeks, including learning objectives, practical exercises, labs, personalization, and translation features for logged-in users.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

-   **I. AI/Spec-Driven Book Creation & Deployment**: PASSED. The project explicitly uses SDD, Docusaurus, GitHub Pages, Claude Code, and Spec-Kit Plus for book creation and deployment.
-   **II. Integrated RAG Chatbot Development**: PASSED. A RAG chatbot using specified technologies is a core deliverable, including content-based and selected-text querying.
-   **III. Reusable Intelligence with Subagents & Skills**: PASSED. The plan will incorporate the development of Claude Code Subagents and Agent Skills for enhanced automation and efficiency.
-   **IV. Secure & Personalized User Experience**: PASSED. Better-Auth is mandated for signup/signin, with user background data collected for content personalization.
-   **V. Multi-lingual Content Support**: PASSED. Urdu translation for chapter content for logged-in users is a defined feature.
-   **VI. Test-First Development (NON-NEGOTIABLE)**: PASSED. TDD is mandated, with tests written and approved before implementation, following a Red-Green-Refactor cycle.

All core principles outlined in the constitution are adhered to.

## Project Structure

### Documentation (this feature)

```text
specs/1-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
my-website/              # Docusaurus project for the textbook
├── docs/                # Markdown files for textbook chapters/modules
│   ├── module1/
│   ├── module2/
│   └── ...
├── src/                 # Docusaurus components, custom pages
├── static/              # Static assets (images, diagrams)
└── docusaurus.config.js # Docusaurus configuration

chatbot-backend/         # FastAPI RAG Chatbot
├── app/
│   ├── main.py
│   ├── routers/
│   ├── services/
│   └── models/
├── tests/
└── requirements.txt

auth-service/            # Better-Auth integration (conceptual)
├── src/
│   ├── auth_api.py
│   └── models.py
└── requirements.txt

agents-skills/           # Claude Code Subagents and Agent Skills
├── subagents/
├── skills/
└── README.md
```

**Structure Decision**: The project will adopt a multi-repository (or monorepo with distinct logical areas) structure to separate the Docusaurus-based textbook content, the FastAPI RAG chatbot backend, and the authentication service. Dedicated directories for Claude Code subagents and skills will also be established. This provides clear separation of concerns, allows independent deployment of services, and aligns with the modular nature of the project.

## Phases

### Phase 0: Research & Setup

**Goal**: Establish foundational knowledge, set up development environments, and create initial content scaffolding.

1.  **Environment Setup (Local & Cloud)**
    *   **Task**: Document detailed steps for setting up local development environments (Ubuntu 22.04 LTS, Python, ROS 2, Gazebo, Unity).
    *   **Task**: Research and document steps for setting up cloud-native lab instances (AWS/Azure) with NVIDIA Isaac Sim on Omniverse Cloud.
    *   **Deliverable**: Comprehensive `setup-guides/local-env.md` and `setup-guides/cloud-env.md`.

2.  **Docusaurus Project Initialization**
    *   **Task**: Initialize the Docusaurus project (`my-website/`).
    *   **Task**: Configure Docusaurus for multi-version documentation if necessary (future scaling).
    *   **Deliverable**: `my-website/` directory structure, initial `docusaurus.config.js`.

3.  **Core Content Research & Outline**
    *   **Task**: Deep dive into official documentation for ROS 2, Gazebo, Unity, NVIDIA Isaac Sim/ROS, Jetson, and Unitree to gather authoritative content and examples.
    *   **Task**: Generate detailed outlines for each of the four modules, including learning objectives, key topics, and preliminary practical exercises.
    *   **Deliverable**: `specs/1-physical-ai-textbook/research.md` with consolidated findings, and `my-website/docs/moduleX/outline.md` for each module.

4.  **RAG Chatbot Technology Deep Dive**
    *   **Task**: Research best practices for integrating OpenAI Agents/ChatKit SDKs with FastAPI.
    *   **Task**: Investigate optimal schema design for Neon Serverless Postgres and data ingestion strategies for Qdrant Cloud Free Tier to store textbook content embeddings.
    *   **Deliverable**: `specs/1-physical-ai-textbook/research.md` expanded with chatbot integration patterns.

### Phase 1: Foundation & Initial Content Draft

**Goal**: Implement core infrastructure, draft initial textbook content for Module 1, and set up basic RAG chatbot functionality.

1.  **Module 1 Content Creation (ROS 2)**
    *   **Task**: Draft content for "Module 1: The Robotic Nervous System (ROS 2)", covering nodes, topics, services, rclpy, and URDF.
    *   **Task**: Develop initial ROS 2 Python packages for basic robot control examples to accompany Module 1.
    *   **Deliverable**: `my-website/docs/module1/index.md`, `my-website/docs/module1/exercises.md`, `robot-code/ros2-pkgs/` with examples.

2.  **RAG Chatbot Backend Development (MVP)**
    *   **Task**: Set up FastAPI application (`chatbot-backend/`).
    *   **Task**: Integrate Neon Serverless Postgres for data storage (e.g., textbook content metadata, user interaction logs).
    *   **Task**: Implement basic Qdrant integration for vector database functionality, starting with a small subset of textbook content embeddings.
    *   **Task**: Develop initial RAG endpoint capable of answering questions from the embedded content.
    *   **Deliverable**: `chatbot-backend/app/main.py`, `requirements.txt`, basic Qdrant index.

3.  **Better-Auth Integration (Signup/Signin)**
    *   **Task**: Research Better-Auth API for signup and signin flows.
    *   **Task**: Implement basic signup and signin endpoints within a dedicated service or integrated into the Docusaurus front-end.
    *   **Task**: Design and implement user profile storage for software/hardware background.
    *   **Deliverable**: Authentication service setup, user registration, and login functionality.

### Phase 2: Advanced Features & Module Expansion

**Goal**: Expand textbook content, enhance RAG chatbot, implement personalization and translation, and develop reusable intelligence components.

1.  **Textbook Content Expansion (Modules 2-4)**
    *   **Task**: Draft content for "Module 2: The Digital Twin (Gazebo & Unity)", "Module 3: The AI-Robot Brain (NVIDIA Isaac™)", and "Module 4: Vision-Language-Action (VLA)".
    *   **Task**: Develop corresponding practical exercises and labs, including Gazebo simulation assets, Unity visualization setups, NVIDIA Isaac perception pipelines, and VLA examples.
    *   **Deliverable**: `my-website/docs/moduleX/index.md`, `my-website/docs/moduleX/exercises.md` for Modules 2-4, associated code/assets.

2.  **Personalization Feature Implementation**
    *   **Task**: Develop Docusaurus theme or component to allow logged-in users to toggle content personalization.
    *   **Task**: Implement logic to dynamically adjust content based on user's stored software/hardware background.
    *   **Deliverable**: Personalized content rendering for chapters.

3.  **Urdu Translation Feature Implementation**
    *   **Task**: Research and select a translation API or library for Urdu translation (e.g., Google Translate API).
    *   **Task**: Implement Docusaurus component for one-click Urdu translation of chapter content for logged-in users.
    *   **Deliverable**: Functional Urdu translation for textbook chapters.

4.  **RAG Chatbot Enhancement**
    *   **Task**: Refine RAG chatbot to answer questions based *only* on user-selected text, using contextual querying techniques for Qdrant.
    *   **Task**: Implement comprehensive data ingestion pipeline to automatically embed all textbook content into Qdrant.
    *   **Deliverable**: Advanced RAG capabilities meeting spec requirements.

5.  **Reusable Intelligence (Subagents & Skills)**
    *   **Task**: Identify repetitive or complex development tasks within the project that can be automated.
    *   **Task**: Develop Claude Code Subagents and Agent Skills (e.g., for Docusaurus content generation, code example validation, API contract generation).
    *   **Deliverable**: `agents-skills/` with working subagents and skills.

### Phase 3: Synthesis, Review & Deployment

**Goal**: Integrate all components, perform comprehensive testing, finalize content, and deploy the complete textbook.

1.  **End-to-End Integration**
    *   **Task**: Integrate the Docusaurus frontend with the FastAPI chatbot backend and the Better-Auth service.
    *   **Task**: Ensure seamless flow between textbook content, chatbot interactions, personalization, and translation.
    *   **Deliverable**: Fully integrated system.

2.  **Comprehensive Testing & Validation**
    *   **Task**: Execute all reproducibility checks for code examples and simulations.
    *   **Task**: Conduct thorough testing of all ROS packages, Gazebo scenes, Isaac pipelines, and edge kit exercises.
    *   **Task**: Perform extensive RAG chatbot response verification, including accuracy and response time (< 3 seconds).
    *   **Task**: Validate personalization logic and Urdu translation quality.
    *   **Deliverable**: Test reports, bug fixes, passing automated tests.

3.  **Content Finalization**
    *   **Task**: Review all textbook content for clarity, accuracy, consistency, and adherence to word count (20,000–35,000 words).
    *   **Task**: Ensure all diagrams are created using open tools and correctly integrated.
    *   **Task**: Verify all references to official documentation are accurate and complete.
    *   **Deliverable**: Polished textbook content.

4.  **Deployment to GitHub Pages**
    *   **Task**: Configure Docusaurus for deployment to GitHub Pages.
    *   **Task**: Set up CI/CD pipeline for automated deployment of the textbook.
    *   **Deliverable**: Live textbook on GitHub Pages.

## Research Approach

The project will adopt a research-concurrent method, where detailed investigations into technologies, best practices, and official documentation will run in parallel with content creation and development phases. Each module will include a dedicated section for "Sources & References" to document all external materials used. APA citation style will be followed for all academic and technical references.

### Placeholder for Module Sources & References

*   **Module 1: The Robotic Nervous System (ROS 2)**
    *   [Source 1 Title](URL)
    *   [Source 2 Title](URL)
*   **Module 2: The Digital Twin (Gazebo & Unity)**
    *   [Source 1 Title](URL)
    *   [Source 2 Title](URL)
*   **Module 3: The AI-Robot Brain (NVIDIA Isaac™)**
    *   [Source 1 Title](URL)
    *   [Source 2 Title](URL)
*   **Module 4: Vision-Language-Action (VLA)**
    *   [Source 1 Title](URL)
    *   [Source 2 Title](URL)

## Quality Validation / Testing Strategy

A multi-faceted quality validation and testing strategy will be employed to ensure the accuracy, reproducibility, and functionality of the textbook and its integrated components.

1.  **Reproducibility Checks**: All code examples, scripts, and simulation setups provided within the textbook chapters will undergo rigorous reproducibility checks to ensure students can successfully execute them on both local high-performance workstations and cloud-based GPU setups.
2.  **ROS Package Validation**: Developed ROS 2 packages will be unit-tested and integrated into simulated environments to validate their functionality, adherence to ROS 2 standards, and correct interaction with robot components.
3.  **Gazebo & Unity Scene Validation**: Gazebo simulation environments and Unity visualization setups will be tested for accurate physics, sensor emulation, and proper rendering of robot models.
4.  **NVIDIA Isaac Pipeline Validation**: AI-powered perception and manipulation pipelines developed with NVIDIA Isaac SDK (Isaac Sim, Isaac ROS) will be tested against various scenarios to confirm their robustness and accuracy.
5.  **Edge Kit Exercise Validation**: Exercises designed for NVIDIA Jetson Edge AI kits will be validated on target hardware to ensure correct deployment and execution in resource-constrained environments.
6.  **RAG Chatbot Response Verification**: The RAG chatbot's responses will be systematically verified against the textbook content for accuracy (SC-003: 95% accuracy) and context-awareness (SC-004: 90% accuracy for selected text). Response time will also be monitored to ensure it meets the `< 3 seconds` target.
7.  **User Experience Testing**: Personalization and Urdu translation features will undergo user acceptance testing to ensure they enhance the learning experience as intended, are intuitive to use, and meet defined quality metrics (SC-005: 100% correct personalization; SC-006: 85%+ Urdu translation quality).

## Decisions Needing Documentation

The following architectural and design decisions will require explicit documentation (e.g., via ADRs) to capture options, trade-offs, and final rationale:

1.  **Hardware Selection (Local vs. Cloud Infrastructure)**
    *   **Options**: Primarily local high-performance workstations + Edge AI kits vs. predominantly cloud-native (AWS/Azure) with local "bridge" hardware.
    *   **Pros/Cons**:
        *   **Local**: Higher initial CapEx, lower OpEx, lower latency for physical interaction.
        *   **Cloud**: Lower initial CapEx, higher OpEx, higher latency for physical interaction (training in cloud, model deployment to local edge).
    *   **Rationale**: The current plan focuses on providing options and documentation for both, with a strong recommendation for local workstations + edge kits for optimal learning, while acknowledging cloud as a viable (but more costly) alternative for students without powerful local hardware. Actual student choices will dictate the mix. *Decision already captured in Constitution.*

2.  **Simulation vs. Physical Robot Deployment for Exercises**
    *   **Options**: Confine all student exercises to simulation and edge computing kits vs. include direct deployment to physical robots.
    *   **Pros/Cons**:
        *   **Simulation/Edge**: Cost-effective, safe, reproducible, accessible for all students, aligns with "Not building: Full humanoid robot deployment guide beyond simulated or edge kit" constraint.
        *   **Physical**: Higher cost, safety concerns, limited accessibility, complexity in setup and debugging.
    *   **Rationale**: All practical exercises and labs will be strictly confined to simulations (Gazebo, Isaac Sim) and NVIDIA Jetson Edge AI kits. This decision aligns with the project's "Not building" scope and ensures accessibility and safety for all students. *Decision already clarified in Spec.*

3.  **RAG Chatbot Framework and Data Stack**
    *   **Options**: OpenAI Agents/ChatKit SDKs with FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier vs. alternative LLM frameworks/vector databases/backends.
    *   **Pros/Cons**: Specific choices offer a robust, scalable, and cost-effective solution for hackathon constraints.
    *   **Rationale**: The specified stack (OpenAI, FastAPI, Neon, Qdrant) has been chosen for its modern capabilities, scalability, and alignment with the hackathon's requirements, offering a comprehensive solution for RAG functionality.

4.  **User Authentication and Personalization Framework**
    *   **Options**: Better-Auth for signup/signin and custom implementation for personalization logic vs. other authentication providers/personalization engines.
    *   **Pros/Cons**: Better-Auth simplifies authentication, custom personalization allows fine-grained control over content adaptation.
    *   **Rationale**: Better-Auth provides a streamlined solution for authentication, and a custom approach to personalization ensures direct control over how content adapts to user backgrounds.

5.  **Multi-language Translation Strategy**
    *   **Options**: In-browser translation API (e.g., Google Translate) vs. pre-translated content vs. a dedicated translation service.
    *   **Pros/Cons**: In-browser API offers dynamic translation but relies on external service; pre-translation ensures quality but increases content management; dedicated service offers control but adds complexity.
    *   **Rationale**: An in-browser API solution (e.g., Google Translate API) is initially planned for Urdu translation due to its ease of integration and dynamic capabilities, balancing functionality with development effort.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | No constitution violations identified. | N/A |

# Tasks: Physical AI & Humanoid Robotics — Technical Textbook

**Input**: Design documents from `/specs/1-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The implementation plan implies comprehensive testing, so test tasks are included where relevant.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `my-website/docs/`, `setup-guides/`
- **Chatbot Backend**: `chatbot-backend/`
- **Auth Service**: `auth-service/`
- **Robot Code Examples**: `robot-code/`
- **Agents/Skills**: `agents-skills/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Docusaurus project structure `my-website/`
- [ ] T002 Configure Docusaurus `my-website/docusaurus.config.js`
- [ ] T003 Create chatbot backend project structure `chatbot-backend/`
- [ ] T004 Create authentication service project structure `auth-service/`
- [ ] T005 Create agents and skills directory `agents-skills/`
- [ ] T006 Document local environment setup `setup-guides/local-env.md`
- [ ] T007 Document cloud environment setup `setup-guides/cloud-env.md`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T008 Deep dive research on ROS 2, Gazebo, Unity, NVIDIA Isaac `specs/1-physical-ai-textbook/research.md`
- [ ] T009 Generate detailed outlines for Module 1 `my-website/docs/module1/outline.md`
- [ ] T009.1 Generate detailed outlines for Module 2 `my-website/docs/module2/outline.md`
- [ ] T009.2 Generate detailed outlines for Module 3 `my-website/docs/module3/outline.md`
- [ ] T009.3 Generate detailed outlines for Module 4 `my-website/docs/module4/outline.md`
- [ ] T009.4 Create detailed 13-week breakdown `my-website/docs/weekly-breakdown.md`
- [ ] T010 Research OpenAI Agents/ChatKit SDKs with FastAPI integration `specs/1-physical-ai-textbook/research.md`
- [ ] T011 Research Neon Serverless Postgres and Qdrant Cloud Free Tier for embeddings `specs/1-physical-ai-textbook/research.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learn Physical AI Foundations (Priority: P1) 🎯 MVP

**Goal**: Students understand the foundational principles of Physical AI and embodied intelligence.

**Independent Test**: Review module quizzes/exercises and assess comprehension of key concepts.

### Implementation for User Story 1

- [ ] T012 [P] [US1] Draft content for Module 1 `my-website/docs/module1/index.md`
- [ ] T013 [P] [US1] Develop ROS 2 Python packages for basic robot control examples `robot-code/ros2-pkgs/`
- [ ] T014 [US1] Draft exercises for Module 1 `my-website/docs/module1/exercises.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Master ROS 2 for Robot Control (Priority: P1)

**Goal**: Students learn how to use ROS 2 to control robots and develop robotic applications.

**Independent Test**: Successfully complete the ROS 2 package development project and demonstrate basic robot control.

### Implementation for User Story 2

- [ ] T015 [P] [US2] Draft content for Module 2 `my-website/docs/module2/index.md`
- [ ] T016 [P] [US2] Develop Gazebo simulation assets `robot-code/gazebo-assets/`
- [ ] T017 [P] [US2] Develop Unity visualization setups `robot-code/unity-setups/`
- [ ] T018 [US2] Draft exercises for Module 2 `my-website/docs/module2/exercises.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Simulate Robots with Gazebo & Unity (Priority: P1)

**Goal**: Students can simulate robots and their environments using Gazebo and Unity to test and refine robot control algorithms.

**Independent Test**: Set up a Gazebo environment, simulate a robot, and visualize it in Unity.

### Implementation for User Story 3

- [ ] T019 [P] [US3] Draft content for Module 3 `my-website/docs/module3/index.md`
- [ ] T020 [P] [US3] Develop NVIDIA Isaac perception pipelines `robot-code/isaac-pipelines/`
- [ ] T021 [US3] Draft exercises for Module 3 `my-website/docs/module3/exercises.md`

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 6 - Utilize RAG Chatbot for Learning (Priority: P1)

**Goal**: Readers can use an integrated RAG chatbot to ask questions about the textbook content and get immediate, relevant answers.

**Independent Test**: Ask the chatbot various questions about the textbook and verify accuracy and relevance of responses.

### Implementation for User Story 6

- [ ] T022 [P] [US6] Set up FastAPI application `chatbot-backend/app/main.py`
- [ ] T023 [P] [US6] Integrate Neon Serverless Postgres `chatbot-backend/app/services/db.py`
- [ ] T024 [P] [US6] Implement basic Qdrant integration `chatbot-backend/app/services/vector_db.py`
- [ ] T025 [US6] Develop initial RAG endpoint `chatbot-backend/app/routers/chatbot.py`
- [ ] T026 [US6] Implement comprehensive data ingestion pipeline for Qdrant `chatbot-backend/scripts/ingest.py`
- [ ] T027 [US6] Refine RAG chatbot for selected text querying `chatbot-backend/app/services/rag.py`

---

## Phase 7: User Story 4 - Develop with NVIDIA Isaac AI Platform (Priority: P2)

**Goal**: Students utilize the NVIDIA Isaac SDK for AI-powered perception and manipulation to implement advanced robot behaviors.

**Independent Test**: Implement an Isaac-based perception pipeline and demonstrate object recognition or navigation.

### Implementation for User Story 4

- [ ] T028 [P] [US4] Draft content for Module 4 `my-website/docs/module4/index.md`
- [ ] T029 [P] [US4] Develop VLA system examples `robot-code/vla-examples/`
- [ ] T030 [US4] Draft exercises for Module 4 `my-website/docs/module4/exercises.md`

---

## Phase 8: User Story 5 - Integrate Conversational AI with Robots (Priority: P2)

**Goal**: Students integrate GPT models for conversational AI in robots to create more natural human-robot interactions.

**Independent Test**: Demonstrate a simulated robot responding to voice commands and executing a sequence of actions.

### Implementation for User Story 5

- [ ] T031 [P] [US5] Research Better-Auth API for signup and signin flows `auth-service/README.md`
- [ ] T032 [P] [US5] Implement basic signup and signin endpoints `auth-service/src/auth_api.py`
- [ ] T033 [P] [US5] Design and implement user profile storage `auth-service/src/models.py`

---

## Phase 9: User Story 7 - Personalized Content (Priority: P3)

**Goal**: Logged-in users can personalize chapter content based on their background, tailoring the learning experience.

**Independent Test**: Create user profiles with different backgrounds and observe content adaptations.

### Implementation for User Story 7

- [ ] T034 [P] [US7] Develop Docusaurus theme/component for personalization toggle `my-website/src/components/PersonalizationToggle.js`
- [ ] T035 [US7] Implement logic to dynamically adjust content based on user background `my-website/src/theme/DocItem/Content/index.js`

---

## Phase 10: User Story 8 - Urdu Content Translation (Priority: P3)

**Goal**: Logged-in users can translate chapter content into Urdu to learn in their preferred language.

**Independent Test**: Activate the translation feature in a chapter and verify the accuracy of the Urdu translation.

### Implementation for User Story 8

- [ ] T036 [P] [US8] Research and select translation API/library for Urdu `my-website/src/utils/translation.js`
- [ ] T037 [US8] Implement Docusaurus component for Urdu translation `my-website/src/components/UrduTranslationButton.js`

---

## Phase 11: Reusable Intelligence (Subagents & Skills)

**Purpose**: Develop Claude Code Subagents and Agent Skills for enhanced automation and efficiency.

- [ ] T038 [P] Identify repetitive tasks for Claude Code Subagents/Skills `agents-skills/README.md`
- [ ] T039 [P] Develop Claude Code Subagents `agents-skills/subagents/`
- [ ] T040 [P] Develop Claude Code Agent Skills `agents-skills/skills/`

---

## Phase 12: Synthesis, Review & Deployment

**Purpose**: Integrate all components, perform comprehensive testing, finalize content, and deploy the complete textbook.

- [ ] T041 [P] End-to-end integration of Docusaurus, Chatbot, and Auth `my-website/docusaurus.config.js`
- [ ] T042 [P] Reproducibility checks for all code examples/simulations `tests/reproducibility.py`
- [ ] T043 [P] ROS package validation `robot-code/ros2-pkgs/tests/`
- [ ] T044 [P] Gazebo & Unity scene validation `robot-code/gazebo-assets/tests/`
- [ ] T045 [P] NVIDIA Isaac pipeline validation `robot-code/isaac-pipelines/tests/`
- [ ] T046 [P] Edge Kit exercise validation `robot-code/edge-kit-exercises/tests/`
- [ ] T047 [P] RAG chatbot response verification `chatbot-backend/tests/rag_e2e.py`
- [ ] T048 [P] User experience testing for personalization/translation `my-website/tests/e2e.spec.js`
- [ ] T049 [P] Review all textbook content for clarity, accuracy, consistency `my-website/docs/`
- [ ] T049.1 [P] Verify textbook word count (20,000-35,000 words) `my-website/docs/`
- [ ] T050 [P] Ensure all diagrams use open tools `my-website/static/img/`
- [ ] T051 [P] Verify all references to official documentation `my-website/docs/`
- [ ] T052 [P] Configure Docusaurus for GitHub Pages deployment `my-website/docusaurus.config.js`
- [ ] T053 [P] Set up CI/CD pipeline for automated deployment `.github/workflows/deploy.yml`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-10)**: All depend on Foundational phase completion
  - Can then proceed in parallel (if staffed) or sequentially in priority order
- **Reusable Intelligence (Phase 11)**: Can run in parallel with User Story phases, but benefits from understanding content and features.
- **Synthesis, Review & Deployment (Phase 12)**: Depends on all previous phases being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 6 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3 but should be independently testable
- **User Story 5 (P2)**: Can start after Foundational (Phase 2) - May integrate with US7 but should be independently testable
- **User Story 7 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 8 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks can run in parallel once research is done.
- Once Foundational phase completes, user stories can start in parallel (if team capacity allows), respecting internal story task dependencies.
- All tasks marked [P] within a phase or story can run in parallel.
- Different user stories can be worked on in parallel by different team members.

---

## Parallel Example: User Story 1

```bash
# Tasks that can run in parallel:
Task: "Draft content for Module 1 in my-website/docs/module1/index.md"
Task: "Develop ROS 2 Python packages for basic robot control examples in robot-code/ros2-pkgs/"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 3, 6 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all user stories)
3.  Complete Phase 3: User Story 1 (Learn Physical AI Foundations)
4.  Complete Phase 4: User Story 2 (Master ROS 2 for Robot Control)
5.  Complete Phase 5: User Story 3 (Simulate Robots with Gazebo & Unity)
6.  Complete Phase 6: User Story 6 (Utilize RAG Chatbot for Learning)
7.  **STOP and VALIDATE**: Test User Stories 1, 2, 3, 6 independently and as an integrated core product.
8.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Stories 1, 2, 3, 6 → Test independently → Deploy/Demo (MVP!)
3.  Add User Stories 4, 5 → Test independently → Deploy/Demo
4.  Add User Stories 7, 8 → Test independently → Deploy/Demo
5.  Add Reusable Intelligence (Subagents & Skills)
6.  Final Synthesis, Review & Deployment
7.  Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 1, 2, 3 (Core Content & Simulation)
    -   Developer B: User Story 6 (RAG Chatbot)
    -   Developer C: User Story 4, 5 (Isaac & Conversational AI)
    -   Developer D: User Story 7, 8 (Personalization & Translation)
3.  Reusable Intelligence development can be a dedicated effort or distributed.
4.  Stories complete and integrate independently.

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (if TDD is applied)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

# Research: Physical AI & Humanoid Robotics

This document consolidates research findings for the "Physical AI & Humanoid Robotics" textbook project.

## Phase 2: Foundational Research

### T008: Deep Dive on Core Robotics Technologies

This section covers the foundational technologies for the textbook's practical exercises.

#### ROS 2 (Robot Operating System 2)

*   **Official Documentation**: [docs.ros.org](https://docs.ros.org/)
*   **Key Concepts**: ROS 2 is an open-source set of software libraries and tools for building robot applications. It provides services like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.
*   **Relevance**: Core of Module 1. Students will learn about nodes, topics, services, and `rclpy` to control simulated robots.

#### Gazebo Simulator

*   **Official Documentation**: [gazebosim.org](https://gazebosim.org/)
*   **Key Concepts**: Gazebo is a 3D robotics simulator that allows for the development, testing, and validation of robot designs and algorithms in realistic environments. It supports high-fidelity physics, a wide range of sensors, and a pluggable architecture.
*   **Relevance**: Core of Module 2. Students will use Gazebo to create simulation worlds and test their ROS 2-based robot control code.

#### Unity for Robotics

*   **Official Hub**: [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
*   **Key Concepts**: Unity is a powerful real-time 3D development platform. For robotics, it provides tools for simulation, visualization, and training of ML agents. The Unity Robotics Hub provides packages for URDF import, ROS integration (ROS TCP Connector), and various examples.
*   **Relevance**: Part of Module 2. Students will learn to use Unity for high-quality visualization of robot simulations running in Gazebo or directly in Unity.

#### NVIDIA Isaac

*   **Official Platform**: [NVIDIA Isaac Platform](https://www.nvidia.com/en-us/robotics/developers/isaac/)
*   **Key Concepts**: NVIDIA Isaac is a comprehensive platform for developing and deploying AI-powered robots. It includes:
    *   **Isaac Sim**: A photorealistic, physically-accurate simulator built on NVIDIA Omniverse.
    *   **Isaac ROS**: A collection of hardware-accelerated packages for ROS 2, providing AI and computer vision functionalities.
    *   **Isaac Lab**: A framework for robot learning.
*   **Relevance**: Core of Module 3. Students will use Isaac Sim for advanced simulations and Isaac ROS for perception and AI tasks.

---

### T010: RAG Chatbot Technology Deep Dive

*   **Key Technologies**: OpenAI Agents/ChatKit SDKs and FastAPI.
*   **Integration Pattern**:
    *   **Backend**: FastAPI will be used to create a robust and scalable backend for the RAG chatbot. Its asynchronous nature is well-suited for handling requests to the OpenAI API.
    *   **Agent Logic**: The OpenAI Agents SDK will be used to define the logic of the chatbot, including its instructions, capabilities, and tools.
    *   **Frontend**: The ChatKit SDK can be used to create a rich user interface for the chatbot, which can be embedded into the Docusaurus site. For this project, a self-hosted backend with FastAPI is the chosen approach for more control.
    *   **Communication**: FastAPI endpoints will receive user messages from the frontend, pass them to the OpenAI agent, and stream the responses back to the user.

---

### T011: RAG Chatbot Data Stack Research

*   **Key Technologies**: Neon Serverless Postgres and Qdrant Cloud.
*   **Data Storage**:
    *   **Neon Serverless Postgres**: Neon will be used as the primary data store for the textbook content and relational data. It supports the `pgvector` extension, which allows for storing and querying vector embeddings directly within Postgres. This simplifies the architecture by keeping all data in one place.
    *   **Qdrant Cloud**: Qdrant is a specialized vector database that is optimized for high-performance similarity search. While `pgvector` is sufficient for this project's scale, Qdrant could be considered for future scalability or if more advanced filtering and search capabilities are needed. For this project, we will start with `pgvector` in Neon.
*   **Workflow**:
    1.  Textbook content will be chunked and converted into vector embeddings using an OpenAI model.
    2.  These embeddings will be stored in a `vector` column in a Neon Postgres database, alongside the original text and other metadata.
    3.  When a user asks a question, the question will be converted into an embedding, and a similarity search will be performed on the Neon database to find the most relevant chunks of text.
    4.  These chunks of text will be used to augment the prompt sent to the OpenAI agent, which will then generate a response.

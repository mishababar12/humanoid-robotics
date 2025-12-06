# ROS 2: Week 1 Overview - Getting Started with ROS 2

This week serves as your foundational introduction to the Robot Operating System 2 (ROS 2). We will cover everything from setting up your development environment to understanding the core communication mechanisms that make ROS 2 so powerful for robotics development. By the end of this week, you will be able to set up a ROS 2 workspace, create basic ROS 2 packages, and implement simple publisher and subscriber nodes.

## Topics Covered

### 1. ROS 2 Core Concepts Revisited

*   **Distributed Architecture:** Understanding how ROS 2 leverages DDS (Data Distribution Service) for robust, peer-to-peer communication, eliminating the single point of failure present in ROS 1's master-slave architecture.
*   **Quality of Service (QoS):** An in-depth look at how QoS policies (reliability, durability, history, liveliness, deadline) can be configured to meet diverse application requirements, from high-fidelity sensor streams to critical control commands.
*   **The ROS 2 Graph:** Visualizing the network of nodes communicating via topics, services, and actions, and how this distributed graph forms the operational backbone of a robot system.

### 2. Installation and Setup

A successful ROS 2 development journey begins with a correctly configured environment. We will focus on the recommended installation for Ubuntu, as it offers the broadest compatibility and community support for ROS 2.

*   **Operating System Requirements:**
    *   **Ubuntu 20.04 (Foxy/Humble - LTS):** We will primarily use ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) release, built for Ubuntu 20.04.
    *   **Ubuntu 22.04 (Iron - latest LTS):** Alternatively, you can use Iron Irwini on Ubuntu 22.04.
*   **Installation Steps (Ubuntu Humble Example):**
    1.  **Set locale:** Ensure your locale is set to UTF-8.
    2.  **Add ROS 2 apt repository:** Add the official ROS 2 repository to your system's sources list.
    3.  **Install `ros-humble-desktop`:** This meta-package includes ROS 2 base, development tools, and `rviz2`.
    4.  **Install `colcon`:** The build tool for ROS 2.
    5.  **Source the setup file:** Add `source /opt/ros/humble/setup.bash` to your `~/.bashrc` to automatically set up your environment.

### 3. ROS 2 Workspace and Package Creation

A ROS 2 workspace is an organized collection of ROS 2 packages. Packages are the fundamental unit of software organization in ROS 2.

*   **Creating a Workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    ```
*   **Overlaying a Workspace:** Understanding how new workspaces can be built on top of existing ones (e.g., the base `/opt/ros/humble` installation).
*   **Creating a New Package (`ros2 pkg create`):**
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_ros2_pkg --dependencies rclpy std_msgs
    # For C++:
    # ros2 pkg create --build-type ament_cmake my_cpp_pkg --dependencies rclcpp std_msgs
    ```
    This command creates a new Python package (`my_ros2_pkg`) with `rclpy` (ROS Client Library for Python) and `std_msgs` (standard messages) as dependencies.

### 4. Nodes, Topics, and Messages in Detail

Building on the overview from the main module, we'll delve deeper into these core communication primitives.

*   **Nodes:** Executable processes that perform specific tasks. Each node should ideally adhere to the Single Responsibility Principle.
    *   **`rclpy.node.Node` (Python):** The base class for creating ROS 2 nodes in Python.
    *   **Lifecycle Nodes:** (Brief mention) Special nodes that allow for state transitions (e.g., `unconfigured`, `inactive`, `active`) for more robust system management.
*   **Topics:** Asynchronous, many-to-many communication channels.
    *   **Publishers:** Nodes that send messages to a topic (`create_publisher`).
    *   **Subscribers:** Nodes that receive messages from a topic (`create_subscription`).
    *   **`ros2 topic list`, `ros2 topic echo`, `ros2 topic info`:** Command-line tools for inspecting topics.
*   **Messages:** Structured data passed over topics.
    *   **`std_msgs`:** Common message types like `String`, `Int32`, `Float64`.
    *   **Custom Messages:** How to define your own message types (`.msg` files) for specific application data.

## Assignments

### Required Readings:
*   ROS 2 Documentation: [Concepts Guide](https://docs.ros.org/en/humble/Concepts.html)
*   ROS 2 Documentation: [About ROS 2 packages](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Creating-A-New-ROS2-Package.html)
*   ROS 2 Documentation: [Quality of Service settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

### Practical Exercises
*   **Exercise 2.1 (from main module):** Create a simple ROS 2 publisher and subscriber using a custom message type. This will reinforce your understanding of topics, messages, and package creation.
*   **Additional Exercise:** Experiment with different QoS settings (e.g., `reliability=RMW_QOS_POLICY_RELIABILITY_RELIABLE` vs. `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`) for your publisher/subscriber and observe their impact on message delivery.

### Labs
*   **Lab 2.1 (from main module):** Basic robot control in a simulated environment using ROS 2. For this week, focus on getting your development environment ready and ensuring you can compile and run the provided example code.

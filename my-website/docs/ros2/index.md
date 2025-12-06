# ROS 2 for Robot Control

This module focuses on mastering the Robot Operating System 2 (ROS 2) for developing robust and scalable robotic applications. ROS 2 is a flexible framework for writing robot software, offering a collection of tools, libraries, and conventions that simplify the complex task of building robots. It enables developers to create modular and distributed systems, crucial for the advanced physical AI systems we aim to build.

## What is ROS 2?

ROS 2 is the successor to the original Robot Operating System (ROS 1), reimagined to address the demands of modern robotics, particularly in areas like real-time control, multi-robot systems, and embedded platforms. It provides a standardized communication infrastructure for different processes (nodes) to exchange data, facilitating the development of complex robotic behaviors by breaking them down into smaller, manageable components.

### Key Features and Advantages of ROS 2 over ROS 1:

1.  **Quality of Service (QoS) Settings:** ROS 2 introduces flexible QoS policies (e.g., reliability, durability, history, deadline) that allow developers to fine-tune communication behavior based on application needs, critical for real-time and mission-critical systems.
2.  **Distributed Nature (DDS):** Built on the Data Distribution Service (DDS) standard, ROS 2 is inherently distributed, allowing seamless communication across multiple machines, networks, and even different operating systems without a central master node.
3.  **Support for Embedded Systems:** Designed with embedded systems and microcontrollers in mind, ROS 2 can run on a wider range of hardware, from powerful workstations to resource-constrained edge devices.
4.  **Security:** ROS 2 includes built-in security features (SROS 2) for authentication, authorization, and encryption, essential for deployments in sensitive environments.
5.  **Multi-Robot Support:** Its distributed architecture and improved discovery mechanisms make it ideal for coordinating multiple robots.
6.  **Real-Time Capabilities:** While not a hard real-time OS, ROS 2 provides mechanisms and best practices to improve determinism and predictability, crucial for control loops.

## Core Concepts of ROS 2

Understanding these fundamental concepts is key to developing applications with ROS 2:

### 1. Nodes

Nodes are executable processes that perform computation. In a robotic system, different functionalities are typically encapsulated in separate nodes. For instance, one node might control a motor, another might process camera data, and a third might handle navigation logic.

### 2. Topics

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, and other nodes can subscribe to that topic to receive the messages. This is a one-to-many communication mechanism, ideal for streaming data.

### 3. Messages

Messages are data structures used for communication over topics. ROS 2 provides a rich set of standard message types for common data (e.g., sensor readings, robot commands, images), and users can define custom message types.

### 4. Services

Services are a request/reply communication mechanism. A service server node offers a service, and a service client node sends a request to the server and waits for a response. This is a synchronous, one-to-one communication pattern, suitable for operations that return a result.

### 5. Actions

Actions are a long-running, asynchronous communication mechanism. An action client sends a goal to an action server, which then provides continuous feedback on its progress and a final result. This is ideal for tasks like "navigate to a point" or "pick up an object," where progress updates are valuable.

## Learning Objectives
*   Understand the fundamental architecture and design principles of ROS 2.
*   Differentiate between ROS 1 and ROS 2, recognizing the advancements and motivations behind the latter.
*   Master core ROS 2 communication concepts: Nodes, Topics, Messages, Services, and Actions.
*   Develop and deploy basic ROS 2 packages using Python.
*   Gain proficiency in controlling simulated robots through ROS 2 interfaces.

## Simple ROS 2 Publisher-Subscriber Example (Python)

Let's illustrate the concept of topics with a basic "talker-listener" example.

### Publisher (`talker.py`)

This node will publish a simple "Hello ROS 2" message every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber (`listener.py`)

This node will subscribe to the 'topic' and print the received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this example:
1.  Save `talker.py` and `listener.py` in a new ROS 2 package.
2.  Build the package.
3.  Open two terminals. In the first, run `ros2 run <your_package_name> talker`.
4.  In the second, run `ros2 run <your_package_name> listener`.

You should see the listener node receiving and printing the messages published by the talker.

## Practical Exercises
*   **Exercise 2.1:** Create a simple ROS 2 publisher and subscriber using a custom message type.
*   **Exercise 2.2:** Implement a ROS 2 service client and server, where the client sends two numbers and the server returns their sum.

## Labs
*   **Lab 2.1:** Basic robot control in a simulated environment using ROS 2. This lab will involve sending `Twist` messages to a simulated robot to control its linear and angular velocities, and monitoring its odometry data.

## Sections
*   [Week 1 Overview](week1_overview.md) (Note: This is an introductory week for ROS 2)
*   [Exercise 2.1 Solution](exercises/exercise1.md)
*   [Lab 2.1 Guide](labs/lab1.md)

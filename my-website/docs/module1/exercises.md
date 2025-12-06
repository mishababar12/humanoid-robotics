---
sidebar_position: 2
---

# Module 1 Exercises

These exercises are designed to accompany Module 1 of the "Physical AI & Humanoid Robotics" textbook.

## Exercise 1: Installing ROS 2

Follow the official ROS 2 documentation to install ROS 2 on your system. We recommend using Ubuntu 22.04.

## Exercise 2: Creating a ROS 2 Workspace and Node

1.  Create a new ROS 2 workspace.
2.  Create a new ROS 2 package named `my_robot_pkg`.
3.  Create a new Python file in your package named `hello_node.py`.
4.  In this file, write a simple ROS 2 node that prints "Hello, World!" to the console.
5.  Build and run your node.

## Exercise 3: Implementing a Publisher/Subscriber System

1.  In your `my_robot_pkg` package, create a new Python file named `publisher_node.py`.
2.  In this file, write a ROS 2 node that publishes a `std_msgs/String` message to a topic named `my_topic` every second. The message should contain the string "Hello, ROS 2!".
3.  Create another Python file named `subscriber_node.py`.
4.  In this file, write a ROS 2 node that subscribes to the `my_topic` topic and prints the received message to the console.
5.  Build and run both nodes to see the communication in action.

## Exercise 4: Building a Service

1.  In your `my_robot_pkg` package, create a new Python file named `add_two_ints_server.py`.
2.  In this file, write a ROS 2 node that provides a service named `add_two_ints` that takes two integers as input and returns their sum.
3.  Create another Python file named `add_two_ints_client.py`.
4.  In this file, write a ROS 2 node that calls the `add_two_ints` service with two numbers and prints the result to the console.
5.  Build and run both nodes to test the service.

## Exercise 5: Writing a Basic URDF Model

1.  Create a new file in your `my_robot_pkg` package named `simple_robot.urdf`.
2.  In this file, write a simple URDF model for a robot with a base link and two wheel links.
3.  Use the `check_urdf` command to verify that your URDF file is valid.
4.  (Optional) Visualize your robot model in RViz2.
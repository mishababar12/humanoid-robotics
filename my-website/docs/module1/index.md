---
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

Welcome to Module 1 of the "Physical AI & Humanoid Robotics" textbook. In this module, you will learn about the Robot Operating System 2 (ROS 2), the foundational software framework for modern robotics. By the end of this module, you will be able to build and run your own simple robotic applications.

## Introduction to ROS 2

### What is ROS 2?

ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It is not an operating system in the traditional sense, but rather a flexible framework for writing robot software. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### ROS 2 Architecture (DDS)

ROS 2 is built on top of the Data Distribution Service (DDS), an industry-standard middleware for real-time systems. DDS provides a publish-subscribe communication model, which allows for decoupled and scalable communication between different parts of a robotic system.

### Installation and Setup

Before you can start using ROS 2, you need to install it on your system. We recommend using a Linux-based operating system, such as Ubuntu 22.04. Detailed installation instructions can be found in the official ROS 2 documentation.

## Core Concepts

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. In a robotic system, you might have a node for controlling the motors, a node for reading sensor data, and a node for planning the robot's path.

### Topics

Topics are named buses over which nodes exchange messages. Topics have a publish-subscribe communication model. A node can publish messages to a topic, and any node that is subscribed to that topic will receive the messages.

### Services

Services are another way for nodes to communicate with each other. They are based on a request-response model. One node offers a service, and another node can call that service with a request and wait for a response.

### Actions

Actions are similar to services, but they are designed for long-running tasks. They provide feedback on the progress of the task and can be preempted. For example, you might use an action to tell a robot to move to a certain location.

### Parameters

Parameters are values that can be configured for a node at runtime. They are used to change the behavior of a node without having to recompile the code.

## Programming with `rclpy`

`rclpy` is the Python client library for ROS 2. It allows you to write ROS 2 nodes in Python.

### Creating a ROS 2 package

A ROS 2 package is a directory that contains your ROS 2 code, along with a `package.xml` file that describes the package.

### Writing a simple publisher and subscriber

A common pattern in ROS 2 is to have a node that publishes data to a topic, and another node that subscribes to that topic. For example, a sensor node might publish sensor data to a topic, and a processing node might subscribe to that topic to process the data.

### Writing a simple service client and server

Another common pattern is to have a node that provides a service, and another node that calls that service. For example, a "calculator" node might provide a service to add two numbers, and another node could call that service to get the sum.

## Robot Modeling with URDF

### What is URDF?

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It describes the physical properties of the robot, such as its links, joints, and sensors.

### URDF Syntax and Structure

A URDF file consists of a set of `link` and `joint` elements. A `link` element describes a rigid part of the robot, and a `joint` element describes how two links are connected.

### Creating a simple robot model

You can create a URDF model for your robot by creating a file with a `.urdf` extension and writing the XML code to describe your robot's structure.
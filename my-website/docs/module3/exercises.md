---
sidebar_position: 2
---

# Module 3 Exercises

These exercises are designed to accompany Module 3 of the "Physical AI & Humanoid Robotics" textbook.

## Exercise 1: Setting up Isaac Sim

1.  Follow the official NVIDIA documentation to install Omniverse Launcher and Isaac Sim.
2.  Launch Isaac Sim and explore the user interface.
3.  Open one of the provided example scenes, such as the warehouse scene.

## Exercise 2: Importing a robot and environment into Isaac Sim

1.  Import your URDF robot model from Module 1 into Isaac Sim.
2.  Create a simple environment for your robot with a ground plane and some simple shapes.
3.  Add a camera to your robot and view the camera feed in Isaac Sim.

## Exercise 3: Using an Isaac ROS package for object detection

1.  Launch the Isaac ROS Dev environment using the provided Docker container.
2.  Run the Isaac ROS object detection example on a sample image.
3.  (Optional) Connect the object detection node to the camera feed from your Isaac Sim simulation.

## Exercise 4: Integrating the object detection output into a ROS 2 application

1.  Create a new ROS 2 Python node that subscribes to the output of the Isaac ROS object detection node.
2.  In your node, write a callback function that processes the detected objects.
3.  For example, you could have your robot print a message to the console whenever it detects a certain object.

## Exercise 5: (Optional) A brief introduction to training a simple policy in Isaac Lab

1.  Explore the reinforcement learning examples provided with Isaac Sim.
2.  Follow one of the tutorials to train a simple policy, for example, for a robot to reach a target.
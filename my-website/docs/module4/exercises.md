---
sidebar_position: 2
---

# Module 4 Exercises

These exercises are designed to accompany Module 4 of the "Physical AI & Humanoid Robotics" textbook.

## Exercise 1: Using an LLM API to generate robot commands

1.  Sign up for an API key from an LLM provider (e.g., OpenAI).
2.  Write a Python script that takes a natural language command as input and uses the LLM API to generate a sequence of robot commands.
3.  For example, if the input is "go to the kitchen", the output could be `["go_to('kitchen')"]`.

## Exercise 2: Building a simple text-based interface to control a simulated robot

1.  Create a new ROS 2 Python node that takes text input from the user.
2.  In this node, use the script from Exercise 1 to convert the text input into a robot command.
3.  Publish the robot command to a ROS 2 topic.
4.  Create another ROS 2 node that subscribes to the topic and controls a simulated robot in Gazebo or Isaac Sim.

## Exercise 3: Integrating a simple VLA model

1.  Find a pre-trained VLA model that is suitable for a simple robotics task, such as pick-and-place.
2.  Write a ROS 2 node that uses the VLA model to generate a robot command from an image and a text prompt.
3.  For example, you could give the model an image of a table with several objects on it and the prompt "pick up the red ball".

## Exercise 4: Exploring the limitations and biases of LLMs in a robotics context

1.  Design an experiment to test the limitations of an LLM for a robotics task.
2.  For example, you could try giving the LLM ambiguous or nonsensical commands and see how it responds.
3.  Write a short report on your findings, including a discussion of the potential biases of LLMs in robotics.
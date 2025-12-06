---
sidebar_position: 2
---

# Module 2 Exercises

These exercises are designed to accompany Module 2 of the "Physical AI & Humanoid Robotics" textbook.

## Exercise 1: Creating a simple Gazebo world

1.  Create a new file named `my_world.world` in a new directory `my_gazebo_worlds`.
2.  In this file, write a simple Gazebo world with a ground plane, a sun, and a simple building made of boxes.

## Exercise 2: Spawning a URDF model in Gazebo

1.  Use the `ros2 launch` command to launch your `my_world.world` file in Gazebo.
2.  Write a launch file to spawn your URDF model from Module 1 into the Gazebo world.

## Exercise 3: Controlling the simulated robot with ROS 2 messages

1.  Create a new ROS 2 Python node that publishes `geometry_msgs/Twist` messages to the `/cmd_vel` topic.
2.  Run your node and see the robot move in Gazebo. You will need to add a diff drive plugin to your URDF for this to work.

## Exercise 4: Visualizing sensor data from the simulated robot in RViz2

1.  Add a camera sensor to your robot's URDF model.
2.  Use the `ros_gz_bridge` to bridge the camera data from Gazebo to a ROS 2 topic.
3.  Visualize the camera data in RViz2.

## Exercise 5: Setting up a Unity project to visualize the robot simulation

1.  Follow the instructions in `robot-code/unity-setups/README.md` to set up a new Unity project.
2.  Connect your Unity project to your ROS 2 network.
3.  Visualize your robot from the Gazebo simulation in Unity.
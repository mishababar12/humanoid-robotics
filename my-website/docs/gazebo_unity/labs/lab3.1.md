# Lab 3.1: Simulating a Mobile Robot and Streaming Sensor Data in Gazebo

This lab focuses on setting up a Gazebo simulation for a mobile robot and configuring it to stream various sensor data to ROS 2 topics. This is a crucial step for developing and testing robot perception and navigation algorithms.

## Objectives
*   Launch a pre-configured Gazebo simulation environment with a mobile robot.
*   Verify the existence of key ROS 2 topics for control and sensor data.
*   Record and analyze simulated sensor data (e.g., laser scan, odometry, camera feed).
*   Understand the basic structure of a robot model in Gazebo and how sensors are integrated.

## Prerequisites
*   A functional ROS 2 Humble/Iron installation.
*   A ROS 2 workspace set up (e.g., `~/ros2_ws`).
*   Basic understanding of Gazebo environments (from previous section).
*   **A simulated robot environment with sensors:** We will use the TurtleBot3 simulation in Gazebo as it comes pre-configured with a differential drive robot, a 2D laser scanner, a camera, and publishes odometry.
    ```bash
    sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
    ```

## Instructions

### Step 1: Launch the TurtleBot3 Gazebo Simulation

Open a terminal and launch the `turtlebot3_world` simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This command will open the Gazebo simulator with the TurtleBot3 in a simple world. You should see the robot and potentially some obstacles.

### Step 2: Inspect ROS 2 Topics

While the simulation is running, open a new terminal and source your ROS 2 environment. Then, use `ros2 topic list` to see all active topics:

```bash
source install/setup.bash # or your ROS 2 setup file
ros2 topic list
```
You should see topics like:
*   `/cmd_vel` (for sending velocity commands to the robot)
*   `/odom` (odometry data)
*   `/scan` (laser scan data)
*   `/camera/image_raw` (raw camera image)
*   `/camera/depth/image_raw` (raw depth image, if configured)
*   `/tf` and `/tf_static` (robot transformations)

### Step 3: Analyze Sensor Data

Use `ros2 topic echo` to inspect the data published on various sensor topics.

*   **Odometry Data:**
    ```bash
    ros2 topic echo /odom
    ```
    You will see `nav_msgs/msg/Odometry` messages, containing the robot's pose (position and orientation) and twist (linear and angular velocities) in the `odom` frame.

*   **Laser Scan Data:**
    ```bash
    ros2 topic echo /scan
    ```
    You will see `sensor_msgs/msg/LaserScan` messages. Pay attention to the `ranges` array, which contains the distance measurements from the laser scanner.

*   **Camera Image Data (Raw):**
    ```bash
    ros2 topic echo /camera/image_raw
    ```
    This will print `sensor_msgs/msg/Image` messages. The output will be very verbose as it's raw pixel data. To visualize images, you typically use a tool like `rqt_image_view`.

    ```bash
    rqt_image_view /camera/image_raw
    ```
    This will open a GUI displaying the camera feed from the simulated robot.

### Step 4: Visualizing the Robot and Sensor Data in RViz2

**RViz2** is a 3D visualization tool for ROS 2. It's invaluable for understanding what your robot is doing and how its sensors perceive the world.

1.  Open a new terminal and launch RViz2:
    ```bash
    rviz2
    ```
2.  **Add Displays:** In the RViz2 interface, click on "Add" in the "Displays" panel (bottom left).
    *   **RobotModel:** Add a `RobotModel` display. Set its `Description Topic` to `/robot_description` (or similar) and its `TF Prefix` if your robot uses one. This will show your robot's 3D model.
    *   **TF:** Add a `TF` display to visualize the coordinate frames.
    *   **LaserScan:** Add a `LaserScan` display and set its `Topic` to `/scan`. You should see the laser scanner's readings as points in the 3D view.
    *   **Image:** Add an `Image` display and set its `Topic` to `/camera/image_raw`. You'll see the camera feed.
    *   **Odometry:** Add an `Odometry` display and set its `Topic` to `/odom`. You'll see an arrow representing the robot's pose and a trail.

3.  **Drive the Robot:** While RViz2 and Gazebo are running, use your `velocity_commander` node from Lab 2.1 or directly publish `Twist` messages to `/cmd_vel` to drive the robot:
    ```bash
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```
    Observe how the robot moves in Gazebo, and how its representation and sensor data update in RViz2.

### Step 5: Recording Data (Optional)

You can record all ROS 2 topics or selected topics using `ros2 bag`. This is useful for replaying simulations and debugging algorithms offline.

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /odom /scan /camera/image_raw
```
Press `Ctrl+C` to stop recording. The data will be saved in a directory named `ros2_bag_DATE_TIME`.

To replay the bag file:
```bash
ros2 bag play ros2_bag_DATE_TIME
```

## Experimentation and Analysis

*   **Sensor Noise:** In Gazebo, you can configure noise properties for simulated sensors. Explore how different noise levels affect the quality of your sensor data.
*   **Environmental Impact:** Place different objects in the Gazebo world and observe how they affect laser scan readings (e.g., reflections, occlusions).
*   **Robot Parameters:** If you were to modify the robot's physical parameters (e.g., mass, friction), how would it affect its motion and odometry?
*   **Visualizing TF:** Pay close attention to the `TF` frames in RViz2. Understand the relationships between `/base_link`, `/odom`, `/camera_link`, `/laser_link`, etc.

This lab provides hands-on experience in generating and analyzing simulated sensor data, a fundamental skill for anyone working with robot perception and autonomous navigation.

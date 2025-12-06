# Lab 2.1: Basic Robot Control in Simulation

This lab will guide you through controlling a simulated robot using ROS 2. You will learn to send commands to make the robot move and receive feedback on its position and orientation, providing a fundamental understanding of mobile robot control.

## Objectives
*   Launch a simulated robot environment (e.g., a simple differential drive robot) in Gazebo.
*   Use ROS 2 topics to send velocity commands (`geometry_msgs/msg/Twist`) to the robot.
*   Monitor the robot's state (`nav_msgs/msg/Odometry`) using ROS 2 topics.
*   Experiment with different velocity commands to achieve desired robot movements.

## Prerequisites
*   A functional ROS 2 Humble/Iron installation.
*   A ROS 2 workspace set up (e.g., `~/ros2_ws`).
*   Basic understanding of ROS 2 nodes, topics, and messages (from the previous sections).
*   **A simulated robot environment:** For this lab, we will assume you have a basic Gazebo simulation running with a differential drive robot that exposes a `/cmd_vel` topic for `geometry_msgs/msg/Twist` commands and publishes `nav_msgs/msg/Odometry` on an `/odom` topic. (Detailed setup of Gazebo environments will be covered in a subsequent module.)

    *Example of a robot that typically works with this setup:* The `turtlebot3_gazebo` package provides such an environment. You can install it with:
    ```bash
    sudo apt install ros-humble-turtlebot3-gazebo
    ```
    And launch a simulation (e.g., `turtlebot3_world.launch.py`):
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

## Instructions

### Step 1: Create a ROS 2 Python Package for the Lab

Navigate to your ROS 2 workspace `src` directory (e.g., `~/ros2_ws/src`) and create a new package named `robot_control_lab`.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_control_lab --dependencies rclpy geometry_msgs nav_msgs
```
Here, `geometry_msgs` is for `Twist` messages, and `nav_msgs` is for `Odometry` messages.

### Step 2: Implement the Velocity Commander Node

This node will publish `Twist` messages to control the robot's linear and angular velocity.

Create a new Python file `robot_control_lab/robot_control_lab/velocity_commander.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Send commands every second
        self.get_logger().info('Velocity Commander Node has been started.')

    def timer_callback(self):
        twist_msg = Twist()

        # Example: Move forward for 2 seconds, then turn for 2 seconds
        if self.get_clock().now().nanoseconds / 1e9 % 4 < 2:
            twist_msg.linear.x = 0.2  # Move forward at 0.2 m/s
            twist_msg.angular.z = 0.0
            self.get_logger().info('Moving Forward...')
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.5  # Turn at 0.5 rad/s
            self.get_logger().info('Turning...')

        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    velocity_commander = VelocityCommander()
    rclpy.spin(velocity_commander)
    velocity_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Implement the Odometry Subscriber Node

This node will subscribe to the robot's odometry topic and print its current position and orientation.

Create a new Python file `robot_control_lab/robot_control_lab/odometry_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Odometry Subscriber Node has been started.')

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles for easier understanding (yaw)
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        self.get_logger().info(
            f'Robot Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f} | '
            f'Orientation (Yaw): {yaw:.2f} radians'
        )

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    rclpy.spin(odometry_subscriber)
    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Note:* You might need to install `tf_transformations`: `pip install transforms3d` or `sudo apt install ros-humble-tf-transformations` (for Humble).

### Step 4: Update `setup.py`

Modify `robot_control_lab/setup.py` to add the entry points for your new nodes:

```python
from setuptools import find_packages, setup

package_name = 'robot_control_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com', # Update with your email
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = robot_control_lab.velocity_commander:main',
            'odom_listener = robot_control_lab.odometry_subscriber:main',
        ],
    },
)
```

### Step 5: Build Your Package

Build your workspace from the root of your `ros2_ws`:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_control_lab
```

### Step 6: Run the Lab

1.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```
2.  **Launch the simulated robot (e.g., TurtleBot3):**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
    This will open a Gazebo window with the robot.

3.  **Run the Velocity Commander:** In a new terminal:
    ```bash
    ros2 run robot_control_lab commander
    ```
    You should see the robot moving in Gazebo, alternating between moving forward and turning.

4.  **Run the Odometry Subscriber:** In another new terminal:
    ```bash
    ros2 run robot_control_lab odom_listener
    ```
    You should see the subscriber printing the robot's estimated position and orientation.

## Experimentation and Verification
*   **Modify `velocity_commander.py`:** Change the linear and angular velocities. Make the robot move in a square, a circle, or follow a more complex path.
*   **Observe Odometry:** How does the reported position and orientation change with different commands?
*   **Topic Inspection:** Use `ros2 topic echo /cmd_vel` and `ros2 topic echo /odom` to directly observe the messages being published.
*   **Visualization:** If you have `rviz2` installed, you can launch it (`rviz2`) and add a `RobotModel` and `Odometry` display to visualize the robot's movement and reported pose.

This lab provides a hands-on experience with fundamental robot control using ROS 2, paving the way for more complex navigation and manipulation tasks.

# Exercise 2.1: Simple ROS 2 Publisher and Subscriber with Custom Message

This exercise guides you through creating a basic ROS 2 publisher and subscriber using Python, incorporating a custom message type. This will solidify your understanding of ROS 2 package creation, message definition, and the fundamental publisher-subscriber communication pattern.

## Objectives
*   Create a new ROS 2 Python package.
*   Define a custom ROS 2 message type.
*   Implement a publisher node that sends instances of your custom message.
*   Implement a subscriber node that receives and processes these custom messages.
*   Correctly configure `package.xml` and `setup.py` for a Python package using custom messages.

## Instructions

### Step 1: Create a ROS 2 Python Package

Navigate to your ROS 2 workspace `src` directory (e.g., `~/ros2_ws/src`) and create a new package named `custom_message_py`.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python custom_message_py --dependencies rclpy std_msgs
```

### Step 2: Define a Custom Message Type

Inside your new package, create a directory for messages and define your custom message.

1.  Create a `msg` directory inside `custom_message_py`:
    ```bash
    mkdir custom_message_py/msg
    ```
2.  Create a file named `MyCustomMsg.msg` inside `custom_message_py/msg` with the following content:
    ```
    # custom_message_py/msg/MyCustomMsg.msg
    string header_info
    int32 id
    float32 temperature
    bool is_valid
    ```
    This message will contain a string for general info, an integer ID, a float for temperature, and a boolean flag.

### Step 3: Update `package.xml`

You need to tell ROS 2 that your package defines custom messages and depends on `rosidl_default_generators` and `rosidl_default_runtime`.

Open `custom_message_py/package.xml` and add the following lines within the `<build_depends>` and `<exec_depends>` tags (you might already have `rclpy` and `std_msgs`):

```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

Your `package.xml` should look similar to this (dependencies might vary slightly):

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_message_py</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 4: Update `setup.py`

You need to configure `setup.py` to ensure your custom messages are generated and installed correctly.

Open `custom_message_py/setup.py` and modify it as follows:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_message_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these lines to install message files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = custom_message_py.publisher_member_function:main',
            'subscriber = custom_message_py.subscriber_member_function:main',
        ],
    },
)
```
**Explanation of Changes:**
*   `import os` and `from glob import glob`: Needed for path manipulation and finding `.msg` files.
*   `data_files`: The critical change here is `(os.path.join('share', package_name, 'msg'), glob('msg/*.msg'))`, which tells `ament` to install your custom `.msg` files so they can be found by other packages.
*   `entry_points`: Defines the executable scripts for your publisher and subscriber nodes.

### Step 5: Implement the Publisher Node

Create a new Python file `custom_message_py/custom_message_py/publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node

from custom_message_py.msg import MyCustomMsg # Import your custom message

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        # Create a publisher for your custom message type
        self.publisher_ = self.create_publisher(MyCustomMsg, 'custom_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = MyCustomMsg() # Create an instance of your custom message
        msg.header_info = f'Message from Publisher {self.count}'
        msg.id = self.count
        msg.temperature = 25.5 + (self.count * 0.1) # Example data
        msg.is_valid = True if self.count % 2 == 0 else False

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.header_info}", ID: {msg.id}, Temp: {msg.temperature}°C, Valid: {msg.is_valid}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    custom_publisher = CustomPublisher()
    rclpy.spin(custom_publisher)
    custom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Implement the Subscriber Node

Create a new Python file `custom_message_py/custom_message_py/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node

from custom_message_py.msg import MyCustomMsg # Import your custom message

class CustomSubscriber(Node):
    def __init__(self):
        super().__init__('custom_subscriber')
        # Create a subscriber for your custom message type
        self.subscription = self.create_subscription(
            MyCustomMsg, # Subscribe to your custom message type
            'custom_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: MyCustomMsg):
        self.get_logger().info(f'Received: "{msg.header_info}", ID: {msg.id}, Temp: {msg.temperature}°C, Valid: {msg.is_valid}')

def main(args=None):
    rclpy.init(args=args)
    custom_subscriber = CustomSubscriber()
    rclpy.spin(custom_subscriber)
    custom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 7: Build Your Package

Now, build your workspace from the root of your `ros2_ws`:

```bash
cd ~/ros2_ws
colcon build --packages-select custom_message_py
```
This command will build only your `custom_message_py` package, including generating the necessary Python interfaces for `MyCustomMsg`.

### Step 8: Run the Nodes

Source your workspace and then run the publisher and subscriber in separate terminals:

1.  **Source your workspace:**
    ```bash
    source install/setup.bash
    ```
    (You need to do this in every new terminal you open for ROS 2).

2.  **Run the Publisher:**
    ```bash
    ros2 run custom_message_py publisher
    ```

3.  **Run the Subscriber:**
    ```bash
    ros2 run custom_message_py subscriber
    ```

You should see the publisher node sending custom messages, and the subscriber node receiving and printing the structured data. This demonstrates successful communication using a custom message type in ROS 2.

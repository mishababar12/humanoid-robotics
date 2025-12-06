# Lab 5.1: Translating Natural Language Instructions to Robot Actions with an LLM

This lab demonstrates how to use a Large Language Model (LLM) to interpret natural language instructions and translate them into a sequence of executable ROS 2 actions for a simulated robot. This bridges the gap between high-level human commands and low-level robot control.

## Objectives
*   Understand the workflow of using an LLM for robotic task planning.
*   Integrate with a conversational AI service (e.g., OpenAI API).
*   Develop a ROS 2 node that sends natural language commands to an LLM.
*   Translate LLM responses into ROS 2 `Twist` commands for a simulated robot.

## Prerequisites
*   A functional ROS 2 Humble/Iron installation.
*   A ROS 2 workspace set up (e.g., `~/ros2_ws`).
*   Familiarity with ROS 2 publisher-subscriber concepts.
*   **Access to an LLM API:** You will need an API key for a service like OpenAI (GPT-3.5/GPT-4) or Google Gemini.
*   **A simulated robot:** The TurtleBot3 in Gazebo (`ros-humble-turtlebot3-gazebo`) is recommended.

## Instructions

### Step 1: Set Up Your API Key

Ensure you have an API key for your chosen LLM service (e.g., OpenAI). It's best practice to set this as an environment variable rather than hardcoding it into your script.

```bash
export OPENAI_API_KEY="YOUR_OPENAI_API_KEY"
# or for Google Gemini
export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
```

### Step 2: Create a ROS 2 Python Package

Navigate to your ROS 2 workspace `src` directory and create a new package named `llm_robot_commander`.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python llm_robot_commander --dependencies rclpy geometry_msgs
```
This package will contain our LLM interface and robot commander node.

### Step 3: Implement the LLM Commander Node

This node will:
1.  Subscribe to a text topic for human commands.
2.  Send these commands to the LLM API.
3.  Parse the LLM's response to extract robot actions (e.g., move forward, turn left).
4.  Publish `Twist` messages to the robot's `/cmd_vel` topic.

Create a new Python file `llm_robot_commander/llm_robot_commander/commander_node.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import openai # or google.generativeai for Gemini
import os
import json
import time

class LLMRobotCommander(Node):
    def __init__(self):
        super().__init__('llm_robot_commander')

        # Initialize LLM API
        # For OpenAI
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if not openai.api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set.")
            rclpy.shutdown()
            return
        
        # # For Google Gemini (uncomment and replace if using Gemini)
        # import google.generativeai as genai
        # genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
        # self.model = genai.GenerativeModel('gemini-pro')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command_subscription = self.create_subscription(
            String,
            'human_command',
            self.command_callback,
            10)
        self.get_logger().info('LLM Robot Commander Node has been started.')
        self.current_action_publisher = self.create_publisher(String, 'robot_action_status', 10)


        self.robot_speed_linear = 0.2 # m/s
        self.robot_speed_angular = 0.5 # rad/s

        self.command_history = [] # For conversation context

    def get_llm_response(self, prompt):
        # For OpenAI
        try:
            response = openai.chat.completions.create(
                model="gpt-3.5-turbo", # Or "gpt-4" for more complex tasks
                messages=[
                    {"role": "system", "content": "You are a robot assistant. Translate human commands into simple robot actions: forward, backward, turn_left, turn_right, stop. Respond with only a JSON object like {'action': 'forward', 'duration': 2.0} or {'action': 'stop'}."},
                    *self.command_history, # Include history
                    {"role": "user", "content": prompt}
                ],
                max_tokens=50,
                temperature=0.0
            )
            # Add user and assistant messages to history
            self.command_history.append({"role": "user", "content": prompt})
            self.command_history.append({"role": "assistant", "content": response.choices[0].message.content})

            return response.choices[0].message.content
        except openai.APIError as e:
            self.get_logger().error(f"OpenAI API Error: {e}")
            return None
        
        # # For Google Gemini (uncomment to use)
        # try:
        #     response = self.model.generate_content([
        #         "You are a robot assistant. Translate human commands into simple robot actions: forward, backward, turn_left, turn_right, stop. Respond with only a JSON object like {'action': 'forward', 'duration': 2.0} or {'action': 'stop'}.",
        #         prompt
        #     ])
        #     return response.text
        # except Exception as e:
        #     self.get_logger().error(f"Gemini API Error: {e}")
        #     return None


    def execute_robot_action(self, action_data):
        twist_msg = Twist()
        action = action_data.get('action')
        duration = action_data.get('duration', 0.0) # Default duration for non-stop actions

        status_msg = String()
        status_msg.data = f"Executing: {action}"
        if duration > 0:
            status_msg.data += f" for {duration} seconds"
        self.current_action_publisher.publish(status_msg)


        if action == 'forward':
            twist_msg.linear.x = self.robot_speed_linear
        elif action == 'backward':
            twist_msg.linear.x = -self.robot_speed_linear
        elif action == 'turn_left':
            twist_msg.angular.z = self.robot_speed_angular
        elif action == 'turn_right':
            twist_msg.angular.z = -self.robot_speed_angular
        elif action == 'stop':
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown action: {action}")
            return

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f"Published Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

        if duration > 0:
            time.sleep(duration) # Execute for duration
            # Stop the robot after the duration
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().info(f"Action '{action}' completed. Robot stopped.")
            status_msg.data = f"Action '{action}' completed."
            self.current_action_publisher.publish(status_msg)


    def command_callback(self, msg: String):
        human_command = msg.data
        self.get_logger().info(f"Received human command: '{human_command}'")

        llm_response_text = self.get_llm_response(human_command)
        if llm_response_text:
            self.get_logger().info(f"LLM Raw Response: {llm_response_text}")
            try:
                action_data = json.loads(llm_response_text)
                self.execute_robot_action(action_data)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse LLM response as JSON: {e}. Response: {llm_response_text}")
            except Exception as e:
                self.get_logger().error(f"Error executing robot action: {e}")
        else:
            self.get_logger().warn("LLM did not provide a valid response.")

def main(args=None):
    rclpy.init(args=args)
    llm_robot_commander = LLMRobotCommander()
    rclpy.spin(llm_robot_commander)
    llm_robot_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*   **Important:** This script uses the `openai` library. Install it with `pip install openai`. If using Google Gemini, install `google-generativeai` with `pip install google-generativeai`. You will also need to manually uncomment the Gemini related code and comment out the OpenAI code.
*   **`time.sleep()`:** Using `time.sleep()` in a ROS 2 callback blocks the executor. For real-world applications, you'd integrate action duration into a more sophisticated action server or a state machine. This simple example uses `time.sleep()` for clarity.

### Step 4: Update `setup.py`

Modify `llm_robot_commander/setup.py` to add the entry point for your new node:

```python
from setuptools import find_packages, setup

package_name = 'llm_robot_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'openai'], # Add openai here (or google-generativeai for Gemini)
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = llm_robot_commander.commander_node:main',
        ],
    },
)
```

### Step 5: Build Your Package

Build your workspace from the root of your `ros2_ws`:

```bash
cd ~/ros2_ws
colcon build --packages-select llm_robot_commander
```

### Step 6: Run the Lab

1.  **Launch the simulated robot (e.g., TurtleBot3):**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

2.  **Run the LLM Robot Commander Node:** In a new terminal:
    ```bash
    source install/setup.bash
    ros2 run llm_robot_commander commander_node
    ```
    Ensure your API key is set as an environment variable before running this.

3.  **Send Natural Language Commands:** Open another new terminal and publish commands to the `human_command` topic:

    *   **Move Forward:**
        ```bash
        ros2 topic pub --once /human_command std_msgs/msg/String "data: 'Move forward for 3 seconds'"
        ```
    *   **Turn Left:**
        ```bash
        ros2 topic pub --once /human_command std_msgs/msg/String "data: 'Please turn left for 2 seconds'"
        ```
    *   **Stop:**
        ```bash
        ros2 topic pub --once /human_command std_msgs/msg/String "data: 'Stop the robot'"
        ```
    *   **Complex Command (might require more advanced LLM parsing):**
        ```bash
        ros2 topic pub --once /human_command std_msgs/msg/String "data: 'Go forward a bit, then make a right turn'"
        ```
        (Note: The current LLM prompt is very simple, so complex chained commands might not work well without refining the prompt or adding more logic to the node.)

## Experimentation and Verification

*   **LLM Prompt Engineering:** Experiment with different system prompts for the LLM to guide its output format and behavior. Can you make it respond with more specific actions or handle more complex sequences?
*   **Error Handling:** What happens if the LLM returns an unexpected format? Enhance the JSON parsing in the `commander_node.py`.
*   **Speech Interface:** Integrate the speech-to-text functionality from Exercise 5.2 into this lab to allow voice commands instead of typing them into `ros2 topic pub`.
*   **Advanced Actions:** Extend the `execute_robot_action` function to handle more complex robot behaviors beyond simple `Twist` messages (e.g., calling ROS 2 services or actions for picking, placing, etc.).

This lab provides a powerful demonstration of how conversational AI and LLMs can directly influence robot control, opening up new possibilities for intuitive and flexible human-robot interaction.

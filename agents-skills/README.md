# Agents and Skills

This directory contains reusable intelligence components (subagents and skills) for the "Physical AI & Humanoid Robotics" textbook project.

## Identified Repetitive Tasks

The following tasks have been identified as candidates for automation using Claude Code Subagents and Agent Skills:

*   **Docusaurus Content Generation:** A subagent could be created to generate the boilerplate for new Docusaurus documents, such as modules and exercises.
*   **Code Example Validation:** A skill could be developed to automatically run and validate the code examples in the textbook. This would involve parsing the code from the markdown files, executing it, and checking for errors.
*   **API Contract Generation:** A subagent could be used to generate API client code from an OpenAPI specification. For example, it could generate a Python client for the Better-Auth API.
*   **ROS 2 Package Creation:** A subagent could be created to automate the creation of new ROS 2 packages, including the `package.xml` and `setup.py` files.
*   **Weekly Breakdown Generation:** A subagent could be used to generate the weekly breakdown files based on the module outlines.

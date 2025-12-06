import re
import subprocess
import os

# This is a conceptual example of a skill for validating code examples.
# In a real implementation, this would be a more complex script that
# can handle different programming languages and has better error handling.

def validate_python_code(code):
    """
    Validates a block of Python code by running it.
    """
    try:
        # We can't just exec the code, as it might have side effects.
        # Instead, we'll write it to a temporary file and run it as a subprocess.
        with open("temp_code.py", "w") as f:
            f.write(code)
        result = subprocess.run(["python", "temp_code.py"], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("Code is valid.")
            return True
        else:
            print("Code is invalid:")
            print(result.stderr)
            return False
    except Exception as e:
        print(f"An error occurred while validating the code: {e}")
        return False
    finally:
        # Clean up the temporary file
        if os.path.exists("temp_code.py"):
            os.remove("temp_code.py")


def extract_code_blocks(markdown_file):
    """
    Extracts code blocks from a markdown file.
    """
    with open(markdown_file, "r") as f:
        content = f.read()
    # This regex will find all python code blocks
    code_blocks = re.findall(r"```python\n(.*?)\n```", content, re.DOTALL)
    return code_blocks

if __name__ == '__main__':
    # This is an example of how you might use this skill.
    # You would need to provide a markdown file with code examples.
    # markdown_file = "my-website/docs/module1/exercises.md"
    # code_blocks = extract_code_blocks(markdown_file)
    # for code in code_blocks:
    #     validate_python_code(code)
    print("Conceptual code validator skill.")

import json
import re
import os

def remove_comments(json_like):
    """Remove C-style comments from a JSON-like string."""
    pattern = r'//.*?$|/\*.*?\*/'
    return re.sub(pattern, '', json_like, flags=re.DOTALL | re.MULTILINE)

def remove_empty_lines(text):
    """Remove empty lines from a text string while keeping line breaks."""
    return "\n".join([line for line in text.splitlines() if line.strip()])

def json_to_dict(path_to_json):
    with open(path_to_json, 'r') as file:
        content = file.read()

        content_no_comments = remove_comments(content)
        # Return parsed JSON
        return json.loads(content_no_comments)

# Function to load template and perform substitutions
def process_template(template_path, substitutions):
    # Read the template content
    with open(template_path, 'r') as file:
        content = file.read()
    # Perform substitutions
    for key, value in substitutions.items():
        content = content.replace(key, value)

    content = remove_comments(content)
    content = remove_empty_lines(content)
    
    return content

def generate_file_from_template(template_path, output_path, substitutions):
    """
    Generate a file from a template by performing variable substitutions.
    
    Args:
        template_path (str): Path to the template file.
        output_path (str): Path to the output file.
        substitutions (dict): Dictionary with keys as placeholders and values as substitutions.
    """
    # Process the template
    processed_content = process_template(template_path, substitutions)

    # Ensure the directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Check if the file already exists
    file_exists = os.path.exists(output_path)

    if file_exists:
        os.remove(output_path)

    # Write the processed content to the new file
    with open(output_path, 'w') as file:
        file.write(processed_content)

    # Inform the user about the action taken
    if file_exists:
        print(f"Existing '{os.path.basename(output_path)}' was replaced in the directory.")
    else:
        print(f"'{os.path.basename(output_path)}' was created in the directory.")

    # Change the file mode to read-only
    os.chmod(output_path, 0o444)
    
#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

ros_version="jazzy"
python_virtual_enviroment_name="myenv"

# Get the Python version dynamically
python_version=$(python3 --version | awk '{print $2}' | cut -d. -f1-2)  # Get Python version in X.Y format

# Create a virtual environment named 'myenv'
python3 -m venv $python_virtual_enviroment_name

# Get the absolute path to the current directory
current_dir=$(pwd)

# Activate the virtual environment
source $python_virtual_enviroment_name/bin/activate

# Check if 'requirements.txt' exists before attempting to install
if [ -f "requirements.txt" ]; then
    # Install packages from requirements.txt
    pip install -r requirements.txt
else
    echo "'requirements.txt' not found. Please make sure it exists before running this script."
    exit 1
fi

# Add the virtual environment's site-packages to PYTHONPATH in ~/.bashrc if not already added
bashrc_entry='export PYTHONPATH="/opt/ros/'$ros_version'/lib/python'$python_version'/site-packages:'$current_dir'/'$python_virtual_enviroment_name'/lib/python'$python_version'/site-packages:$PYTHONPATH"'
if ! grep -Fxq "$bashrc_entry" ~/.bashrc; then
    echo "$bashrc_entry" >> ~/.bashrc
    echo "PYTHONPATH updated in ~/.bashrc."
else
    echo "PYTHONPATH entry already exists in ~/.bashrc."
fi

# Inform the user to source ~/.bashrc to apply changes
echo "Setup complete. Please run 'source ~/.bashrc' to apply the changes."

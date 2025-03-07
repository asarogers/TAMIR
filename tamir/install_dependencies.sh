#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

ros_version="jazzy"
python_virtual_enviroment_name="myenv"

# Create a virtual environment named 'myenv'
python3 -m venv $python_virtual_enviroment_name

# Get the absolute path to the current directory
current_dir=$(pwd)

# Activate the virtual environment
source $python_virtual_enviroment_name/bin/activate

# Install packages from requirements.txt
pip install -r requirements.txt

current_dir=$(pwd)

# Add the virtual environment's site-packages to PYTHONPATH in ~/.bashrc if not already added
bashrc_entry='export PYTHONPATH="/opt/ros/'$ros_version'/lib/python3.12/site-packages:'$current_dir'/'$python_virtual_enviroment_name'/lib/python3.12/site-packages:$PYTHONPATH"'
if ! grep -Fxq "$bashrc_entry" ~/.bashrc; then
    echo "unset PYTHONPATH" >> ~/.bashrc
    echo "$bashrc_entry" >> ~/.bashrc
    echo "PYTHONPATH updated in ~/.bashrc."
else
    echo "PYTHONPATH entry already exists in ~/.bashrc."
fi


# Inform the user to source ~/.bashrc to apply changes
echo "Setup complete. Please run 'source ~/.bashrc' to apply the changes."

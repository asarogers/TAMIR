#!/usr/bin/env python3
import subprocess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions
import os

def play_audio(file_path):
    """Play audio."""
    # Get the package share directory path
    tamir = FindPackageShare('tamir').find('tamir')
    # Join paths correctly - remove the list brackets
    full_path = os.path.join(tamir, file_path)
    # Play the audio file
    subprocess.run(['mpg321', full_path])

def main(args=None):
    """Init music node."""
    play_audio('experiment.mp3')

if __name__ == '__main__':
    main()
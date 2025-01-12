#!/bin/bash

# Function to check if a package is already installed
check_and_install() {
  if ! dpkg -l | grep -q "^ii\s\+$1"; then
    echo "Installing $1..."
    sudo apt-get install -y $1
  else
    echo "$1 is already installed."
  fi
}

# Update package lists
echo "Updating package lists..."
sudo apt-get update

# Install apt-get packages
echo "Installing apt-get packages..."
grep "sudo apt-get install" dependencies.txt | sed 's/.*- //' | while read -r pkg; do
  check_and_install "$pkg"
done

# Install Python packages using pip3
echo "Installing Python packages with pip3..."
grep "pip3 install" dependencies.txt | sed 's/.*- //' | while read -r pkg; do
  pip3 install --user "$pkg"
done

echo "All packages have been installed!"

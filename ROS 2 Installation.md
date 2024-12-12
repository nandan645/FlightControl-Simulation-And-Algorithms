In case of Ubuntu 22.04 the `.deb` binaries already exist so we don't need to build or something. https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

It's simple to install from the site, but to even simplify the steps further, I created this script.

![[Assets/ros2installation.sh]]
```
#!/bin/bash

# Check for UTF-8
echo "Checking locale settings..."
locale

# Update package lists and install locales
echo "Updating packages and installing locales..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
echo "Verifying locale settings..."
locale

# Setup Sources
echo "Installing necessary packages..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key with apt
echo "Adding ROS 2 GPG key..."
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "Adding ROS 2 repository to sources list..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update your apt repository caches
echo "Updating repository caches..."
sudo apt update

# Upgrade your system
echo "Upgrading system packages..."
sudo apt upgrade -y

# Install ROS 2 packages
echo "Installing ROS 2 packages..."
sudo apt install -y ros-humble-desktop ros-dev-tools

echo "All tasks completed successfully!"

```

Just make sure before running this that the file is set as executable
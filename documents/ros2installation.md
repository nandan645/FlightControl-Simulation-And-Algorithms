In case of Ubuntu 22.04 the `.deb` binaries already exist so we don't need to build or something. https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

It's simple to install from the site, but to even simplify the steps further, I created this script.

```
#!/bin/bash

# Update package index
sudo apt update

# Check current locale settings
locale

# Install and set up UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale

# Install necessary tools
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update package index and install curl
sudo apt update && sudo apt install -y curl

# Add ROS 2 repository key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update

# Upgrade installed packages
sudo apt upgrade -y

# Install ROS 2 Humble desktop version
sudo apt install -y ros-humble-desktop

# Install ROS development tools
sudo apt install -y ros-dev-tools

# Source ROS setup file (adjust for your shell if not using bash)
if ! grep -q 'source /opt/ros/humble/setup.bash' ~/.bashrc; then
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
fi
source ~/.bashrc

# Instructions for testing
cat << EOF

ROS 2 installation completed. You can now test the installation:

1. In one terminal, source the setup file and run a C++ talker:
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp talker

2. In another terminal, source the setup file and run a Python listener:
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_py listener

You should see messages being published and received, verifying both the C++ and Python APIs.

EOF

```

Just make sure before running this that the file is set as executable

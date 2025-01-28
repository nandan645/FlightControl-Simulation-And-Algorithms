This script automates the setup process for installing essential packages, dependencies, and software required for building and running ORB-SLAM3 with Pangolin visualization on Ubuntu-based systems.

#### Features:
- Installs essential build tools and libraries.
- Configures and builds Pangolin for 3D visualization.
- Downloads, configures, and builds ORB-SLAM3.
- Includes fixes for common issues, such as library path setup and compatibility adjustments.

```
#!/bin/bash

# Install essential packages
echo "Installing essential packages..."
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libglew-dev libboost-all-dev libssl-dev
sudo apt install -y libeigen3-dev
sudo apt-get install -y libepoxy-dev

# Install compilers and Python pip
echo "Installing compilers and Python pip..."
sudo apt install -y g++ gcc
sudo apt install -y python3-pip

# Install Pangolin
echo "Installing Pangolin..."
cd ~/Documents
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake .. -D CMAKE_BUILD_TYPE=Release
make
sudo make install

# Install ORB SLAM3
echo "Installing ORB SLAM3..."
cd ~/Documents
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
sed -i 's/++11/++14/g' CMakeLists.txt

# Edit build script to use 2 cores instead of 4
sed -i 's/j4/j2/g' build.sh
./build.sh

# Fix issue with libpango1.0-0
echo "Fixing issue with libpango1.0-0..."
sudo apt install -y libpango1.0-0
if ! grep -q 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
fi
source ~/.bashrc
```

Please note that the script updates the `LD_LIBRARY_PATH` environment variable by appending `/usr/local/lib` to it. However, depending on your system configuration, the library path might differ.

#### Download test datasets
```
#!/bin/bash

echo "Downloading test datasets..."
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```

#### Run simulation examples

Make sure you are within root dir of ORB_SLAM3 wich can be `~/Documents/ORB_SLAM3` in this case.
```
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo

./Examples/Stereo-Inertial/stereo_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereoi

```
Preview from `./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi`
![Assets/Screenshot from 2025-01-29 00-10-34.png](https://raw.githubusercontent.com/nandan645/FlightControl-Simulation-And-Algorithms/refs/heads/main/assets/Screenshot%20from%202025-01-29%2000-10-34.png)

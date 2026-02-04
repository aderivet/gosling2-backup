#!/bin/bash

# Usage ./install_software_jetson.sh

# Author: Boluwatife Olabiran

# Description: Installs the necessary packages and software needed to run the F1/10 code developed by the RASLAB Autonomous Systems Group at the FAMU-FSU College of Engineering.

# Exit immediately if a command exits with a non-zero status.
set -e

sudo apt-get update && sudo apt-get install openssh-server && echo "Installed SSH."  # ssh
sudo apt update && sudo apt install -y python3-dev python3-pip python3-setuptools python3-venv build-essential git curl nano cmake  && echo "Installation common dependencies"  # common dependencies
sudo pip3 install -U jetson-stats && echo "Installed jtop"  # jtop

# Install VCS tool (if not installing ROS on the host)
curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
sudo apt-get update
sudo apt-get install python3-vcstool

## Install Docker. Comment out this block if installed using the sd card method or if docker is already installed.
## Thanks to Jetsonhacks (https://github.com/jetsonhacks/install-docker and https://jetsonhacks.com/2025/02/24/docker-setup-on-jetpack-6-jetson-orin/)
## ### Begin: Docker installation block
#./install_nvidia_docker.sh
## ### End: Docker installation block

# Configure NVIDIA Container Toolkit.
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml  # Generate CDI Spec for GPU/PVA
sudo nvidia-ctk runtime configure --runtime=docker --cdi.enabled=true
sudo systemctl restart docker
sudo apt-get update
sudo apt-get install software-properties-common
#sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
#sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt update && sudo apt-get install -y pva-allow-2
echo "Installed docker"

## Restart the docker service and add our user to the docker group to use without sudo. Must be done outside this script as it causes the script to exit.
#sudo systemctl restart docker && sudo usermod -aG docker $USER && newgrp docker && echo "Added user to docker group."

# Add Nvidia to the default runtime
sudo apt install -y jq
sudo jq '. + {"default-runtime": "nvidia"}' /etc/docker/daemon.json | \
  sudo tee /etc/docker/daemon.json.tmp && \
  sudo mv /etc/docker/daemon.json.tmp /etc/docker/daemon.json
echo "Setup Nvidia as default-runtime"

sudo systemctl daemon-reload && sudo systemctl restart docker  # Restart docker

sudo apt update && sudo apt install -y nvidia-jetpack && echo "Installed nvidia-jetpack" # install nvidia jetpack (optional)

sudo nvpmodel -m 2  && echo "Set MAXN super as NVP power mode."

sudo systemctl disable nvzramconfig && echo "Disabled zram."

sudo fallocate -l 16G /mnt/16GB.swap
sudo chmod 600 /mnt/16GB.swap
sudo mkswap /mnt/16GB.swap
sudo swapon /mnt/16GB.swap
sudo cp /etc/fstab /etc/fstab.bak
echo '/mnt/16GB.swap none swap sw 0 0' | sudo tee -a /etc/fstab
echo "Allocated 16GB Swap memory."

sudo systemctl disable nvargus-daemon.service  && echo "Disabled misc services"

#sudo systemctl set-default multi-user.target  && echo "Disabled desktop GUI"  # Disable desktop GUI. `sudo systemctl set-default graphical.target` to reenable.

sudo apt-get install -y joystick jstest-gtk  && echo "Installed joystick support"

cd /tmp && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble
chmod 0777 ydlidar_ros2_driver/startup/*
sudo sh ydlidar_ros2_driver/startup/initenv.sh
echo "Setup YDLIDAR UDEV rules"

sudo apt install -y v4l-utils
cd /tmp && git clone https://github.com/IntelRealSense/librealsense.git && cd librealsense
sudo sh scripts/setup_udev_rules.sh
echo "Setup Intel RealSense UDEV rules"

echo 'KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' | sudo tee -a /etc/udev/rules.d/99-vesc.rules
echo "Setup VESC UDEV rules"

echo 'KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"' | sudo tee -a /etc/udev/rules.d/99-joypad-f710.rules
echo "Setup Logitech F710 Joystick UDEV rules"

sudo udevadm control --reload-rules && sudo udevadm trigger && echo "Reloaded UDEV rules"  # reload UDEV rules

# Install jetson-containers
cd ~/Downloads && git clone https://github.com/dusty-nv/jetson-containers && bash jetson-containers/install.sh

# (optional) Add Logitech F710 support
sudo apt-get install -y dkms
#sudo dkms remove -m xpad -v 0.4 --all
sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4 && sudo dkms install -m xpad -v 0.4

echo "Installation complete"
exit 0
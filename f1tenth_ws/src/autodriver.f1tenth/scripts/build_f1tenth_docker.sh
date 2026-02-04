#!/bin/bash

# Usage ./build_f1tenth_docker.sh

# Author: Boluwatife Olabiran

# Description: Sets up the docker container needed to build and run the F1/10 code.
# This build uses NVIDIA's Isaac container. See https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html

# Exit immediately if a command exits with a non-zero status.
set -e

# Install Git LFS
sudo apt-get install git-lfs -y
git lfs install --skip-repo

# Create the ROS2 workspace
mkdir -p ~/f1tenth_ws/src
echo "export ISAAC_ROS_WS=$HOME/f1tenth_ws" >> ~/.bashrc && echo "export ROS_WS=$HOME/f1tenth_ws" >> ~/.bashrc && echo "export ROS2_WS=$HOME/f1tenth_ws" >> ~/.bashrc && source ~/.bashrc && export ISAAC_ROS_WS=$HOME/f1tenth_ws

cd ${ISAAC_ROS_WS}/src
git clone -b release-3.2 https://github.com/privvyledge/isaac_ros_common.git isaac_ros_common
#cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && touch .isaac_ros_common-config && echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config  # about 1 hour download (with ethernet) + realsense build
#realsense-viewer  # Check if the realsense camera is detected. In docker
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && touch .isaac_ros_common-config && echo -e "CONFIG_IMAGE_KEY=ros2_humble.realsense.f1tenth\nCONFIG_DOCKER_SEARCH_DIRS=(../../autodriver.f1tenth/docker ../docker)" > .isaac_ros_common-config  # 70 minutes total

# Pull the repositories
cd ${ISAAC_ROS_WS} && vcs import --recursive --workers 6 src < src/autodriver.f1tenth/f1tenth.repos

mkdir -p ${HOME}/shared_dir ${HOME}/data
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
echo -e "-v /dev:/dev\n-v ${HOME}/shared_dir:/mnt/shared_dir\n-v ${HOME}/data:/mnt/data" > .isaac_ros_dev-dockerargs

# Run the container
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/ && ./scripts/run_dev.sh -d $ISAAC_ROS_WS  --docker_arg "-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e VEHICLE_NAME=${VEHICLE_NAME} --env QT_X11_NO_MITSHM=1"

# run_dev.sh (https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html#run-dev-sh)
## Run the container without rebuilding, e.g Offline usage
#cd ${ISAAC_ROS_WS}/src/isaac_ros_common/ && ./scripts/run_dev.sh -d $ISAAC_ROS_WS --skip_image_build

## Add docker arguments
#cd ${ISAAC_ROS_WS}/src/isaac_ros_common/ && ./scripts/run_dev.sh --docker_arg "-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e VEHICLE_NAME=${VEHICLE_NAME} --env QT_X11_NO_MITSHM=1"

##### Deployment
#### Method1: for more detailed deployment. Use the deploy_docker_image.sh script
#mv "${ISAAC_ROS_WS}"/src/isaac_ros_common/scripts/.isaac_ros_common-config "${ISAAC_ROS_WS}"/src/isaac_ros_common/scripts/.isaac_ros_common-config.bak
#"${ISAAC_ROS_WS}"/src/autodriver.f1tenth/scripts/deploy_docker_image.sh \
#  --ws-src "${ISAAC_ROS_WS}"/src \
#  --isaac-ros-common "${ISAAC_ROS_WS}"/src/isaac_ros_common \
#  --docker-tags "ros2_humble.realsense.f1tenth.deploy_custom" \
#  --image-name "privvyledge/f1tenth/humble:latest"
#mv "${ISAAC_ROS_WS}"/src/isaac_ros_common/scripts/.isaac_ros_common-config.bak "${ISAAC_ROS_WS}"/src/isaac_ros_common/scripts/.isaac_ros_common-config

### Method 2: Using Nvidia Isaac ROS. Not recommended as it does not copy environment variables
# To add more options to the deployment script (https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html#api-docker-deploy-sh)
#${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/docker_deploy.sh \
#   --base_image_key "aarch64.ros2_humble.realsense.f1tenth" \
#   --custom_apt_source "deb https://mycool-apt-get-server.com/ros-debian-local focal main" \
#   --install_debians "mydebian01,libnvvpi3,tensorrt" \
#   --include_dir /workspaces/isaac_ros-dev/tests \
#   --include_dir /home/admin/scripts:/home/admin/scripts \
#   --ros_ws ${ISAAC_ROS_WS}  \
#   --launch_package "f1tenth_launch" \
#   --launch_file "isaac_ros_image_flip.launch.py" \
#   --name "privvyledge/f1tenth:humble-latest"

# Deploy the docker images into one ready t transfer/deploy image. Colcon packages must be build without --symlink-install
${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/docker_deploy.sh \
   --base_image_key "aarch64.ros2_humble.realsense.f1tenth" \
   --ros_ws ${ISAAC_ROS_WS}  \
   --name "privvyledge/f1tenth:humble-latest"

## Run using Jetson containers
#xhost +local:root && \
#  jetson-containers run -v /dev:/dev -v ${HOME}/shared_dir:/mnt/shared_dir \
#  -v ${HOME}/data:/mnt/data --ipc host --env="QT_X11_NO_MITSHM=1" --privileged \
#  privvyledge/f1tenth:humble-latest /bin/bash && \
#  xhost -local:root

## Or Run without Jetson containers
#xhost +local:root && \
#  docker run --rm -it --privileged --network host --ipc=host \
#  -v /dev/*:/dev/* --gpus all --runtime nvidia \
#  -v ${HOME}/shared_dir:/mnt/shared_dir -v ${HOME}/data:/mnt/data \
#  -v /tmp/.X11-unix:/tmp/.X11-unix \
#  -e DISPLAY=$DISPLAY \
#  -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
#  --env="QT_X11_NO_MITSHM=1" \
#  privvyledge/f1tenth:humble-latest /bin/bash && \
#  xhost -local:root


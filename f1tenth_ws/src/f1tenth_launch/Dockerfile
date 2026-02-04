# todo: fix Realsense issues: color image, gyro, accel, point clouds, GPU [done]
# todo: switch to a dusty_nv container https://github.com/dusty-nv/jetson-containers/blob/master/packages/ros/Dockerfile.ros2 or Nvidia ISAAC ROS or Mine
# todo: setup NVIDIA ISAAC NVBLOX (mapping) and map localizer
# Todo: setup Nvidia ISAAC ROS vSLAM
# todo: setup particle filter
# todo: install Autoware dev tools manually since we're not using their container or modify setup-dev-env and ansible scripts
# pull base image
# Autoware: https://github.com/autowarefoundation/autoware/pkgs/container/autoware-universe/versions?filters%5Bversion_type%5D=tagged
# or OSRF ROS2 or Dusty-NV
#FROM ghcr.io/autowarefoundation/autoware-openadk:latest-humble-devel-cuda
FROM privvyledge/r36.2.0-ros-humble-ml:latest

# Set up the shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York

# Setup user
ARG USER=autoware
ARG USERNAME=${USER}
ENV USERNAME ${USERNAME}

ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo,video $USERNAME && \
        usermod  --uid $USER_UID $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME

# Setup env and shell
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV TZ=America/New_York

ARG ROS_VERSION="ROS2"
ARG ROS_DISTRO="humble"
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


ENV NVIDIA_DRIVER_CAPABILITIES all
# todo: test with newer images
ENV NVIDIA_VISIBLE_DEVICES all  # causes graphical failures

# Install Sudo
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y sudo

# Install packages
RUN sudo apt-get update -y && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    wget \
    less \
    zstd \
    udev \
    unzip \
    build-essential \
    apt-transport-https \
    openssh-server libv4l-0 libv4l-dev v4l-utils binutils xz-utils bzip2 lbzip2 \
    ca-certificates libegl1 \
    lsb-release \
    gnupg2 \
    cmake \
    pkg-config \
    swig \
    g++ \
    libpython3-dev \
    python3-dev \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-rosdep \
    python3-matplotlib \
#    python3-opencv \
    python3-pil \
    python3-yaml \
    python3-tk \
    python3-pyqt5 \
#    libopencv-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    qtcreator && \
    sudo rm -rf /var/lib/apt/lists/*

# Install Python Packages
RUN python3 -m pip install do-mpc casadi

# Initialize ROS workspace
ENV BUILD_HOME=/f1tenth_ws
ARG BUILD_HOME=$BUILD_HOME

RUN mkdir -p "$BUILD_HOME/src"

#################################################### (Optional) Setup ROS2
WORKDIR $BUILD_HOME/src

#################################################### Setup TF2 and Geometry2
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-tf2-geometry-msgs && \
    python3 -m pip install transforms3d numpy && \
    sudo rm -rf /var/lib/apt/lists/*

#################################################### (Optional) Setup F1tenth. Todo: setup IMU fix on humble branch
RUN cd "$BUILD_HOME/src" && rm -rf f1tenth_system && git clone https://github.com/privvyledge/f1tenth_system.git -b foxy-devel && \
    cd f1tenth_system && git submodule update --init --force --remote && \
    cd vesc && git checkout ros2_motor_direction_fix
#################################################### (Optional) Setup VESC
#################################################### (Optional) Setup Autoware

#################################################### (Optional) Setup ROS Nav
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN  sudo apt remove ros-humble-navigation2 ros-humble-nav2* && sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
#    cd "$BUILD_HOME/src" && git clone --recursive https://github.com/ros-planning/navigation2.git -b ${ROS_DISTRO} && \
#    sudo rm -rf /var/lib/apt/lists/*

#################################################### (Optional) Setup SLAM toolbox. Use galactic and above (or noetic) to get pose
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-slam-toolbox && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/SteveMacenski/slam_toolbox.git -b ${ROS_DISTRO}-devel && \
#    cd slam_toolbox && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### (Optional) Setup Robot localization
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-robot-localization && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN cd "$BUILD_HOME/src" && git clone https://github.com/cra-ros-pkg/robot_localization.git -b ${ROS_DISTRO}-devel && \
#    cd robot_localization && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### (Optional) Setup IMU Filters
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-imu-tools && \
    sudo rm -rf /var/lib/apt/lists/*

#################################################### Setup Autonomous bringup packages.
RUN cd "$BUILD_HOME/src" && git clone https://github.com/privvyledge/f1tenth_launch.git -b ${ROS_DISTRO}-dev && \
    git clone https://github.com/privvyledge/trajectory_following_ros2.git && \
    git clone https://github.com/privvyledge/f1tenth_autoware_launch_py.git

#################################################### Setup Laser filters/pipeline.
#RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-laser-pipeline ros-${ROS_DISTRO}-laser-filters  && \
#    rm -rf /var/lib/apt/lists/*
RUN cd "$BUILD_HOME/src" && sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-filters && \
    git clone https://github.com/ros-perception/laser_filters.git -b ros2

#################################################### Setup depth image to laser scan. No need as its already in the container
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-depthimage-to-laserscan  && \
    sudo rm -rf /var/lib/apt/lists/*
# RUN #cd "$BUILD_HOME/src" && git clone https://github.com/ros-perception/depthimage_to_laserscan.git -b ros2 && \
#    cd laser_filters && rosdep install -q -y -r --from-paths src --ignore-src

#################################################### Setup laser odometry packages. Todo: setup for humble
#RUN cd "$BUILD_HOME/src" && git clone https://github.com/Adlink-ROS/rf2o_laser_odometry.git && \
#    git clone https://github.com/AlexKaravaev/csm && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git

#################################################### Setup rf2o laser odometry
RUN cd "$BUILD_HOME/src" && git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git

#################################################### Setup Image Proc (e.g PointClouds from depth or stereo images)
RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-image-pipeline && \
    sudo rm -rf /var/lib/apt/lists/*

#-------------------------------------------------
# Setup MicroROS (https://github.com/micro-ROS/micro_ros_setup.git | https://micro.ros.org/docs/tutorials/core/first_application_linux/)
#-------------------------------------------------
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup && \
    python3 -m pip install pyserial

#-------------------------------------------------
# Setup Autoware.
#-------------------------------------------------
ARG AUTOWARE_DIR='src'
ARG AUTOWARE_FOLDER_NAME='autoware_f1tenth'
RUN git clone https://github.com/privvyledge/autoware.f1tenth.git && cd autoware.f1tenth && mkdir src  && \
    vcs import src < autoware.repos && \
    chmod +x install_autoware_dependencies.sh && ./install_autoware_dependencies.sh

#### Setup Tensorrt YOLOX (https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts). Note should be done automatically but just in case
RUN mkdir -p ~/autoware_data/tensorrt_yolox/ && wget -P ~/autoware_data/tensorrt_yolox/ \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.onnx \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt

WORKDIR /sdks

# Install Acados.
#ARCHITECTURES: "" (Recommended), ARMV8A_ARM_CORTEX_A57-TX2, ARMV8A_ARM_CORTEX_A76-ORIN, X64_AUTOMATIC, GENERIC
# Could remove the -DBLASFEO_TARGET specification and should be automatically detected
ARG TX2_ARCHITECTURE=ARMV8A_ARM_CORTEX_A57
ARG ORIN_ARCHITECTURE=ARMV8A_ARM_CORTEX_A76
ARG ACADO_BLASFEO_TARGET_CPU_ARCHITECHTURE=${ORIN_ARCHITECTURE}
ARG ACADOS_OPENMP_PARALLELIZATION_ENABLED=ON
ARG ACADOS_NUM_THREADS=2
RUN mkdir -p "/sdks/" && cd "/sdks/" && \
    export ACADOS_ROOT='/sdks/acados' && export ACADOS_PATH=${ACADOS_ROOT} && export ACADOS_SOURCE_DIR=${ACADOS_ROOT} && \
    git clone https://github.com/acados/acados.git && cd acados && \
    git submodule update --recursive --init && \
    mkdir build && cd build && \
    cmake \
        -DACADOS_WITH_QPOASES=ON \
        -DACADOS_WITH_OSQP=ON \
        -DACADOS_INSTALL_DIR=${ACADOS_ROOT} \
#        -DBLASFEO_TARGET=${ACADO_BLASFEO_TARGET_CPU_ARCHITECHTURE} \
        -DCMAKE_BUILD_TYPE=release \
        -DACADOS_NUM_THREADS=${ACADOS_NUM_THREADS} \
        -DACADOS_WITH_OPENMP=${ACADOS_OPENMP_PARALLELIZATION_ENABLED} .. && \
    sudo make install -j$(nproc) && \
    python3 -m pip install -e ${ACADOS_ROOT}/interfaces/acados_template && \
#    curl https://sh.rustup.rs -sSf | sh -s -- -y && source $HOME/.cargo/env && \
    sudo apt update && sudo apt install -y rustc cargo && cd ../bin && \
    git clone https://github.com/acados/tera_renderer.git && cd tera_renderer && $HOME/.cargo/bin/cargo build --verbose --release && \
    cp target/release/t_renderer ${ACADOS_ROOT}/bin

#################################################### Setup YDLidar
RUN mkdir -p "/sdks/YDLIDAR" && cd "/sdks/YDLIDAR" && git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    cd YDLidar-SDK && mkdir build && cd build && cmake .. && make && sudo make install

RUN cd "$BUILD_HOME/src" && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b ${ROS_DISTRO}
#RUN cd $BUILD_HOME && \
#    chmod 0777 src/ydlidar_ros2_driver/startup/* && sudo sh src/ydlidar_ros2_driver/startup/initenv.sh

#################################################### Setup Realsense ROS
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-librealsense2* \
#    ros-${ROS_DISTRO}-realsense2-* && \
#    sudo rm -rf /var/lib/apt/lists/*

# todo: set envs above instead of exporting
# branches R/2542, master, development, etc
ARG LIBREALSENSE_BRANCH="master"
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib && \
    export CUDACXX=/usr/local/cuda/bin/nvcc && export PATH=${PATH}:/usr/local/cuda/bin && \
    cd /sdks && git clone --branch ${LIBREALSENSE_BRANCH} --depth=1 https://github.com/IntelRealSense/librealsense && \
    cd librealsense && \
    mkdir build && \
    cd build && \
    cmake \
       -DBUILD_EXAMPLES=true \
	   -DFORCE_RSUSB_BACKEND=true \
	   -DBUILD_WITH_CUDA=true \
	   -DCMAKE_BUILD_TYPE=release \
	   -DBUILD_PYTHON_BINDINGS=bool:true \
	   -DPYTHON_EXECUTABLE=/usr/bin/python3 \
       -DBUILD_GRAPHICAL_EXAMPLES=true \
	   -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') \
	   ../ && \
    make -j$(($(nproc)-1)) && \
    sudo make install
#    cd ../ && \
#    sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/

#ARG LIBREALSENSE_VERSION="2.54.2"
#RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib && \
#    export CUDACXX=/usr/local/cuda/bin/nvcc && export PATH=${PATH}:/usr/local/cuda/bin && \
#    cd /sdks && wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${LIBREALSENSE_VERSION}.zip && \
#    unzip "v${LIBREALSENSE_VERSION}.zip" && rm "v${LIBREALSENSE_VERSION}.zip" && \
#    mv librealsense-${LIBREALSENSE_VERSION} librealsense && cd librealsense && mkdir build && cd build && \
#    cmake \
#       -DBUILD_EXAMPLES=true \
#	   -DFORCE_RSUSB_BACKEND=true \
#	   -DBUILD_WITH_CUDA=true \
#	   -DCMAKE_BUILD_TYPE=release \
#	   -DBUILD_PYTHON_BINDINGS=bool:true \
#	   -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#       -DBUILD_GRAPHICAL_EXAMPLES=true \
#	   -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")') \
#	   ../ && \
#    make -j$(($(nproc)-1)) && \
#    sudo make install &&  \
#    cd ../ && \
#    sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/

# Setup UDEV Rules. Disconnect all cameras. Todo: Might need to setup udev rules on host instead of docker.
# Todo: test setting up udev rules as root (https://forums.docker.com/t/udevadm-control-reload-rules/135564)

# Install realsense ros
# RUN sudo apt install -y --no-install-recommends ros-${ROS_DISTRO}-realsense2-*
#ARG REALSENSE_ROS_VERSION=4.54.1
#RUN cd ${BUILD_HOME}/src && wget https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/${REALSENSE_ROS_VERSION}.zip && \
#    unzip ${REALSENSE_ROS_VERSION}.zip && \
#    mv realsense-ros-${REALSENSE_ROS_VERSION}/ realsense-ros && \
#    rm ${REALSENSE_ROS_VERSION}.zip

RUN cd ${BUILD_HOME}/src && git clone https://github.com/IntelRealSense/realsense-ros.git

#################################################### Setup RTAB-Map (which also publishes odometry from laser_scan)
RUN cd "$BUILD_HOME/src" && git clone https://github.com/introlab/rtabmap.git && git clone https://github.com/introlab/rtabmap_ros.git -b ${ROS_DISTRO}-devel
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    ros-${ROS_DISTRO}-rtabmap* && \
#    sudo rm -rf /var/lib/apt/lists/*

##--------------------------------
## Setup NVIDIA Isaac packages
##--------------------------------
#RUN sudo apt-get update && DEBIAN_FRONTEND="noninteractive" sudo apt-get install -y --no-install-recommends \
#    python3-distutils \
#    libboost-all-dev \
#    libboost-dev \
#    libpcl-dev \
#    libode-dev \
#    lcov \
#    python3-zmq \
#    libxaw7-dev \
#    libgraphicsmagick++1-dev \
#    graphicsmagick-libmagick-dev-compat \
#    libceres-dev \
#    libsuitesparse-dev \
#    libncurses5-dev \
#    libassimp-dev \
#    libyaml-cpp-dev \
#    libpcap-dev \
#    && sudo rm -rf /var/lib/apt/lists/* \
#    && sudo apt-get clean
#
## Add Isaac apt repository
#RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
#    grep -qxF 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' /etc/apt/sources.list || \
#    echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | tee -a /etc/apt/sources.list
#
## Add Nvidia Isaac dependencies to Rosdep
#RUN curl -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml https://isaac.download.nvidia.com/isaac-ros/extra_rosdeps.yaml \
#    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list
#
#RUN cd ${BUILD_HOME}/src && git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

#--------------------------------
# Build ROS workspace
# The '--event-handlers console_direct+ --base-paths',  ' -DCMAKE_LIBRARY_PATH' and ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' flags are needed by ZED
# The ' -DCMAKE_BUILD_TYPE=Release' flag is for all of them, especially Autoware
#--------------------------------
# Silence setup.py and easy_install deprecation warnings caused by colcon and setuptools (https://github.com/colcon/colcon-core/issues/454#issuecomment-1142649390)
ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources

WORKDIR $BUILD_HOME
#ARG MAKEFLAGS="-j4 -l4"
RUN sudo apt update && \
    sudo rosdep init && rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "rf2o_laser_odometry librealsense2 realsense2_camera libopencv-dev" && \
    colcon build --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args '  -Wno-dev' ' --no-warn-unused-cli' ' -DBUILD_ACCELERATE_GPU_WITH_GLSL=ON' ' -DCMAKE_BUILD_TYPE=Release' ' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON' ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"  -DDOWNLOAD_ARTIFACTS=ON'

#-----------------------------
# Setup microros agent, i.e create_agent_ws, then build_agent. Modified to skip rosdep keys and specify colcon build arguments
#-----------------------------
RUN bash -c 'source install/setup.bash; \
             EXTERNAL_SKIP="lio_sam fast_lio rf2o_laser_odometry librealsense2 realsense2_camera libopencv-dev"; \
             ros2 run micro_ros_setup create_agent_ws.sh; \
             ros2 run micro_ros_setup build_agent.sh'

#--------------------------------
# Todo: run the following. See (https://autowarefoundation.github.io/autoware-documentation/release-v1.0_beta/how-to-guides/others/advanced-usage-of-colcon/#changing-the-default-configuration-of-colcon)
#--------------------------------


#-----------------------------
# Setup environment variables
#-----------------------------
# Todo: use ENV to modify PATHs, e.g PATH, PYTHONPATH, LD_LIBRARY_PATH
# Todo: add autoware environment variables like vehicle_id
RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> ~/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc && \
    echo "source ${BUILD_HOME}/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export CYCLONEDDS_URI=file://$BUILD_HOME/src/autoware.f1tenth/cyclone_dds/cyclonedds_config.xml" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/targets/aarch64-linux/lib/stubs:/opt/ros/${ROS_DISTRO}/install/lib' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/aarch64-linux-gnu/tegra' >> ~/.bashrc && \
    echo 'export PATH=${PATH}:/usr/local/cuda/bin' >> ~/.bashrc && \
    echo 'CUDACXX=/usr/local/cuda/bin/nvcc' >> ~/.bashrc && \
    echo 'export ACADOS_ROOT=/sdks/acados' >> ~/.bashrc && \
    echo 'export ACADOS_PATH=${ACADOS_ROOT}' >> ~/.bashrc && \
    echo 'export ACADOS_SOURCE_DIR=${ACADOS_ROOT}' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ACADOS_ROOT}/lib' >> ~/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc && \
    echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"' >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc && \
    echo "export _colcon_cd_root=${ROS_ROOT}" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

## RUN ros2 doctor # run this if the LIDAR doesn't run (https://github.com/YDLIDAR/ydlidar_ros2_driver/issues/10)

## Todo: remove the lines below
RUN sudo apt update && sudo apt install gedit cheese nautilus net-tools iputils-ping -y
#!/usr/bin/env bash

# Setup Display forwarding
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"

# create workspace on host, in home directory
mkdir -p $HOME/f1tenth_ws

# give docker permission to use X
# sudo xhost +si:localuser:root

# run container with privilege mode, host network, display, and mount workspace on host
docker run -it --runtime nvidia --privileged --network host \
    $VOLUMES \
    -e DISPLAY=$DISPLAY \
    --env="XAUTHORITY=${XAUTH}" \
    -v /dev:/dev \
    --mount type=volume,dst=/f1tenth_ws,volume-driver=local,volume-opt=type=none,volume-opt=o=bind,volume-opt=device=$HOME/f1tenth_ws \
    privvyledge/f1tenth:foxy-dev
#!/usr/bin/env bash
xhost +local:docker

if [ -z "$1" ]; then
  echo "Error: <graphics> argument required."
  echo "Usage: ./run.sh <graphics>"
  echo "where <graphics> can be nvidia or intel."
  exit 1
fi

TAG="orca4"
GRAPHICS=$1

DOCKER_FLAGS="-it --rm \
    --name orca4 \
    -v /etc/localtime:/etc/localtime:ro \
    -v ..:/home/orca4/ros2_ws/src/orca4 \
    --privileged" 

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Flags for GUI applications
DOCKER_FLAGS+=" \
  -e DISPLAY\
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v $XAUTH:$XAUTH \
  -v /tmp/.X11-unix:/tmp/.X11-unix"


if [ "$GRAPHICS" == "nvidia" ]; then
  echo "Adding NVIDIA flags..."
  # For NVIDIA graphics. Install the NVIDIA Container Toolkit on the host:
  # https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
  DOCKER_FLAGS+=" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /dev/input:/dev/input \
    --security-opt seccomp=unconfined \
    --gpus all"
elif [ "$GRAPHICS" == "intel" ]; then
  echo "Adding Intel flags..."
  # For Intel graphics
  DOCKER_FLAGS+=" \
    -v /dev/dri:/dev/dri \
    --device /dev/dri \
    --device /dev/dri/card1:/dev/dri/card1 \
    --device /dev/dri/card2:/dev/dri/card2 \
    --device /dev/dri/renderD128:/dev/dri/renderD128 \
    --device /dev/dri/renderD129:/dev/dri/renderD129"
else
  echo "Error: unsupported graphics option '$GRAPHICS'. Use 'nvidia' or 'intel'."
  exit 1
fi

echo "Starting container ${TAG}..."
docker run $DOCKER_FLAGS $TAG

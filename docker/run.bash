#!/bin/bash

# Use  ./run.bash [version]

IMAGENAME=ros_grpc

VERSION=latest
if [ ! "$1" == "" ]; then
  VERSION=$1
fi

PLEXI_DIR="$(cd "$(dirname "$0")/.."; pwd)"

echo "Running image $IMAGENAME:$VERSION ..."

docker run -it \
    --name ros_grpc --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
    -e DISPLAY=$DISPLAY \
    --privileged \
    --net=host \
    -v $PLEXI_DIR:/home/robot/src/PLEXI \
    $IMAGENAME:$VERSION $2



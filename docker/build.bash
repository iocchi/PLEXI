#!/bin/bash

# Use  ./build.bash [Dockerfile] [version]

IMAGENAME=ros_grpc

DOCKERFILE=Dockerfile
if [ ! "$1" == "" ]; then
  DOCKERFILE=$1
fi

VERSION=`cat version.txt`
if [ ! "$2" == "" ]; then
  VERSION=$2
fi

echo "======================================="
echo "   Building $IMAGENAME:$VERSION "
echo "======================================="

docker build --network=host -t $IMAGENAME:$VERSION -f $DOCKERFILE .

docker tag $IMAGENAME:$VERSION $IMAGENAME:latest


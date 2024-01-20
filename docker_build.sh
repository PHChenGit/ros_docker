#!/bin/bash

#export HOST_UID=$(id -u)
#export HOST_GID=$(id -g)
DOCKER_IMAGE_REPO=hsun
DOCKER_IMAGE_TAG=1.1-noetic-cudagl-11.3.0-ubuntu2004
INSTALL_PX4=false

# Build the Docker image
docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --build-arg INSTALL_PX4=${INSTALL_PX4} \
  -t $DOCKER_IMAGE_REPO:$DOCKER_IMAGE_TAG .

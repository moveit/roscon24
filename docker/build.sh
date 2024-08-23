#!/bin/bash

IMAGE_NAME=roscon-2024-moveit2-workshop

set -xe

docker build -t $IMAGE_NAME .

docker save $IMAGE_NAME | pv | gzip >docker/docker-image.tar.gz

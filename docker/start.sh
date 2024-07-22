#!/bin/bash

DOCKER_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $DOCKER_DIR/..

name=roscon-2024-moveit2-workshop

if [ "$(id -u)" == 0 ]; then
  echo "ERROR: you are running this as root. This doesn't work with this container. Please run this script without root." >&2
  echo "If you can't use Docker without root, follow the instructions here: " >&2
  echo "https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user" >&2
  exit 1
fi

docker stop $name >/dev/null 2>&1
docker rm -f $name >/dev/null 2>&1
while docker container inspect $name >/dev/null 2>&1; do
  sleep 0.1
done

set -x

docker run \
  -d \
  --device=/dev/dri \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --volume=$(pwd):/code \
  --ulimit=rtprio=98 \
  --cap-add=SYS_NICE \
  --init \
  --rm \
  -e "DISPLAY=${DISPLAY}" \
  -e "QT_X11_NO_MITSHM=1" \
  -e "HOST_UID=$(id -u)" \
  -e "HOST_GID=$(id -g)" \
  --name=$name \
  $name

set +x

if [ "$?" == "0" ]; then
  echo "Started container $name. To enter the container, run the command:"
  echo
  echo "  docker/shell"
else
  echo "Error: failed to start container $name. did you import the image?" >&2
  echo "" >&2
  echo "Try running docker/fetch before running this again" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash

if [ -d /code ]; then
  cd /code
fi

colcon() {
  if [ "$(pwd)" == "/code" ]; then
    echo "ERROR: Please do not run colcon commands in /code." >&2
    echo "       Instead, run it in one of the exercise directories." >&2
    return 1
  fi

  /usr/bin/colcon "$@"
}

FROM ros:jazzy-ros-base

ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV ROS_DOMAIN_ID=24

RUN set -xe; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update; \
    apt-get install -y \
      # ROS dependencies
      ros-jazzy-rviz2 \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-rmw \
      ros-jazzy-lifecycle-msgs \
      ros-jazzy-rclcpp \
      ros-jazzy-rcl-lifecycle \
      ros-jazzy-rosidl-typesupport-cpp \
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-ament-lint-auto \
      ros-jazzy-ament-lint-common \
      ros-jazzy-mimick-vendor \
      ros-jazzy-performance-test-fixture \
      ros-jazzy-rcpputils \
      ros-jazzy-rcutils \
      ros-jazzy-test-msgs \
      ros-jazzy-ament-cmake-ros \
      ros-jazzy-ament-index-cpp \
      ros-jazzy-builtin-interfaces \
      ros-jazzy-rcl-interfaces \
      ros-jazzy-rosgraph-msgs \
      ros-jazzy-rosidl-runtime-cpp \
      ros-jazzy-rosidl-typesupport-c \
      ros-jazzy-libstatistics-collector \
      ros-jazzy-rcl \
      ros-jazzy-rcl-yaml-param-parser \
      ros-jazzy-statistics-msgs \
      ros-jazzy-tracetools \
      ros-jazzy-ament-cmake-gmock \
      ros-jazzy-ament-cmake-google-benchmark \
      ros-jazzy-rmw-implementation-cmake \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-ament-cmake-gen-version-h \
      python3-dev \
      ros-jazzy-std-msgs \
      ros-jazzy-ament-cmake \
      ros-jazzy-class-loader \
      ros-jazzy-composition-interfaces \
      ros-jazzy-launch-testing \
      ros-jazzy-rosidl-runtime-c \
      ros-jazzy-action-msgs \
      ros-jazzy-rcl-action \
      # MoveIt 2
      ros-jazzy-moveit \
      # UR
      ros-jazzy-ur \
      # Exercise dependencies
      ros-jazzy-geometric-shapes \
      ros-jazzy-moveit-msgs \
      ros-jazzy-shape-msgs \
      ros-jazzy-std-msgs \
      ros-jazzy-tf2-geometric-msgs \
      # Utility tools
      sudo \
      git \
      rsync \
      gdb \
      sshpass \
      clang-format-14 \
    ; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*;

# TODO: If using MTC, put it into this folder and built it
# COPY vendor /opt/code/vendor

# TODO: Consider pre-building all exercises such that workshop attendees only will need an incremental build

COPY docker/bin /opt/bin
COPY docker/profile.d/custom.sh /etc/profile.d/custom.sh

ENTRYPOINT ["/opt/bin/entrypoint"]

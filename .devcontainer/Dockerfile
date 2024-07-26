FROM roscon-2024-moveit2-workshop:latest

ARG WORKSPACE
ARG USERNAME=user

# If using VSCode devcontainers, users will need to build the devcontainer
# We also lack the ability to use the custom start script to pass in the host UID and GID
# TODO: Need to test this on a computer with a non-1000 UID user
RUN set -xe; \
    deluser ubuntu; \
    adduser --gecos "" --disabled-password $USERNAME; \
    usermod -a -G sudo $USERNAME; \
    usermod -a -G video $USERNAME; \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudoers;

RUN set -xe; \
    echo "export CONTAINER_WORKSPACE_FOLDER=${WORKSPACE}" > /etc/default/container-workspace-folder; \
    chsh -s /bin/bash $USERNAME

COPY ros-activate.sh /etc/profile.d/ros-activate.sh

# ROSCon 2024 Workshop: Manipulate With MoveIt Like a Pro

This repository hosts content for the **Manipulate With MoveIt Like a Pro** workshop for ROSCon 2024.

## Workshop Requirements

Please bring a laptop running Ubuntu 22.04 or Ubuntu 24.04 with Docker installed to the workshop to participate in the exercises.

To install Docker, see the official Docker instructions to [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

Officially, we test with **Ubuntu 24.04** and **Ubuntu 24.04**.
The Docker setup may be compatible with Mac/Windows, but please note that this is an untested configuration.

## Workshop Setup

This workshop requires some initial setup - this should take less than 10 minutes.
Please try to follow these steps to set up your laptop before attending the workshop to ensure the smoothest experience.
If you run into any issues during this process, please file an [issue](https://github.com/moveit/roscon24/issues) in this repository.

### Docker

All workshop exercises will be run from a Docker container.
**Please download the provided image before the workshop begins, as we canot guarantee the network performance at the conference.**

Before getting the image, you must have Docker installed on your machine.
You can install Docker by following [the official instructions](https://docs.docker.com/engine/install/ubuntu/).
At the end, confirm that you see the following output when running the following commands:

```bash
$ docker version | grep -A2 Client
Client: Docker Engine - Community
 Version:           27.0.2
 API version:       1.46

$ docker version | grep -A2 Server
Server: Docker Engine - Community
 Engine:
  Version:          27.0.2
```

Note that `Version` may be different than the above example output.

To use Docker without `sudo`, follow the Docker [post-install instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

#### Getting the image BEFORE the workshop

1. Download the image file from [the latest release](https://github.com/moveit/roscon24/releases). The file is called `docker-image.tar.gz`
2. Clone the workshop repository with `git clone https://github.com/moveit/roscon24.git`.
3. `cd` into the `roscon2024` repository
4. Use the fetch script to get the image: `docker/fetch ~/Downloads/docker-image.tar.gz` 

Note: do NOT use `docker import` or `docker load`. Please use the `docker/fetch` script above.

#### Docker Usage

**If you would like to use VS Code and its dev container system, you can skip this section and go directly to [VSCode Dev Containers](#vscode-dev-containers).**

##### Starting the Docker container

After importing the Docker image, you can start the Docker container via the special shell file `docker/start.sh`. To do this:

  1. `cd` into this repository.
  2. `docker/start.sh`

This should start the Docker container and it will mount this repository into the `/code` directory inside the container.
As a result, changes to the repository in the host will be reflected in the container.

##### Log into the Docker container

After starting the Docker container, you can login to the Docker container using the special `docker/shell` script:

  1. cd into this repository.
  2. `docker/shell`

You should be greeted with something like the following:

```
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.

user@a1abc5c215e9:/code$ 
```

This script logs you into an user named `user` which has the same UID and GID as your host user.
This eliminates some of the permission errors you may encounter with using Docker.
It also allows you to run GUI programs.

#### VSCode Dev Containers

This repositry contains a .devcontainer folder to work with the VSCode [Remote Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension.
This setup relies on the image imported above using the `docker/fetch` script.
If you haven't performed the image import step, launching VSCode with dev containers will not work, as it will fail to find the image.

Once you imported the image, you can simply open VSCode with dev containers in this repository.
You can then use the VSCode terminal inside the container.

#### Building the Docker Image

For the purposes of the workshop, please use the image provided in [the lastes release](https://github.com/moveit/roscon24/releases).

To build the image, use the `docker/build.sh` script:

  1. `cd` into this repository.
  2. `docker/build.sh`

This will build the image, tag it as `roscon-2024-moveit2-workshop`, and save the image as a compressed tarball. The `docker/fetch` script can be used to uncompress and load this output.

### Running the robot

After logging into the Docker container, you can run the UR robot to check if everything is working. To do this:

```bash
cd exercise1
colcon build
source install/setup.bash
ros2 launch exercise1-1 ur.launch.py
```

This should open an RViz window with the UR visible.

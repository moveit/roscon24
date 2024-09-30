### Building the Docker Image

**For the purposes of the workshop, please use the image provided in [the lastest release](https://github.com/moveit/roscon24/releases).**
**Workshop attendees should skip these steps as they do not need to build the image if they use the provided release.**

These steps are here for reference only to build the Docker image and are not requried for and should not be done for the workshop itself.

Running the build script is a supported workshop activity.
Please proceed with building the Docker image only if you are certain of your understanding and are prepared to handle potential issues that may arise.

The build script requries `pv`: `sudo apt install pv`

To build the image, use the `docker/build.sh` script:

  1. `cd` into this repository.
  2. `docker/build.sh`

This will build the image, tag it as `roscon-2024-moveit2-workshop`, and save the image as a compressed tarball. The `docker/fetch` script can be used to uncompress and load this output.

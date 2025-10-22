# Docker Instructions

This container provides a reproducible environment for the "ROS 2 Testing: A Practical Survival Guide" workshop, based on the `osrf/ros:jazzy-desktop-full` image.

It includes development tools, linters, Google Test/Mock, and all the necessary utilities to follow the exercises (using rosdep to install all the dependencies).

## 🚀 Building the Image and Running the Container

To build the image, use the [build.sh](./build.sh) script provided in this repository:

```bash
./docker/build.sh
```

Or, if the user wants to build the image and run the container directly, the [run.sh](./run.sh) script can be used with the corresponding flag:

```bash
./docker/run.sh --build
```

It's possible to get information about the available flags for both scripts using the `--help` or `-h` option:

```bash
./docker/build.sh --help
./docker/run.sh --help
```

Once the container is started, the entrypoint is located at the root of the ROS 2 workspace (~/ws).

After shutdown, the user will be asked whether to save the changes made inside the container or not to the image (performing a `docker commit` internally).

If at any moment it's required to start a new session to the running container, it can be done by executing:

```bash
docker exec -it ros2-testing-workshop-roscon-es-25-container bash
```

> [!NOTE]
> The name of the container might differ if a different name is provided in the script, so be careful about that.

<!-- TODO: Add Troubleshoot section? Common problems? -->

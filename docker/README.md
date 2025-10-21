# ROS 2 Testing: A Practical Survival Guide — Docker

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

After shutdown, the user will be asked if he/she wants to save the changes made inside the container to the image (performing a `docker commit` internally).

<!-- TODO: Add Troubleshoot section? Common problems? -->

# Docker Instructions

This container provides a reproducible environment for the "ROS 2 Testing: A Practical Survival Guide" workshop, based on the `ros:jazzy-ros-core` image.

It includes development tools, linters, Google Test/Mock, and all the necessary utilities to follow the exercises (using rosdep to install all the dependencies).

## 🚀 Building the Image and Running the Container

You can either build the **image locally** or pull it directly from **DockerHub**.

### Pull from DockerHub (recommended)

The prebuilt image is available at:

```bash
docker pull ekumenlabs/ros2-testing-workshop-roscon-es-25:jazzy
```

This is the easiest and fastest option.

### Build the Image Locally

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

## 📦 Container Usage

Once the container is started, the entrypoint is located at the root of the ROS 2 workspace (`~/ws`).

After exiting the container, you’ll be asked whether you want to save your changes back to the image (internally performing a `docker commit`).

If you need to reconnect to a running container:

```bash
docker exec -it ros2-testing-workshop-roscon-es-25-container bash
```

> [!NOTE]
> The name of the container might differ if a different name is provided in the script, so be careful about that.

<!-- TODO: Add Troubleshoot section? Common problems? -->

# isaac_roscon_demo

Demo docker for ROScon 2022

# Install 

There are two steps to follow, Install FoxGlove and Install Isaac ROS

## Foxglove

Download the latest [foxglove](https://foxglove.dev/download) version for ARM64

```
sudo apt install ./foxglove-studio-*.deb
sudo apt update
sudo apt install -y foxglove-studio
```

## Isaac ROS

Install essential software on host

```
sudo apt install -y git-lfs python3-vcstool
mkdir -p $HOME/isaac_ros-dev/ros_ws/src
cd $HOME/isaac_ros-dev/ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/rbonghi/isaac_roscon_demo.git
```

Make workspace folder and pull all packages

```
cd $HOME/isaac_ros-dev/ros_ws
# wget -L https://raw.githubusercontent.com/rbonghi/isaac_roscon_demo/main/demo.rosinstall -o demo.rosinstall
cp src/isaac_roscon_demo/demo.rosinstall demo.rosinstall
vcs import src < demo.rosinstall
vcs pull src
```

Build docker and run docker

```
cd $HOME/isaac_ros-dev/ros_ws/src/isaac_ros_common
cp docker/realsense-dockerfile-example/.isaac_ros_common-config scripts/
bash scripts/run_dev.sh $HOME/isaac_ros-dev/ros_ws
```

# Docker

From this stage you work mainly from the docker container, if your are not on your docker container watch the installation above

## Build ROS2 packages

Run from docker container

```
colcon build --symlink-install --merge-install
```

## Convert model

Following [isaac_ros_dnn_stereo_disparity](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_disparity) convert the DNN stereo disparity model

```
/opt/nvidia/tao/tao-converter -k ess -t fp16 -e /workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/ess.engine -o output_left /workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_disparity/resources/ess.etlt
```

## Run

Launch script from docker container

```
source install/setup.bash
ros2 launch isaac_roscon_demo demo.launch.py
```

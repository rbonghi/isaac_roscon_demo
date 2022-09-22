# isaac_roscon_demo

Demo docker for ROScon 2022

# Install and setup

Install essential software on host

```
sudo apt install -y git-lfs python3-vcstool
mkdir -p $HOME/workspaces/isaac_ros-dev/ros_ws/src
cd $HOME/workspaces/isaac_ros-dev/ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/rbonghi/isaac_roscon_demo.git
```

Make workspace folder and pull all packages

```
cd $HOME/workspaces/isaac_ros-dev/ros_ws
wget -L https://raw.githubusercontent.com/rbonghi/isaac_roscon_demo/main/demo.rosinstall -o demo.rosinstall
vcs import src < demo.rosinstall
vcs pull src
```

Build docker and run docker

```
cd $HOME/workspaces/isaac_ros-dev/ros_ws/src/isaac_ros_common
cp docker/realsense-dockerfile-example/.isaac_ros_common-config scripts/.isaac_ros_common-config 
bash scripts/run_dev.sh $HOME/isaac_ros-dev/ros_ws
```

# Docker

## Build ROS2 packages

```
colcon build --symlink-install --merge-install
```

## Run

Launch script

```
source install/setup.bash
ros2 launch isaac_roscon_demo demo.launch.py
```




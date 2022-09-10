# isaac_roscon_demo

Demo docker for ROScon 2022

# Install and setup

Install essential software on host

```
sudo apt install -y git-lfs 
```

Make workspace folder and pull all packages

```
mkdir -p $HOME/workspaces/isaac_ros-dev/ros_ws/src
cd $HOME/workspaces/isaac_ros-dev/ros_ws
wget -L https://raw.githubusercontent.com/rbonghi/isaac_roscon_demo/main/demo.rosinstall -o demo.rosinstall
vcs import src < demo.rosinstall
vcs pull src
```

# Build from docker

Start docker 

```
scripts/run_dev.sh
```

```
colcon build --symlink-install --merge-install
```

# Run

Launch script

```
ros2 launch isaac_roscon_demo demo.launch.py
```




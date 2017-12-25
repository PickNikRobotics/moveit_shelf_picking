# MoveIt! Shelf Picking Benchmark

Benchmarking suite for dual arm manipulation

Developed by Dave Coleman at [PickNik Consulting](http://picknik.ai/)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-kinetic-moveit_shelf_picking

### Build from Source

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the following build tools.
```
sudo apt-get install python-wstool python-catkin-tools clang-format-3.8
```

2. Re-use or create a catkin workspace:
```
mkdir -p ~/ws_catkin/src
cd ~/ws_catkin/src
```

3. Download the required repositories and install any dependencies:
```
git clone git@github.com:PickNikRobotics/moveit_shelf_picking.git
wstool init .
wstool merge ./moveit_shelf_picking/moveit_shelf_picking.rosinstall
wstool update
rosdep install --from-paths . --ignore-src --rosdistro kinetic
```

4. Configure and build the workspace:
```
cd ..
catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

5. Source the workspace.
```
source . ./devel/setup.bash
```

## Usage

Start Rviz:

    roslaunch moveit_shelf_picking baxter_visualize.launch

Run benchmark:

    roslaunch moveit_shelf_picking run_benchmark.launch

If you are prompted to press next to continue, find the "MoveItDashboard" panel in Rviz and click Next.

## Configuration

There are lots of settings that can easily be tweaked in the following file:

    moveit_shelf_picking/config/config_baxter.yaml

In particular, pay attention to the ``visualize/`` configurations for more indepth view of what is going on.

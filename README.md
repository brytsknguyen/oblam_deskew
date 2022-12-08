# OBLAM Assignment: Deskewing lidar scan with IMU propagated states
<!-- via Continuous-time Optimization -->

# Course page

The course materials and instructions can be found at [KTH Canvas](https://canvas.kth.se/courses/40649) .

# Prerequisite

The software was developed on the following dependancies
1. [Ubuntu 20.04](https://releases.ubuntu.com/20.04/)
2. [ROS Noetic](http://wiki.ros.org/noetic/Installation)
3. [robot_localization package](http://wiki.ros.org/robot_localization) (with apt-get)
4. PCL libary (built-in of Ubuntu 20.04)

The code was editted on VS Code with [#region folding add-on](https://marketplace.visualstudio.com/items?itemName=maptz.regionfolder) for tidier view.

# Installation
Please install all dependencies first. Afterwards, create a ros workspace, clone the package to the workspace, and build by `catkin build` or `catkin_make`, for e.g.:

```
mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/brytsknguyen/oblam_deskew
cd ..; catkin build
```
# Download Data
Please download the data [here](https://kth-my.sharepoint.com/:f:/g/personal/tmng_ug_kth_se/Em6tmNkGrJhHubL2o8PJ6gcB_iiLQJEIXDyOquTdF0o6jQ?e=JKDW3X)

Declare the path to the data in the launch file run_deskew.launch.

# Assignment
Go to the function PropagateIMU() and DeskewByImuPropagation() and add the codes to complete the motion compensation of the pointclouds. If sucess you should see the deskewed pointcloud on the right.

<p align="center">
    <img src="docs/deskew.gif" alt="mcd ntu daytime 04" width="99%"/>
</p>

# Happy Studying!
<img src="docs/thinkingguy.png" alt="drawing" width="300"/>

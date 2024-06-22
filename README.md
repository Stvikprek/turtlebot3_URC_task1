# turtlebot3_mapping_and_navigation

A repo with a ROS1 workspace that contains all packages required to simulate TurtleBot3 in a Gazebo world, map said world and perform SLAM and also perform autonomous navigation. URC Task-1

## Setup and dependencies
The repo uses the following:
- Ubuntu 20.04
- ROS-noetic
- Gazebo for simulation

Before we begin, make sure you have the `ros-noetic-gazebo-ros-pkgs` installed or the simulation will be unable to communicate with any external nodes (which is required for SLAM and navigation).
```
sudo apt install ros-noetic-gazebo-ros-pkgs
```
Also run the following command to install more dependencies used by Turtlebot3 and the TurtleBot3 packages themselves.
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
  ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3

```
This repo utilizes files from the following repositories:
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- The map small_house from [Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)






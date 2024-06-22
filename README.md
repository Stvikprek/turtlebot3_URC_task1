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



## Usage
- First, make sure to put the required files from [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) and [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) repos into your workspace and run `catkin_make` in order to build the files
### Launching a World
- First, from the Gazebo Worlds repo, paste the models folder into `<workspace>/src/turtlebot3_simulations/turtlebot3_gazebo/models` and into `~/.gazebo/models` and make sure to run `export GAZEBO_MODEL_PATH=~/.gazebo/models/models` in each shell session you launch the Gazebo world in.
- Next paste the `small_house.world` file into `turtlebot3_gazebo/worlds` and remove
  ```
  <include>
      <pose>-3.5 -4.5 0.0 0.0 0.0 1.58</pose>
      <uri>model://turtlebot3_waffle_pi</uri>
  </include>
  ```
  from the world file as the launch file we will be using will be spawning in our required turtlebot3 model.
  #### The launch file
  - Named `new_house.launch`
  ```
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="-3.5"/>
  <arg name="y_pos" default="-4.5"/>
  <arg name="z_pos" default="0.0"/>
  ```
  - Defines arguments that determine the model of the robot to be loaded in and the default x,y and z coordinates of the robot in our world. The defaults for the coordinates have been changed from the standard 0,0,0 to avoid the robot spawning inside obstacles
  ```
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/small_house.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  ```
  - Includes the base file required to launch our world, next we set a value to the arg `world_name` to the path of `small_house.world` (P.S. the world files are basically just XML definitions of the 3D world we will be simulating)
  ```
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  ```
  - Defines and describes the robot using a urdf file(another XML definition file) shortened via xarco (XML macro language)
  ```
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
  ```
  - Starts the node responsible for spawning the robot model using the initial positions and description written above
  #### Actually Launching the World
  - Run
  ```
  export TURTLEBOT3_MODEL=waffle
  roslaunch turtlebot3_gazebo new_world.launch
  ```
  to start the Gazebo simulation (with the waffle model), it may take a while to load
### SLAM and Mapping
  - Assuming you have the world from before running, in a new terminal run
  ```
  export TURTLEBOT3_MODEL=waffle
  roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=<your choice>
  ```
  This launch file is reponsible for starting the nodes responsible for performing Simultaneous Localization and Mapping (SLAM) for our simulated robot, with our preferred method for the same (default being gmapping). The launch file mainly starts the       
  `turtlebot3_slam_gmapping` node which does all the SLAM computations by gathering simulated LiDAR scan data from the `/scan` topic and publishes it to the `/map` topic. This topic is subscribed to by rviz, which displays the gathered map.
  (NOTE: Algorithms like gmapping require lots of parameters to perform their estimation, all these parameters are loaded in via a .yaml file using a `<rosparam>` tag in their specific launch file)
  - Using the `turtlebot3_teleop_key` node of the `turtlebot3_teleop` package , the robot can be controlled with the keyboard to gather a map of the entire world
  - Using the `map_server` node and running `rosrun map_server map_saver -f <path>` will save a map.pgm and map.yaml file in the specified path. `map_server` subscribes to the `/map` topic to do so.
### Navigation
  - In a new terminal with the simulation running, run
  ```
  export TURTLEBOT3_MODEL=waffle
  roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<path to map file>
  ```
  This launch 
  

  


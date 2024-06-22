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


  


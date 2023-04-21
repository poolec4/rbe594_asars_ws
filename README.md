# rbe594_asars_ws

An Autonomous Search and Rescue System (ASARS) created for the Spring 2023 RBE 594 Capstone project.

## Installation

### Option 1:

Run `instal.sh` (TBD)

### Option 2: 
Clone repository:
```bash
cd ~/
git clone https://github.com/poolec4/rbe594_asars_ws.git
cd ~/rbe594_asars_ws/
git submodule init
git submodule update
```

Manually install dependencies:

```bash
sudo apt-get install ros-noetic-husky-simulator
sudo apt-get install ros-noetic-teb-local-planner
sudo apt-get install python3-pcl pcl-tools
```

These may be needed as well:
```bash
sudo apt-get install ros-[distro]-tf2-sensor-msgs
sudo apt-get install ros-[distro]-navigation
sudo apt-get install ros-[distro]-costmap-converter
sudo apt-get install ros-[distro]-mbf-costmap-nav
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-move-base-msgs
```

Add exports:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/rbe594_asars_ws/src/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/rbe594_asars_ws/src/gazebo_models_worlds_collection/worlds
```

Build catkin environment:
```bash
catkin_make 
source ~/rbe594_asars_ws/devel/setup.bash
```

## Full Simulation

This will run the full ASARS simulation. It will begin by generating the world and spawning the victims. Then, it will begin the UAV mapping and scanning operations. Once this is complete, the occupancy grid is generated and the AGV planning commences.
```bash
roslaunch rbe594_asars main_asars_world.launch
```

## Hector Test

This will run Hector in a test world and start the teleop menu. It will also open the sensor visualization in RViz. This has the downward facing LIDAR for ground mapping:
```bash
roslaunch rbe594_asars hector_world.launch
```

## Husky Test

TBD


## World Test

Open the test environment with LIDAR unit:
```bash
roslaunch rbe594_asars my_world.launch
```


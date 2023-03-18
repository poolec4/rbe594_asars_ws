# rbe594_asars_ws

An Autonomous Search and Rescue System (ASARS) created for the Spring 2023 RBE 594 Capstone project.

## Installation

Clone repository:
```bash
cd ~/
git clone https://github.com/poolec4/rbe594_asars.git
cd ~/rbe594_asars_ws/
git submodule init
git submodule update
```

Build catkin environment:
```bash
catkin_make 
source ~/rbe594_asars_ws/devel/setup.bash
```

## World Test

Open the test environment with LIDAR unit:
```bash
roslaunch rbe594_asars my_world.launch
```

## Hector Test

This will run Hector in a test world and start the teleop menu. It will also open the sensor visualization in RViz. This has the downward facing LIDAR for ground mapping:
```bash
roslaunch rbe594_asars hector_world.launch
```
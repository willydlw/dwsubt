# ros package dwsubt

Currently supports 

1. left wall following for the X1 robot, sensor configuration 1. Only works for the tunnel circuit practice.

2. Freespace visualization of laser xy data points.


## Build Instructions
Clone the repository in the src directory of a catkin workspace.

```
cd ~/catkin_ws/src
git clone https://github.com/willydlw/dwsubt.git
catkin_make 
```

## Wall Following

Base controller node implements a simple state machine that moves robot to entrance and then begins driving toward the largest freespace opening detected in laser data scan. This
"works" for the X1 and X2 robots in the tunnel circuit practice worlds.

This code is in development to create a working wall-following solution for ground vehicle robots in the cave circuit. 


### Terminal 1

```
source ~/subt_ws/install/setup.bash
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1

```

### Terminal 2

In the catkin workspace directory

```
source ./devel/setup.bash
roslaunch dwsubt laser_obstacle_avoid.launch
```


## Freespace Visualizer 

Currently working on the free_space_visualizer node to plot the detected free space openings. Want a verification tool that allows us to see the openings are correctly detected and show which opening is selected as a goal point.

The current state plots the laser scan in its xy coordinate form. Next step is plotting the free space openings.


### Running the program 

Current testing uses the simple_cave_01 and the X1_SENSOR_CONFIG_1. The laser scan parameters are defined in laser_planar5m_params.yaml

### Terminal 1

```
source ~/subt_ws/install/setup.bash
ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1
```

### Terminal 2

```
source ./devel/setup.bash
roslaunch dwsubt freespace_visualizer.launch
```


The image below shows the expected output.


![Laser XY data points](./images/xy_data_2020-06-06.png "laser data")



# Cartographer Notes

How did I get cartographer running? Luck and help from the subt_hello_world cartographer posts and repository. <br>


## Install Cartographer Locally

The following instructions pertain to running the simulation and cartographer locally on my machine, not in a docker container.

```
sudo apt-get install ros-melodic-cartographer ros-melodic-cartographer-ros ros-melodic-cartographer-rviz
```
<br><br>

## Cartographer launch and lua files

Having completed the subt_hello_world cartographer tutorial for docker, it provided insight and program files to guess at a local installation. 

https://github.com/osrf/subt_hello_world/blob/master/posts/02_docker_and_slam.md 

1. Copy the following files from the subt_hello_world/subt_solution_launch directory to your ros package directory.

launch/cartographer.launch
config/cartographer/cartograhper_2d.lua
config/cartographer/cartograhper_3d.lua
config/cartographer/cartograhper_X1.lua

Example: my ros package name is dwsubt. I put the cartographer.launch file in dwsubt/launch/. The lua files were copied to dwsubt/config/cartographer/

I renamed cartographer.launch to cartographer3d.launch and then created a cartographer2d.launch file that references the cartographer_2d.lua file.

Made the following changes to the cartographer_2d.lua file:
- tracking_frame = name,
- published_frame = name,

Removed the remapping of the points topic to scan in the cartographer2d.launch file as the 2d LaserScan topic is usually named scan. **TODO** Verify this is correct. Did not yet find correct topic name in cartographer documentation.
<br> <br>

2. Add the cartographer_ros dependency to the package.xml file

```
<depend>cartographer_ros</depend>
```
<br><br>

## Edit the cartographer.launch files

Replace subt_solution_launch with the name of your package, as shown below. 

```
-configuration_directory $(find dwsubt)/config/cartographer
```

3. catkin_make 

Run catkin_make or catkin_make install. Thought this was necessary due to updating the package.xml dependency and adding config files.

<br><br>

## Test Cartographer 2D

I used the simple_cave_01, joystick teleop, and rviz to test cartographer.

<br>

### Terminal 1

```
source ~/subt_ws/install/setup.bash
ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1
```

### Terminal 2

```
source ~/subt_ws/install/setup.bash
roslaunch subt_example teleop.launch
```


### Terminal 3 

```
source ~/catkin_ws/devel/setup.bash
roslaunch dwsubt cartographer2d.launch name:=X1
```


### Terminal 4

```
source ~/catkin_ws/devel/setup.bash
rviz dwsubt cartographer2d.rviz
```


### Terminal 5

If you want to run the blob detector, in case you come across a red backpack:

```
source ~/catkin_ws/devel/setup.bash
rosrun simple_detect find_red_blob.py
``` 


**TODO** Test cartographer3d.launch file with a 3D Lidar scanner
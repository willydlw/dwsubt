### Build Instructions
Clone the repository in the src directory of a catkin workspace.

```
cd ~/catkin_ws/src
git clone https://github.com/willydlw/dwsubt.git
catkin_make 
```



### Terminal 1


```
source ~/subt_ws/install/setup.bash
ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1

```


ign launch -v 4 cave_circuit.ign worldName:=simple_cave_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1


### Terminal 2

In the catkin workspace directory

```
source ./devel/setup.bash
roslaunch dwsubt laser_obstacle_avoid.launch
```


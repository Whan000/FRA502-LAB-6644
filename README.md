# FRA502-LAB-6644
Phuriwat Kasamesookphaisal 66340500044 (Whan)

# LAB3 - Eater vs. Killer: Spawn–Forage–Pursuit (SFP) with RViz2 and Turtlesim+

An interactive lab implementing user-spawned “pizza” targets, service-driven turtle lifecycle management, and RViz2 click-to-pose evasion/pursuit behaviors using `turtlesim`.

---
## Part 1 - Build All Nodes

**Project Tree**



```
FRA502-LAB-StudentID/
├── src
│   ├── controller_interfaces
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── srv
│   │       ├── SetMaxPizza.srv
│   │       ├── SetParam.srv
│   ├── lab3
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   ├── lab3/
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── eater.py
│   │   │   ├── killer.py
│   │   └── src/
|   └── turtlesim_plus/
└── README.md

```

- Implement **three main nodes**, **three custom service types** and **one launch file** to satisfy the architecture:  
  - `eater` (handles pizza detection and movement to spawned pizza)  
  - `killer` (pursues eater after all pizzas are consumed)  
  - `SetMaxPizza.srv` (custom service type to set maximum pizza count)
  - `SetParam.srv` (custom service type to set `kp_linear` and `kp_angular`)
  - `lab3_bringup.launch.py` (launches all nodes and turtlesim+)

---
### Package `controller_interfaces` requirements
### What this package **must contain**
- **`SetMaxPizza.srv`** : use for setting the maximum number of pizzas that can be spawned.  
  - Define the maximum number of pizzas by this service type : `std_msgs/Int64` with variable name : `max_pizza` and 
  - Return response as this service type `std_msgs/String` with variable name : `log` 
  - Value of `log` must always show the maximum number of pizzas that can be spawned with a status **success** when value of `max_pizza` from service call `>` current `max_pizza` of Node `eater` . Otherwise  show status **failed**.
- **`SetParam.srv`** (custom service type to set `kp_linear` and `kp_angular`)
  - Define the `kp_linear` by this service type : `std_msgs/Float64` with variable name : `kp_linear` 
  - Define the `kp_angular` by this service type : `std_msgs/Float64` with variable name : `kp_angular` 
  - This custom service has no response.

### Launch file requirement
### What this launch file **must do**
- `eater` and `killer` nodes must be launched.
- `turtlesim_plus` must be included in the **launch file**.
- Topics must be remapped as necessary.
- Parameters for all nodes must be set.
- All nodes must be properly configured before starting.
- Able to configure frequency of `eater` node via **launch file** parameter with name `sampling_frequency` and default value `100.0` (Hz).
- Able to configure frequency of `killer` node via **launch file** parameter with name `sampling_frequency` and default value `100.0` (Hz).
- Able to configure target `eater` turtle via **launch file** parameter of `killer` node with name `eater_name` and default value `eater_turtle`.

### Node `eater` requirement
### What this node **must do**
- **Control XXXX to target position** by publishing velocity to `/XXXX/cmd_vel` using pose feedback from `/XXXX/pose` for navigation.
- **Accept click targets** from `/mouse_position`  and convert to turtlesim coordinates.
- **Forage mode operation** by continuously spawning pizzas at clicked locations via spawn service and eating them in order-by-order sequence using `/XXXX/eat` service - must handle simultaneous pizza spawning while executing eating sequence.
- **Evade mode operation** (when all pizzas eaten) robot must move to the target from `/mouse_position` .
- Able to use **ros2 param** to set and get parameter, name `sampling_frequency` with default value `100.0` (Hz). 
- Able to config **controller gain** via `/XXXX/set_param` service with request type `SetParam.srv`.
- Able to config **maximum pizza** via `/XXXX/set_max_pizza` service with request type `SetMaxPizza.srv`.

### Node `killer` requirement  
### What this node **must do**
- **Control YYYY to target position** by publishing velocity to `/YYYY/cmd_vel` using pose feedback from `/YYYY/pose`.
- **Track eater target** by subscribing to `/XXXX/pose` as moving pursuit target after all pizzas are eaten.
- **Terminate on capture** by calling `/remove_turtle` service when close enough to eater, then stop motion.
- Able to use **ros2 param** to set and get parameter, name `sampling_frequency` with default value `100.0` (Hz). 
- Able to config **controller gain** via `/XXXX/set_param` service with request type `SetParam.srv`.

### What the `eater` and `killer` Nodes Must Do Together

- Both nodes must be able to configure their controller gains via the `/XXXX/set_param` service for the `eater` node and the `/YYYY/set_param` service for the `killer` node.  
- While the `killer` is tracking the `eater`, if the **pizza** of `eater` is modified through a service call, the `killer` must adapt by waiting for the `eater` to finish eating all the pizza before continuing.  
- Use `/XXXX/eat_status` topic to monitor the eating status of the `eater` node.

### Student requirement
- **Modify the service and topic** of the `eater_node` so that the namespace can be defined according to the project structure as specified. The namespace will be named XXXX.
- **Modify the service and topic** of the `killer_node` so that the namespace can be defined according to the project structure as specified. The namespace will be named YYYY.
- **Perform a kill** on the turtle named `/turtle1` through a launch file.
- **Perform a spawn** for the turtle named `/XXXX` through a launch file, where XXXX will be the namespace name.
- **Perform a spawn** for the turtle named `/YYYY` through a launch file, where YYYY will be the namespace name.
- **Define the namespace** for the `eater_node` through a launch file according to the specified project structure.
- **Define the namespace** for the `killer_node` node through a launch file according to the specified project structure.

### How to run this project ???
To install and run this project user need to execute the following command in order.

1.Run this following command to clone the repository and built it using colcon feature after that source it.
```
cd ~/ git clone -b LAB3 https://github.com/Whan000/FRA502-LAB-6644.git && colcon build &&. install/setup.bash
```
After this your project have been built you can run it with ros2 command accordingly but some of the command need to be run in a seperated terminal dont forgot to source your files.

2.Run this following command launch and start the command being held in one single launch files.
```
ros2 launch lab3 lab3_bringup.launch.py
```

### Additional
You can run these command to call a service. (Set max pizza or set turtle parameter)
```
ros2 run rqt_service_caller rqt_service_caller
```
You can run these command to view work flow via graph using rqt_graph viewer
```
rqt_graph
```
In order to verify your working frequency run this command
```
ros2 topic hz <your topic>
```

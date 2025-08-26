# FRA502-LAB-6644
Phuriwat Kasamesookphaisal 66340500044 (Whan)

To install and run this code user need to execute the following command in order.

cd ~/<user-ws>
git clone -b LAB2 https://github.com/Whan000/FRA502-LAB-6644.git
colcon build
. install/setup.bash
--------------------------------------------------------------------------------
After this your project have been built you can run it with ros2 command accordingly but some of the command need to be run in a seperated terminal dont forgot to source your files.

ros2 run turtlesim_plus turtlesim_plus_node.py
ros2 run lab2 eater.py
ros2 run lab2 killer.py
ros2 service call /spawn_turtle turtlesim/srv/Spawn
ros2 run lab2 turtlesim_pose.py

You can this command to limit and setup the maximum pizza that turtle is going to be eaten.
ros2 topic pub --once /max_pizza std_msgs/msg/Int64 {"data: 10"}

And you can open rviz2 up reviewing odometry for both of the turtle (turtle1 and turtle2) by using this command and open a lab2.rviz locate under /src folder
rviz2
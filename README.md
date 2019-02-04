# Robotik-Praktikum
  You only need catkin_ws
1. Download ZIP and extract
2. open terminal
 - cd catkin_ws
 - catkin_make
 - source devel/setup.bash
 - roslaunch myarm_plugin myarm.launch &
 - rosrun keyboard_non_blocking_input keyboard_non_blocking_input_node
3. Your keyboard inputs get captured and move the arm 

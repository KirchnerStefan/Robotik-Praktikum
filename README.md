# Robotik-Praktikum

1. Download "myarm_plugin"
2. open terminal:  
-	cd <your path to>/myarm_plugin
-	mkdir build 
-	cd build 
-	cmake ../ 
-	make 
-	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<your path to>/myarm_plugin/build
- It is important to export the path and start gazebo from build-folder. 
  Otherwise gazebo will possibly not recognize the shared library for the plugin.
3. in new terminal:  
-	roscore &
-	gazebo --verbose ../myarm.world &
-  you should be able to see topics
5.   for using the input reader for keyboard:
- open new terminal 
- cd catkin_ws 
- catkin_make
- source ./devel/setup.bash
- rosrun keyboard_non_blocking_input keyboard_non_blocking_input_node
- the keyboard inputs are captured in this terminal 

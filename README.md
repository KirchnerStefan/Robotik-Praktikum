# Robotik-Praktikum

1. Download "myarm_plugin"
2. open terminal:  
-	cd <your path to>/myarm_plugin
-	mkdir build 
-	cd build 
-	cmake ../ 
-	make 
-	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<your path to>/myarm_plugin/build
It's important to export the path and start gazebo from build-folder. 
It will possibly not recognize the shared library for the plugin otherwise.
3. in new terminal:  
-	roscore &
-	gazebo --verbose ../myarm.world &
4. you should be able to see topics

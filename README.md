# Robotik-Praktikum
  You only need catkin_ws for manual control
1. Download ZIP and extract 
  - Install https://pypi.org/project/SpeechRecognition/ if you want to voicecontrol the gripper
2. open terminal
 - cd catkin_ws
 - catkin_make
 - source devel/setup.bash
 - roslaunch myarm_plugin myarm.launch &
 - rosrun myarm_plugin voice_node.py &     (only if you have "speech_recognition" installed) 
 - rosrun keyboard_non_blocking_input keyboard_non_blocking_input_node
3. Your keyboard inputs get captured and move the arm 
 - select joint with "q/a"
 - move joint forward/backward with "w/s"
 - activate voice_recognition with "d"    (only if you have "speech_recognition" installed) 
4. Sometimes the keyboard-reader may need a restart.
5. Voicecontrol may sometimes not act on first command.

If you want to use MoveIt for MotionPlanning you need urdf_test
1. Like above
2. open terminal
  - cd urdf_test
  - catkin_make
  - source devel/setup.bash
  - roslaunch myrobot_gazebo empty_world.launch
3. open new terminal
  - cd urdf_test
  - source devel/setup.bash
  - export LC_NUMERIC="en_US.UTF-8"
  - roslaunch myrobot_moveit_config moveit_planning_execution.launch
4. you can now drag the model in rviz and click plan and execute to let model move in Gazebo

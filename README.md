# Robotik-Praktikum
  You only need catkin_ws
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

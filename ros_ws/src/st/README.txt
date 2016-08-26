sample run:
											have it move.. 	and record data..	 with noise and gps..  for x secs..  with the tracker..  and a compromised path
roslaunch cwru_turtlebot one_robot.launch stationary:=false sensor_record:=true noisy:=true gps:=true sim_time:=240 state_tracker:=True comp_one:=True

DEPENDENCIES:
Kobuki
turtlebot
pudb (can comment out)
robot_localization


sudo apt-get install ros-indigo-kobuki ros-indigo-kobuki-core
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
sudo pip install pudb
sudo apt-get install ros-indigo-robot-localization
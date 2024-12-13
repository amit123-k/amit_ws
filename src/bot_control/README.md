This package contains a script inside scripts folder and a launch file inside launch folder.

The reading_laser.py file subcribes the /scan topic and and publish a new beam of laser scan data to /filtered_scan topic. This script only publishes the range of laser_scan which is field of view of a normal camera that is -60° to 60°(120°).

The filtered_rviz.launch.py file launches rviz and it also runs the reading_laser.py script that filters the laser range and it can be visualized in rviz window.
To run the launch file use the following command. **The robot.launch.py file should be run first in another terminal to make the robotmodel and the scan topic is active in the ros environment.Instruction is mentioned in bot_world package.

    ros2 launch bot_control rviz_filtered.launch.py

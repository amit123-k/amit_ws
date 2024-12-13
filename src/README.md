The whole project reflects the behaviour of a differential drive robot.It shows how to build its urdf file, define joints and links, use different sensors and finally how the robot behaves in a simulated environment. Here is written a brief overview of the three packages. Otherwise the package details and its usage process is written inside respective packages.

### The whole project contains three packages

#### bot_description:

Holds the URDF of the robot model along with all the sensors and its plugins.
Contains two launch files:
One spawns the robot model in the Gazebo simulator.
The other spawns the robot model in the RViz tool.

#### bot_world:

Contains the Gazebo world file.
Contains a launch file that launches the bot in the custom-designed world.

To visualise the robot in the gazebo world, run the following command

    ros2 launch bot_world robot.launch.py

#### bot_control:

Holds a script that filters out the 360-degree laser scan range of the LIDAR sensor and passes a new range of LIDAR that is -60 degrees to 60 degrees.
The script is run by a launch file that also opens the RViz window and shows the filtered response of the LIDAR.

To see the filtered laser response in rviz, run the following command after building and sourcing properly.

    ros2 launch bot_control filtered_rviz.launch.py
